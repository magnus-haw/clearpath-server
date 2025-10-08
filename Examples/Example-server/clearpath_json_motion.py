#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include "pubSysCls.h"
#include "json.hpp"   // nlohmann/json single header

using namespace sFnd;
using json = nlohmann::json;
using namespace std::chrono;

#define TIME_TILL_TIMEOUT 10000


// ----------------------------------------------------
// Load the motion sequence directly as JSON
// ----------------------------------------------------
json load_motion_sequence(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open sequence file: " + filename);
    }

    json j;
    try {
        f >> j;
    } catch (const std::exception &e) {
        throw std::runtime_error(std::string("Error parsing JSON file: ") + e.what());
    }

    if (!j.is_array()) {
        throw std::runtime_error("Expected a JSON array of commands in " + filename);
    }

    return j;
}

// ----------------------------------------------------
// Log telemetry (one node, one CSV stream)
// ----------------------------------------------------
void log_telemetry(std::ofstream &log, INode &node,
                   steady_clock::time_point t0) {
    double t = duration<double>(steady_clock::now() - t0).count();
    node.Motion.PosnMeasured.Refresh();
    node.Motion.VelMeasured.Refresh();
    log << std::fixed << std::setprecision(4)
        << t << "," << node.Motion.PosnMeasured.Value() << ","
        << node.Motion.VelMeasured.Value() << "\n";
}

#define ACC_LIM_RPM_PER_SEC	100000
#define VEL_LIM_RPM			700
#define MOVE_DISTANCE_CNTS	10000	
#define NUM_MOVES			5
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

// ----------------------------------------------------
// Main
// ----------------------------------------------------
int main() {
    try {
        // --- Load JSON motion sequence ---
		json sequence = load_motion_sequence("motion_sequence.json");
		std::cout << "Loaded " << sequence.size() << " steps from JSON.\n";

        // --- Discover and open hub ---
        std::vector<std::string> ports;
        SysManager::FindComHubPorts(ports);
        if (ports.empty())
            throw std::runtime_error("No SC hubs found");

        SysManager *mgr = SysManager::Instance();
        mgr->ComHubPort(0, ports[0].c_str());
        mgr->PortsOpen(1);
        IPort &port = mgr->Ports(0);

        size_t node_count = port.NodeCount();
        std::cout << "Found " << node_count << " node(s).\n";
        if (node_count == 0)
            throw std::runtime_error("No ClearPath nodes found");

        // --- Prepare nodes ---
        for (size_t i = 0; i < node_count; ++i) {
            INode &n = port.Nodes(i);
            n.EnableReq(false);
            mgr->Delay(100);
            n.Status.AlertsClear();
            n.Motion.NodeStopClear();
            n.EnableReq(true);

            double timeout = mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
            while (!n.Motion.IsReady()) {
                if (mgr->TimeStampMsec() > timeout)
                    throw std::runtime_error("Timeout enabling node " + std::to_string(i));
            }

            n.AccUnit(INode::RPM_PER_SEC);
            n.VelUnit(INode::RPM);
            std::cout << "Node " << i << " enabled.\n";
        }

        // --- Reference time (steady + system clock) ---
		auto t0 = steady_clock::now();
		auto wall_t0 = std::chrono::system_clock::now();
		std::time_t t0_c = std::chrono::system_clock::to_time_t(wall_t0);

		// Format timestamp (localtime)
		std::tm local_tm = *std::localtime(&t0_c);
		char timebuf[64];
		std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &local_tm);

		// --- Open telemetry files ---
		std::vector<std::ofstream> logs(node_count);
		for (size_t i = 0; i < node_count; ++i) {
			std::string fname = "telemetry_node" + std::to_string(i) + ".csv";
			logs[i].open(fname);
			if (!logs[i].is_open())
				throw std::runtime_error("Failed to open " + fname);

			logs[i] << "# Telemetry start (system clock): " << timebuf << "\n";
			logs[i] << "# steady_clock t0 reference point for relative times below\n";
			logs[i] << "time,pos,vel\n";
		}



        // --- Execute sequence ---
        for (size_t i = 0; i < sequence.size(); ++i) {
            const auto &cmd = sequence[i];
            int node_idx = cmd.value("node", 0);

            if (node_idx < 0 || node_idx >= (int)node_count) {
                std::cerr << "Warning: invalid node index " << node_idx
                          << " at step " << i + 1 << "\n";
                continue;
            }

            INode &node = port.Nodes(node_idx);
            std::ofstream &log = logs[node_idx];

            // --- Homing ---
            if (cmd.contains("home") && cmd["home"].get<bool>()) {
                std::cout << "[Step " << i + 1 << "] Node " << node_idx
                          << ": homing...\n";
                log_telemetry(log, node, t0); // mark before home
                if (node.Motion.Homing.HomingValid()) {
                    node.Motion.Homing.Initiate();
                    while (!node.Motion.Homing.WasHomed()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        log_telemetry(log, node, t0);
                    }
                } else {
                    std::cout << "  Homing not configured.\n";
                }
                log_telemetry(log, node, t0); // mark after home
                continue;
            }

            // --- Dwell ---
            if (cmd.contains("dwell")) {
                double sec = cmd["dwell"].get<double>();
                std::cout << "[Step " << i + 1 << "] Node " << node_idx
                          << ": dwell for " << sec << " s\n";

                auto start = steady_clock::now();
                while (duration<double>(steady_clock::now() - start).count() < sec) {
                    log_telemetry(log, node, t0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                continue;
            }

            // --- Move ---
            if (cmd.contains("pos")) {
                double pos = cmd.value("pos", 0.0);
                double vel = cmd.value("vel", 500.0);
                double acc = cmd.value("accel", 1000.0);

                std::cout << "[Step " << i + 1 << "] Node " << node_idx
                          << ": move pos=" << pos << " vel=" << vel
                          << " accel=" << acc << "\n";

                node.Motion.AccLimit = acc;
                node.Motion.VelLimit = vel;
                node.Motion.MovePosnStart(pos);

                while (!node.Motion.MoveIsDone()) {
                    log_telemetry(log, node, t0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                log_telemetry(log, node, t0); // final sample after stop
            }
        }

        // --- Cleanup ---
        for (size_t i = 0; i < node_count; ++i) {
            port.Nodes(i).EnableReq(false);
            logs[i].close();
        }

        mgr->PortsClose();
        std::cout << "Sequence complete. Telemetry saved per node.\n";

    } catch (mnErr &e) {
        std::cerr << "Teknic error: " << e.ErrorMsg << "\n";
        return -2;
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return -1;
    }
    return 0;
}
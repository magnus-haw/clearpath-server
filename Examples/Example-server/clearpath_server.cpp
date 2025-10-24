#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <atomic>        
#include <mutex>
#include "pubSysCls.h"
#include "json.hpp"
#include "httplib.h"

using namespace sFnd;
using json = nlohmann::json;
using namespace std::chrono;

#define TIME_TILL_TIMEOUT 10000
static const char* TELEMETRY_DIR = "./";
static const int SERVER_PORT = 8080;

// ---------------------------------------------------------------------
// Global system state
// ---------------------------------------------------------------------
struct SystemState {
    std::string status = "idle";
    bool stop_requested = false;
    std::mutex mtx;
    json last_step;
};
static SystemState g_state;

// ---------------------------------------------------------------------
// Persistent Motion Controller
// ---------------------------------------------------------------------
struct MotionController {
    std::mutex mtx;
    SysManager* mgr = nullptr;
    IPort* port = nullptr;
    bool connected = false;

    bool connect() {
        std::lock_guard<std::mutex> lock(mtx);
        if (connected) return true;

        std::vector<std::string> ports;
        SysManager::FindComHubPorts(ports);
        if (ports.empty()) {
            std::cerr << "No SC hubs found\n";
            return false;
        }

        mgr = SysManager::Instance();
        mgr->ComHubPort(0, ports[0].c_str());
        mgr->PortsOpen(1);
        port = &mgr->Ports(0);
        connected = true;
        std::cout << "Connected to " << port->NodeCount() << " ClearPath node(s)\n";
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected) return;
        mgr->PortsClose();
        mgr = nullptr;
        port = nullptr;
        connected = false;
        std::cout << "Disconnected ClearPath ports\n";
    }

    bool ensure_connected() {
        if (!connected) return connect();
        try {
            port->Nodes(0).Status.RT.Refresh();
            return true;
        } catch (...) {
            connected = false;
            return connect();
        }
    }
};

static MotionController g_motion;



// ---------------------------------------------------------------------
// Utility: current wall-clock string
// ---------------------------------------------------------------------
std::string now_string() {
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&t_c);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &local_tm);
    return std::string(buf);
}

// ---------------------------------------------------------------------
// Load motion sequence JSON
// ---------------------------------------------------------------------
json load_motion_sequence(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error("Cannot open sequence file: " + filename);
    json j;
    f >> j;
    if (!j.is_array())
        throw std::runtime_error("Expected JSON array in " + filename);
    return j;
}

// ---------------------------------------------------------------------
// Telemetry logging helper
// ---------------------------------------------------------------------
void log_telemetry(std::ofstream &log, INode &node,
                   steady_clock::time_point t0) {
    double t = duration<double>(steady_clock::now() - t0).count();
    node.Motion.PosnMeasured.Refresh();
    node.Motion.VelMeasured.Refresh();
    log << std::fixed << std::setprecision(4)
        << t << "," << node.Motion.PosnMeasured.Value() << ","
        << node.Motion.VelMeasured.Value() << "\n";
}

// ---------------------------------------------------------------------
// Core motion sequence runner
// ---------------------------------------------------------------------
int run_motion_sequence(const std::string &json_path) {
    try {
        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            g_state.status = "loading sequence";
            g_state.stop_requested = false;
        }

        json sequence = load_motion_sequence(json_path);
        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            g_state.status = "enabling motors";
        }
        std::cout << "Loaded " << sequence.size() << " steps from JSON.\n";

        if (!g_motion.ensure_connected())
            throw std::runtime_error("Failed to connect to ClearPath");

        SysManager* mgr = g_motion.mgr;
        IPort& port = *g_motion.port;


        size_t node_count = port.NodeCount();
        if (node_count == 0) {
            {
                std::lock_guard<std::mutex> lock(g_state.mtx);
                g_state.status = "No ClearPath nodes found";
            }
            throw std::runtime_error("No ClearPath nodes found");
        }
        std::cout << "Found " << node_count << " node(s).\n";

        // --- Enable all nodes ---
        for (size_t i = 0; i < node_count; ++i) {
            INode &n = port.Nodes(i);
            n.EnableReq(false);
            mgr->Delay(100);
            n.Status.AlertsClear();
            n.Motion.NodeStopClear();
            n.EnableReq(true);

            double timeout = mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
            while (!n.Motion.IsReady()) {
                if (mgr->TimeStampMsec() > timeout) {
                    throw std::runtime_error("Timeout enabling node " + std::to_string(i));
                }
            }
            n.AccUnit(INode::RPM_PER_SEC);
            n.VelUnit(INode::RPM);
            std::cout << "Node " << i << " enabled.\n";
        }

        // --- Telemetry setup ---
        auto t0 = steady_clock::now();
        std::string t0_stamp = now_string();

        std::vector<std::ofstream> logs(node_count);
        for (size_t i = 0; i < node_count; ++i) {
            std::string fname = std::string(TELEMETRY_DIR) +
                                "telemetry_node" + std::to_string(i) + ".csv";
            logs[i].open(fname);
            if (!logs[i].is_open())
                throw std::runtime_error("Failed to open " + fname);
            logs[i] << "# Telemetry start (system clock): " << t0_stamp << "\n";
            logs[i] << "# steady_clock t0 reference for relative times below\n";
            logs[i] << "time,pos,vel\n";
        }

        // --- Execute sequence ---
        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            g_state.status = "running";
        }

        for (size_t i = 0; i < sequence.size(); ++i) {
            bool stop_req_copy = false;
            stop_req_copy = g_state.stop_requested;
            if (stop_req_copy) {
                std::cout << "Stop requested, aborting sequence.\n";
                break;
            }

            const auto &cmd = sequence[i];
            {
                std::lock_guard<std::mutex> lock(g_state.mtx);
                g_state.last_step = cmd;
            }

            int node_idx = cmd.value("node", 0);
            if (node_idx < 0 || node_idx >= (int)node_count) {
                std::cerr << "Invalid node index at step " << i + 1 << "\n";
                continue;
            }
            INode &node = port.Nodes(node_idx);
            std::ofstream &log = logs[node_idx];

            if (cmd.contains("home") && cmd["home"].get<bool>()) {
                std::cout << "[Step " << i + 1 << "] Node " << node_idx << ": homing\n";
                log_telemetry(log, node, t0);
                if (node.Motion.Homing.HomingValid()) {
                    node.Motion.Homing.Initiate();
                    while (!node.Motion.Homing.WasHomed()) {
                        if (g_state.stop_requested) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        log_telemetry(log, node, t0);
                    }
                }
                log_telemetry(log, node, t0);
                continue;
            }

            if (cmd.contains("dwell")) {
                double sec = cmd["dwell"].get<double>();
                std::cout << "[Step " << i + 1 << "] Node " << node_idx
                          << ": dwell " << sec << " s\n";
                auto start = steady_clock::now();
                while (duration<double>(steady_clock::now() - start).count() < sec) {
                    if (g_state.stop_requested) break;
                    log_telemetry(log, node, t0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                continue;
            }

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
                    if (g_state.stop_requested) {
                        node.Motion.NodeStop(STOP_TYPE_RAMP);
                        break;
                    }
                    log_telemetry(log, node, t0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                log_telemetry(log, node, t0);
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            g_state.status = g_state.stop_requested ? "stopped" : "complete";
        }

        for (size_t i = 0; i < node_count; ++i) {
            port.Nodes(i).EnableReq(false);
            logs[i].close();
        }
        
        std::cout << "Sequence complete.\n";
    } catch (mnErr &e) {
        std::cerr << "Teknic error: " << e.ErrorMsg << "\n";
        return -2;
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return -1;
    }
    return 0;
}


// ---------------------------------------------------------------------
// Embedded HTTP server
// ---------------------------------------------------------------------
void start_http_server() {
    httplib::Server svr;

    svr.Post("/run", [](const httplib::Request &req, httplib::Response &res) {
        try {
            std::string seq_path = "motion_sequence.json";
            std::ofstream f(seq_path);
            f << req.body;
            f.close();

            std::thread([seq_path] {
                run_motion_sequence(seq_path);
            }).detach();

            res.set_content(R"({"status":"sequence received"})", "application/json");
        } catch (const std::exception &e) {
            res.status = 500;
            res.set_content(std::string("{\"error\":\"") + e.what() + "\"}",
                            "application/json");
        }
    });

    svr.Get(R"(/telemetry/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int node = std::stoi(req.matches[1]);
        std::string fname = std::string(TELEMETRY_DIR) +
                            "telemetry_node" + std::to_string(node) + ".csv";
        std::ifstream f(fname);
        if (!f.is_open()) {
            res.status = 404;
            res.set_content("{\"error\":\"not found\"}", "application/json");
            return;
        }
        std::stringstream buffer; buffer << f.rdbuf();
        res.set_content(buffer.str(), "text/csv");
    });

    // ---------------------------------------------------------------------
    // In /stop endpoint
    // ---------------------------------------------------------------------
    svr.Post("/stop", [](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(g_state.mtx);
        if (g_state.status == "running") {
            g_state.stop_requested = true;
            g_state.status = "stopping";
            res.set_content(R"({"status":"stopping"})", "application/json");
        } else {
            res.set_content(R"({"status":"not_running"})", "application/json");
        }
    });

    // ---------------------------------------------------------------------
    // In /status endpoint
    // ---------------------------------------------------------------------
    svr.Get("/status", [](const httplib::Request&, httplib::Response& res) {
        json j;
        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            j["status"] = g_state.status;
            j["last_step"] = g_state.last_step;
            j["stop_requested"] = g_state.stop_requested;
        }
        res.set_content(j.dump(), "application/json");
    });

    // ---------------------------------------------------------------------
    // /node/<id>/status endpoint
    // ---------------------------------------------------------------------
    svr.Get(R"(/node/(\d+)/status)", [](const httplib::Request &req, httplib::Response &res) {
        // Make sure the connection exists or try to reconnect
        if (!g_motion.ensure_connected() || !g_motion.port) {
            res.status = 503;
            res.set_content(R"({"error":"no active ClearPath connection"})", "application/json");
            return;
        }

        int node_id = std::stoi(req.matches[1]);

        // Lock the controller during hardware access
        std::lock_guard<std::mutex> lock(g_motion.mtx);
        size_t node_count = g_motion.port->NodeCount();

        if (node_id < 0 || node_id >= static_cast<int>(node_count)) {
            res.set_content(R"({"error":"invalid node index"})", "application/json");
            res.status = 404;
            return;
        }

        INode& node = g_motion.port->Nodes(node_id);

        // Refresh status registers safely
        node.Motion.PosnMeasured.Refresh();
        node.Motion.VelMeasured.Refresh();
        node.Status.RT.Refresh();

        json j;
        j["node"] = node_id;
        j["pos"] = node.Motion.PosnMeasured.Value();
        j["vel"] = node.Motion.VelMeasured.Value();
        j["enabled"] = node.Status.RT.Value().cpm.Enabled;
        j["ready"] = node.Motion.IsReady();

        res.set_content(j.dump(), "application/json");
    });

    std::cout << "HTTP server running on port " << SERVER_PORT << " …" << std::endl;
    svr.listen("0.0.0.0", SERVER_PORT);


}

// ---------------------------------------------------------------------
int main() {
    try {
        g_motion.connect();   // persistent initialization
        start_http_server();
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

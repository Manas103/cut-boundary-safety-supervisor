// The supervisor: the independent safety monitor (REQ-001). Genuinely a
// separate OS process from both the commander and the actuator, launched by
// the Python driver as its own PID and communicating with each only over a
// TCP loopback socket (see include/protocol.hpp for why TCP over a named
// pipe or fork+exec pipe).
//
// Concurrency: two threads share a SupervisorCore instance and a decisions
// log file, guarded by one mutex.
//   - the main thread: accepts the one commander connection, blocks on
//     LineReader::read_line for each POSE/SHUTDOWN, and forwards accepted
//     poses (or fault holds) to the actuator connection.
//   - the fault-monitor thread: polls SupervisorCore::poll_dropout on a
//     tight interval so a *complete absence* of commander messages is still
//     detected (REQ-011, REQ-016) -- there is no message for the main
//     thread to react to in that case, so dropout detection cannot live on
//     the message-handling path at all.
// This is the concurrent, inter-process design the ThreadSanitizer run
// (docs/tsan_clean_run.txt) actually exercises.
//
// Usage:
//   supervisor --actuator-port=<port> --boundary=<path> --decisions-log=<path>
//              [--stale-ms=80] [--latency-ms=15] [--dropout-ms=50] [--poll-ms=1]
// Prints "PORT <n>" (its own listening port) to stdout as the first line,
// then blocks on accept() for the one commander connection.
#include <atomic>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "boundary.hpp"
#include "clock_util.hpp"
#include "protocol.hpp"
#include "supervisor_core.hpp"

namespace {

struct Args {
    int actuator_port = -1;
    std::string boundary_path;
    std::string decisions_log_path;
    double stale_ms = 80.0;
    double latency_ms = 15.0;
    double dropout_ms = 50.0;
    double poll_ms = 1.0;
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto value_after = [&](const std::string& prefix) { return arg.substr(prefix.size()); };
        if (arg.rfind("--actuator-port=", 0) == 0) a.actuator_port = std::stoi(value_after("--actuator-port="));
        else if (arg.rfind("--boundary=", 0) == 0) a.boundary_path = value_after("--boundary=");
        else if (arg.rfind("--decisions-log=", 0) == 0) a.decisions_log_path = value_after("--decisions-log=");
        else if (arg.rfind("--stale-ms=", 0) == 0) a.stale_ms = std::stod(value_after("--stale-ms="));
        else if (arg.rfind("--latency-ms=", 0) == 0) a.latency_ms = std::stod(value_after("--latency-ms="));
        else if (arg.rfind("--dropout-ms=", 0) == 0) a.dropout_ms = std::stod(value_after("--dropout-ms="));
        else if (arg.rfind("--poll-ms=", 0) == 0) a.poll_ms = std::stod(value_after("--poll-ms="));
    }
    if (a.actuator_port < 0 || a.boundary_path.empty() || a.decisions_log_path.empty()) {
        std::cerr << "usage: supervisor --actuator-port=N --boundary=PATH --decisions-log=PATH "
                     "[--stale-ms=80] [--latency-ms=15] [--dropout-ms=50] [--poll-ms=1]\n";
        std::exit(2);
    }
    return a;
}

std::mutex g_mutex;
std::ofstream g_log;

void log_decision(const std::string& tag, long seq, const cbs::Decision& d) {
    double hold_ms = -1.0;
    if (d.kind == cbs::DecisionKind::kVetoStale || d.kind == cbs::DecisionKind::kVetoLatency ||
        d.kind == cbs::DecisionKind::kVetoDropout) {
        hold_ms = d.hold_issued_wall_ms - d.fault_onset_wall_ms;
    }
    g_log << "DECISION tag=" << tag << " seq=" << seq << " req=" << (d.requirement_id.empty() ? "none" : d.requirement_id)
          << " detail=" << (d.detail.empty() ? "none" : d.detail) << " age_ms=" << d.age_ms
          << " fault_onset_wall_ms=" << d.fault_onset_wall_ms << " hold_issued_wall_ms=" << d.hold_issued_wall_ms
          << " hold_ms=" << hold_ms << " t_wall_ms=" << cbs::now_steady_ms() << "\n";
    g_log.flush();
}

}  // namespace

int main(int argc, char** argv) {
    using namespace cbs;
    Args args = parse_args(argc, argv);

    Boundary boundary = Boundary::load_from_file(args.boundary_path);  // REQ-002, REQ-008
    SupervisorCore core(boundary, args.stale_ms, args.latency_ms, args.dropout_ms);

    g_log.open(args.decisions_log_path, std::ios::trunc);

    // Connect out to the actuator first: it must already be listening
    // (the driver starts it first and hands us its port).
    int actuator_fd = connect_loopback(args.actuator_port);

    int listen_fd = make_listen_socket_port0();  // REQ-020
    int my_port = get_assigned_port(listen_fd);
    std::cout << "PORT " << my_port << std::endl;
    std::cout.flush();

    std::atomic<bool> shutdown_flag{false};
    std::thread fault_thread([&]() {
        while (!shutdown_flag.load()) {
            double now = now_steady_ms();
            Decision d;
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                d = core.poll_dropout(now);
            }
            if (d.kind == DecisionKind::kVetoDropout) {
                std::lock_guard<std::mutex> lock(g_mutex);
                write_line(actuator_fd, format_hold_line(-1, "dropout"));
                log_decision("dropout", -1, d);
            }
            sleep_ms(args.poll_ms);
        }
    });

    int commander_fd = accept_one(listen_fd);
    LineReader reader(commander_fd);

    std::string line;
    bool got_shutdown = false;
    while (reader.read_line(line)) {
        CommanderMsg msg = parse_commander_line(line);  // REQ-019: never throws
        if (msg.kind == CommanderMsgKind::kShutdown) {
            got_shutdown = true;
            break;
        }
        if (msg.kind == CommanderMsgKind::kMalformed) {
            Decision d;
            d.kind = DecisionKind::kNone;
            d.requirement_id = "REQ-019";
            d.detail = "malformed_line=" + msg.raw_error;
            std::lock_guard<std::mutex> lock(g_mutex);
            log_decision("malformed", -1, d);
            continue;  // REQ-019: log and keep running, do not crash
        }
        // kPose
        double receipt = now_steady_ms();
        Decision d;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            d = core.decide_pose(msg.pose, receipt);
            if (d.kind == DecisionKind::kAccept) {
                write_line(actuator_fd, format_forward_line(msg.pose.seq, msg.pose.x, msg.pose.y, msg.pose.z));
            } else if (d.kind == DecisionKind::kVetoStale) {
                write_line(actuator_fd, format_hold_line(msg.pose.seq, "stale"));
            } else if (d.kind == DecisionKind::kVetoLatency) {
                write_line(actuator_fd, format_hold_line(msg.pose.seq, "latency"));
            }
            // kVetoBoundary: withheld, nothing forwarded, no hold sent -- REQ-003/REQ-005.
            log_decision(d.kind == DecisionKind::kAccept ? "accept" : "veto", msg.pose.seq, d);
        }
    }

    // REQ-022: clean shutdown -- forward SHUTDOWN downstream (whether it
    // arrived explicitly or the commander connection simply closed), stop
    // the fault-monitor thread, close both sockets, then exit.
    (void)got_shutdown;
    write_line(actuator_fd, format_shutdown_line());
    shutdown_flag.store(true);
    fault_thread.join();
    close(commander_fd);
    close(actuator_fd);
    close(listen_fd);
    return 0;
}

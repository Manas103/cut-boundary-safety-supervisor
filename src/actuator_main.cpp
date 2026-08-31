// The simulated actuator: the ground truth for "did an unsafe command ever
// actually reach the arm" (REQ-005, REQ-006). It only ever sees whatever
// the supervisor decides to forward or hold -- it has no boundary logic and
// no fault logic of its own, on purpose, so it cannot accidentally correct
// for a supervisor bug; it is a passive recorder.
//
// Usage:
//   actuator --log=<path>
// Prints "PORT <n>" (its own listening port) to stdout as the first line,
// then blocks on accept() for the one supervisor connection.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "clock_util.hpp"
#include "protocol.hpp"

namespace {

struct Args {
    std::string log_path;
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--log=", 0) == 0) a.log_path = arg.substr(std::string("--log=").size());
    }
    if (a.log_path.empty()) {
        std::cerr << "usage: actuator --log=PATH\n";
        std::exit(2);
    }
    return a;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace cbs;
    Args args = parse_args(argc, argv);

    std::ofstream log(args.log_path, std::ios::trunc);

    int listen_fd = make_listen_socket_port0();  // REQ-020
    int port = get_assigned_port(listen_fd);
    std::cout << "PORT " << port << std::endl;
    std::cout.flush();

    int supervisor_fd = accept_one(listen_fd);
    LineReader reader(supervisor_fd);

    std::string line;
    while (reader.read_line(line)) {
        ActuatorMsg msg = parse_actuator_line(line);
        double t = now_steady_ms();
        if (msg.kind == ActuatorMsgKind::kForward) {
            log << "RECV FORWARD seq=" << msg.seq << " x=" << msg.x << " y=" << msg.y << " z=" << msg.z
                << " t_wall_ms=" << t << "\n";
        } else if (msg.kind == ActuatorMsgKind::kHold) {
            // REQ-015: a distinct message type from FORWARD, logged distinctly.
            log << "RECV HOLD seq=" << msg.seq << " reason=" << msg.reason << " t_wall_ms=" << t << "\n";
        } else if (msg.kind == ActuatorMsgKind::kShutdown) {
            log << "RECV SHUTDOWN t_wall_ms=" << t << "\n";
            log.flush();
            break;
        } else {
            log << "RECV MALFORMED raw=" << line << " t_wall_ms=" << t << "\n";
        }
        log.flush();
    }

    close(supervisor_fd);
    close(listen_fd);
    return 0;
}

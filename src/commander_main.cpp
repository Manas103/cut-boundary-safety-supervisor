// The commander ("planner process"): streams a scenario file of commanded
// tool poses to the supervisor over a TCP loopback connection, one line per
// scenario instruction. This process has no boundary logic and no fault
// logic at all -- it only plays back a script, deliberately, so nothing in
// the safety decision path depends on what the commander believes about the
// boundary (that is the whole point of REQ-002).
//
// Usage:
//   commander --supervisor-port=<port> --scenario=<path> [--log=<path>]
//
// Scenario file grammar (one instruction per line, '#' comments allowed):
//   POSE <x> <y> <z>                   send a pose captured "now"
//   POSE <x> <y> <z> STALE_MS=<n>      send a pose whose capture_time_ms is
//                                       backdated by n ms (REQ-012 case)
//   POSE <x> <y> <z> DELAY_MS=<n>      capture "now", then actually sleep n
//                                       ms before sending (REQ-013 case)
//   DROPOUT_MS <n>                     send nothing for n ms (REQ-011 case)
//   RAW <text>                         send <text> verbatim, unframed
//                                       protocol-wise (REQ-019 case)
//   SHUTDOWN                           send SHUTDOWN and exit
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "clock_util.hpp"
#include "protocol.hpp"

namespace {

struct Args {
    int supervisor_port = -1;
    std::string scenario_path;
    std::string log_path;
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto value_after = [&](const std::string& prefix) -> std::string {
            return arg.substr(prefix.size());
        };
        if (arg.rfind("--supervisor-port=", 0) == 0) {
            a.supervisor_port = std::stoi(value_after("--supervisor-port="));
        } else if (arg.rfind("--scenario=", 0) == 0) {
            a.scenario_path = value_after("--scenario=");
        } else if (arg.rfind("--log=", 0) == 0) {
            a.log_path = value_after("--log=");
        }
    }
    if (a.supervisor_port < 0 || a.scenario_path.empty()) {
        std::cerr << "usage: commander --supervisor-port=N --scenario=PATH [--log=PATH]\n";
        std::exit(2);
    }
    return a;
}

std::string kv(const std::string& token, const std::string& key) {
    auto pos = token.find('=');
    if (pos == std::string::npos) return "";
    if (token.substr(0, pos) != key) return "";
    return token.substr(pos + 1);
}

}  // namespace

int main(int argc, char** argv) {
    using namespace cbs;
    Args args = parse_args(argc, argv);

    std::ifstream scenario(args.scenario_path);
    if (!scenario) {
        std::cerr << "commander: cannot open scenario file: " << args.scenario_path << "\n";
        return 2;
    }

    std::ofstream log;
    if (!args.log_path.empty()) log.open(args.log_path, std::ios::trunc);

    int fd = connect_loopback(args.supervisor_port);

    long seq = 0;
    std::string line;
    while (std::getline(scenario, line)) {
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag.empty() || tag[0] == '#') continue;

        if (tag == "POSE") {
            double x, y, z;
            iss >> x >> y >> z;
            std::string opt;
            double stale_ms = 0.0, delay_ms = 0.0;
            while (iss >> opt) {
                std::string v;
                if (!(v = kv(opt, "STALE_MS")).empty()) stale_ms = std::stod(v);
                if (!(v = kv(opt, "DELAY_MS")).empty()) delay_ms = std::stod(v);
            }
            Pose p;
            p.seq = ++seq;
            p.capture_time_ms = now_steady_ms() - stale_ms;
            p.x = x;
            p.y = y;
            p.z = z;
            if (delay_ms > 0.0) sleep_ms(delay_ms);
            std::string wire = format_pose_line(p);
            write_line(fd, wire);
            if (log) log << "SENT " << wire << " t_wall_ms=" << now_steady_ms() << "\n";
        } else if (tag == "DROPOUT_MS") {
            double n;
            iss >> n;
            if (log) log << "DROPOUT_SLEEP " << n << " t_wall_ms=" << now_steady_ms() << "\n";
            sleep_ms(n);
        } else if (tag == "RAW") {
            std::string rest;
            std::getline(iss, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            write_line(fd, rest);
            if (log) log << "SENT_RAW " << rest << " t_wall_ms=" << now_steady_ms() << "\n";
        } else if (tag == "SHUTDOWN") {
            write_line(fd, format_shutdown_line());
            if (log) log << "SENT SHUTDOWN t_wall_ms=" << now_steady_ms() << "\n";
            break;
        }
    }

    close(fd);
    return 0;
}

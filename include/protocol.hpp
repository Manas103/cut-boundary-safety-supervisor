// IPC wire protocol and the POSIX TCP-loopback plumbing under it.
//
// Why TCP loopback on an OS-assigned port, not a named pipe or POSIX pipe+
// fork/exec (REQ-020). All three processes here are launched by a Python
// driver script that must hold each child's PID for the whole run and never
// discover it by scraping tasklist/netstat later (see the pipeline's
// process-safety rules). A loopback TCP socket bound to port 0 gets an
// OS-assigned ephemeral port that the driver reads back from the child's own
// stdout and hands to the next process as a plain command-line argument --
// no coordination file, no risk of colliding with a port some other
// process already owns, and the same mechanism trivially supports the
// commander and supervisor being genuinely different processes even if a
// future variant of this project ran them on different hosts. A POSIX pipe
// from fork()+exec() would work for the parent-child pair but conflates
// "process separation" with "still forked from a common parent that shares
// its address space's memory allocator and any global state up to the
// fork point"; three independently exec'd processes talking only over a
// socket is the stronger, more literal reading of "genuinely separate".
//
// Why a newline-delimited text protocol (REQ-018), not a length-prefixed
// binary one. Every message here is a handful of ASCII tokens (an id, a
// handful of doubles); a text protocol is trivially inspectable with the
// same eyes reading the log files next to it, and the actual hazard a real
// framed-protocol requirement exists to defend against, a message split
// across two recv() calls, or two messages coalesced into one recv() call,
// applies exactly the same to newline-delimited text as to length-prefixed
// binary. LineReader below buffers across arbitrary recv() boundaries and
// only ever hands a caller one complete, newline-terminated line.
#pragma once

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include "pose.hpp"

namespace cbs {

// ---------------------------------------------------------------------
// Socket setup (REQ-020: always port 0, OS-assigned).
// ---------------------------------------------------------------------

inline int make_listen_socket_port0() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(0);  // OS-assigned ephemeral port, never hardcoded
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        throw std::runtime_error(std::string("bind() failed: ") + std::strerror(errno));
    }
    if (listen(fd, 8) != 0) {
        throw std::runtime_error(std::string("listen() failed: ") + std::strerror(errno));
    }
    return fd;
}

inline int get_assigned_port(int listen_fd) {
    sockaddr_in addr{};
    socklen_t len = sizeof(addr);
    if (getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        throw std::runtime_error(std::string("getsockname() failed: ") + std::strerror(errno));
    }
    return ntohs(addr.sin_port);
}

inline int accept_one(int listen_fd) {
    int fd = accept(listen_fd, nullptr, nullptr);
    if (fd < 0) throw std::runtime_error(std::string("accept() failed: ") + std::strerror(errno));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return fd;
}

inline int connect_loopback(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        throw std::runtime_error(std::string("connect() failed: ") + std::strerror(errno));
    }
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return fd;
}

inline void write_all(int fd, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = send(fd, data.data() + sent, data.size() - sent, 0);
        if (n <= 0) throw std::runtime_error(std::string("send() failed: ") + std::strerror(errno));
        sent += static_cast<size_t>(n);
    }
}

inline void write_line(int fd, const std::string& line) { write_all(fd, line + "\n"); }

// ---------------------------------------------------------------------
// REQ-018: buffered line reader. recv() makes no promise about aligning
// with message boundaries; this reassembles exactly one newline-terminated
// line per call regardless of how the underlying reads were chunked, and is
// exercised directly (no socket needed, a socketpair() feeds it split
// writes) in tests/protocol_test.cpp.
// ---------------------------------------------------------------------
class LineReader {
public:
    explicit LineReader(int fd) : fd_(fd) {}

    // Returns false on a clean EOF with no partial line pending; throws on a
    // real socket error. On success, out holds one line with the trailing
    // '\n' stripped.
    bool read_line(std::string& out) {
        while (true) {
            size_t nl = buffer_.find('\n');
            if (nl != std::string::npos) {
                out = buffer_.substr(0, nl);
                buffer_.erase(0, nl + 1);
                return true;
            }
            char chunk[4096];
            ssize_t n = recv(fd_, chunk, sizeof(chunk), 0);
            if (n < 0) throw std::runtime_error(std::string("recv() failed: ") + std::strerror(errno));
            if (n == 0) {
                if (!buffer_.empty()) {
                    out = buffer_;
                    buffer_.clear();
                    return true;
                }
                return false;
            }
            buffer_.append(chunk, static_cast<size_t>(n));
        }
    }

private:
    int fd_;
    std::string buffer_;
};

// ---------------------------------------------------------------------
// Commander -> supervisor message parsing/formatting.
// ---------------------------------------------------------------------

enum class CommanderMsgKind { kPose, kShutdown, kMalformed };

struct CommanderMsg {
    CommanderMsgKind kind = CommanderMsgKind::kMalformed;
    Pose pose;
    std::string raw_error;
};

// REQ-019: never throws on bad input; malformed input becomes kMalformed
// with the offending line preserved for logging.
inline CommanderMsg parse_commander_line(const std::string& line) {
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;
    if (tag == "SHUTDOWN") {
        return CommanderMsg{CommanderMsgKind::kShutdown, {}, ""};
    }
    if (tag == "POSE") {
        Pose p;
        if (iss >> p.seq >> p.capture_time_ms >> p.x >> p.y >> p.z) {
            std::string trailing;
            if (!(iss >> trailing)) {
                return CommanderMsg{CommanderMsgKind::kPose, p, ""};
            }
        }
        return CommanderMsg{CommanderMsgKind::kMalformed, {}, line};
    }
    return CommanderMsg{CommanderMsgKind::kMalformed, {}, line};
}

inline std::string format_pose_line(const Pose& p) {
    std::ostringstream oss;
    oss << "POSE " << p.seq << " " << p.capture_time_ms << " " << p.x << " " << p.y << " " << p.z;
    return oss.str();
}

inline std::string format_shutdown_line() { return "SHUTDOWN"; }

// ---------------------------------------------------------------------
// Supervisor -> actuator message parsing/formatting (REQ-015: FORWARD and
// HOLD are distinct message types).
// ---------------------------------------------------------------------

enum class ActuatorMsgKind { kForward, kHold, kShutdown, kMalformed };

struct ActuatorMsg {
    ActuatorMsgKind kind = ActuatorMsgKind::kMalformed;
    long seq = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    std::string reason;
};

inline ActuatorMsg parse_actuator_line(const std::string& line) {
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;
    if (tag == "SHUTDOWN") return ActuatorMsg{ActuatorMsgKind::kShutdown, 0, 0, 0, 0, ""};
    if (tag == "FORWARD") {
        ActuatorMsg m;
        m.kind = ActuatorMsgKind::kForward;
        if (iss >> m.seq >> m.x >> m.y >> m.z) return m;
        return ActuatorMsg{ActuatorMsgKind::kMalformed, 0, 0, 0, 0, line};
    }
    if (tag == "HOLD") {
        ActuatorMsg m;
        m.kind = ActuatorMsgKind::kHold;
        if (iss >> m.seq >> m.reason) return m;
        return ActuatorMsg{ActuatorMsgKind::kMalformed, 0, 0, 0, 0, line};
    }
    return ActuatorMsg{ActuatorMsgKind::kMalformed, 0, 0, 0, 0, line};
}

inline std::string format_forward_line(long seq, double x, double y, double z) {
    std::ostringstream oss;
    oss << "FORWARD " << seq << " " << x << " " << y << " " << z;
    return oss.str();
}

inline std::string format_hold_line(long seq, const std::string& reason) {
    return "HOLD " + std::to_string(seq) + " " + reason;
}

}  // namespace cbs

// Protocol framing and parsing: REQ-015, REQ-018, REQ-019, REQ-020.
#include <gtest/gtest.h>

#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include <thread>

#include "protocol.hpp"
#include "req_trace.hpp"

using namespace cbs;

namespace {

// A connected pair of stream sockets, close enough in kernel behavior to a
// TCP loopback connection (same recv()/send() semantics, same "no message
// boundary guarantee") to exercise LineReader without opening a real port.
std::pair<int, int> make_socketpair() {
    int fds[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
        throw std::runtime_error("socketpair() failed");
    }
    return {fds[0], fds[1]};
}

}  // namespace

TEST(Protocol, LineReaderReassemblesMessageSplitAcrossMultipleSendCalls) {
    TRACE_REQ("REQ-018");
    auto [a, b] = make_socketpair();
    LineReader reader(b);

    std::string msg = "POSE 1 0.0 10 20 30";
    // Split the write into three pieces, on purpose crossing the line
    // boundary mid-token, to prove the reader does not act on a partial line.
    write_all(a, msg.substr(0, 5));
    write_all(a, msg.substr(5, 6));
    write_all(a, msg.substr(11) + "\n");

    std::string line;
    ASSERT_TRUE(reader.read_line(line));
    EXPECT_EQ(line, msg);

    close(a);
    close(b);
}

TEST(Protocol, LineReaderSplitsTwoMessagesSentInOneWrite) {
    TRACE_REQ("REQ-018");
    auto [a, b] = make_socketpair();
    LineReader reader(b);

    write_all(a, "POSE 1 0.0 1 2 3\nPOSE 2 0.0 4 5 6\n");

    std::string line1, line2;
    ASSERT_TRUE(reader.read_line(line1));
    ASSERT_TRUE(reader.read_line(line2));
    EXPECT_EQ(line1, "POSE 1 0.0 1 2 3");
    EXPECT_EQ(line2, "POSE 2 0.0 4 5 6");

    close(a);
    close(b);
}

TEST(Protocol, LineReaderReturnsFalseOnCleanEofWithNoPendingData) {
    TRACE_REQ("REQ-018");
    auto [a, b] = make_socketpair();
    close(a);  // EOF from b's perspective
    LineReader reader(b);
    std::string line;
    EXPECT_FALSE(reader.read_line(line));
    close(b);
}

TEST(Protocol, ParseCommanderLineParsesValidPoseAndShutdown) {
    TRACE_REQ("REQ-018");
    CommanderMsg m = parse_commander_line("POSE 5 123.5 10 20 30");
    ASSERT_EQ(m.kind, CommanderMsgKind::kPose);
    EXPECT_EQ(m.pose.seq, 5);
    EXPECT_DOUBLE_EQ(m.pose.capture_time_ms, 123.5);
    EXPECT_DOUBLE_EQ(m.pose.x, 10);

    CommanderMsg s = parse_commander_line("SHUTDOWN");
    EXPECT_EQ(s.kind, CommanderMsgKind::kShutdown);
}

TEST(Protocol, ParseCommanderLineRejectsMalformedInputWithoutThrowing) {
    TRACE_REQ("REQ-019");
    CommanderMsg m1 = parse_commander_line("POSE 5 123.5 10 20");  // missing z
    EXPECT_EQ(m1.kind, CommanderMsgKind::kMalformed);

    CommanderMsg m2 = parse_commander_line("GARBAGE this is not a message");
    EXPECT_EQ(m2.kind, CommanderMsgKind::kMalformed);

    CommanderMsg m3 = parse_commander_line("");
    EXPECT_EQ(m3.kind, CommanderMsgKind::kMalformed);

    CommanderMsg m4 = parse_commander_line("POSE 5 123.5 10 20 30 extra_token");
    EXPECT_EQ(m4.kind, CommanderMsgKind::kMalformed);
}

TEST(Protocol, ParseActuatorLineDistinguishesForwardFromHold) {
    TRACE_REQ("REQ-015");
    ActuatorMsg f = parse_actuator_line("FORWARD 3 10 20 30");
    ASSERT_EQ(f.kind, ActuatorMsgKind::kForward);
    EXPECT_EQ(f.seq, 3);

    ActuatorMsg h = parse_actuator_line("HOLD 3 dropout");
    ASSERT_EQ(h.kind, ActuatorMsgKind::kHold);
    EXPECT_EQ(h.reason, "dropout");
    EXPECT_NE(f.kind, h.kind);
}

TEST(Protocol, ListenSocketBindsToNonZeroOsAssignedPort) {
    TRACE_REQ("REQ-020");
    int fd = make_listen_socket_port0();
    int port = get_assigned_port(fd);
    EXPECT_GT(port, 0);
    close(fd);
}

TEST(Protocol, TwoListenSocketsGetDifferentOsAssignedPorts) {
    TRACE_REQ("REQ-020");
    int fd1 = make_listen_socket_port0();
    int fd2 = make_listen_socket_port0();
    int p1 = get_assigned_port(fd1);
    int p2 = get_assigned_port(fd2);
    EXPECT_NE(p1, p2);
    close(fd1);
    close(fd2);
}

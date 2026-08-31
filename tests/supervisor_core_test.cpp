// SupervisorCore: REQ-002, REQ-004, REQ-011, REQ-012, REQ-013, REQ-014,
// REQ-016. All synthetic wall-clock timestamps -- no socket, no thread, no
// real process, which is exactly the point of factoring this class out of
// supervisor_main.cpp (see supervisor_core.hpp's header comment).
//
// REQ-017 (recovery after a fault clears) is deliberately NOT tagged here or
// anywhere else in this test suite -- see README "Requirements traceability"
// for why that is a disclosed gap, not an oversight.
#include <gtest/gtest.h>

#include "boundary.hpp"
#include "req_trace.hpp"
#include "supervisor_core.hpp"

using namespace cbs;

namespace {

Boundary make_box() {
    Boundary b;
    b.add_half_space({"x_min", -1, 0, 0, 0});
    b.add_half_space({"x_max", 1, 0, 0, 100});
    b.add_half_space({"y_min", 0, -1, 0, 0});
    b.add_half_space({"y_max", 0, 1, 0, 60});
    b.add_half_space({"z_min", 0, 0, -1, 0});
    b.add_half_space({"z_max", 0, 0, 1, 40});
    return b;
}

// thresholds: stale=80ms, latency=15ms, dropout=50ms -- match the shipped
// supervisor_main.cpp defaults, so these unit tests exercise the same
// numbers the end-to-end scenarios do.
SupervisorCore make_core() { return SupervisorCore(make_box(), 80.0, 15.0, 50.0); }

}  // namespace

TEST(SupervisorCore, AcceptsSafePoseWithinLatencyBudget) {
    TRACE_REQ("REQ-002");
    auto core = make_core();
    Pose p{1, 1000.0, 50, 30, 20};
    Decision d = core.decide_pose(p, /*receipt_wall_ms=*/1005.0);  // age 5ms, under latency threshold
    EXPECT_EQ(d.kind, DecisionKind::kAccept);
    EXPECT_TRUE(d.requirement_id.empty());
}

TEST(SupervisorCore, VetoesBoundaryViolationAndNamesReq003AndFace) {
    TRACE_REQ("REQ-002");
    TRACE_REQ("REQ-004");
    auto core = make_core();
    Pose p{2, 1000.0, 500, 30, 20};  // far past x_max, age 0
    Decision d = core.decide_pose(p, 1000.0);
    EXPECT_EQ(d.kind, DecisionKind::kVetoBoundary);
    EXPECT_EQ(d.requirement_id, "REQ-003");
    EXPECT_NE(d.detail.find("x_max"), std::string::npos);
}

TEST(SupervisorCore, ClassifiesStalePoseAboveStaleThreshold) {
    TRACE_REQ("REQ-012");
    auto core = make_core();
    Pose p{3, 1000.0, 50, 30, 20};
    Decision d = core.decide_pose(p, /*receipt_wall_ms=*/1120.0);  // age 120ms > 80ms
    EXPECT_EQ(d.kind, DecisionKind::kVetoStale);
    EXPECT_EQ(d.requirement_id, "REQ-012");
    EXPECT_NEAR(d.hold_issued_wall_ms - d.fault_onset_wall_ms, 0.0, 1e-9);
}

TEST(SupervisorCore, ClassifiesLatencySpikeBetweenLatencyAndStaleThresholds) {
    TRACE_REQ("REQ-013");
    auto core = make_core();
    Pose p{4, 1000.0, 50, 30, 20};
    Decision d = core.decide_pose(p, /*receipt_wall_ms=*/1040.0);  // age 40ms: >15, <=80
    EXPECT_EQ(d.kind, DecisionKind::kVetoLatency);
    EXPECT_EQ(d.requirement_id, "REQ-013");
}

TEST(SupervisorCore, ThresholdBoundariesAreExclusiveNotInclusive) {
    TRACE_REQ("REQ-012");
    TRACE_REQ("REQ-013");
    auto core = make_core();
    // age exactly == latency threshold (15ms): still normal, proceeds to boundary check.
    Decision d1 = core.decide_pose(Pose{5, 1000.0, 50, 30, 20}, 1015.0);
    EXPECT_EQ(d1.kind, DecisionKind::kAccept);
    // age exactly == stale threshold (80ms): still classified as a latency spike, not stale.
    Decision d2 = core.decide_pose(Pose{6, 1000.0, 50, 30, 20}, 1080.0);
    EXPECT_EQ(d2.kind, DecisionKind::kVetoLatency);
}

TEST(SupervisorCore, DropoutDetectedOncePollCrossesWindowAndNotBeforeReq011) {
    TRACE_REQ("REQ-011");
    TRACE_REQ("REQ-016");
    auto core = make_core();
    core.decide_pose(Pose{7, 1000.0, 50, 30, 20}, 1000.0);  // baseline pose, no fault handling involved after this

    // Polling before the dropout window elapses: no fault yet, regardless of
    // whether any pose message has been handled in between (there is none
    // here at all -- this exercises poll_dropout in isolation, REQ-016).
    EXPECT_EQ(core.poll_dropout(1030.0).kind, DecisionKind::kNone);
    EXPECT_EQ(core.poll_dropout(1049.9).kind, DecisionKind::kNone);

    // Crossing the 50ms window: fault detected, hold issued.
    Decision d = core.poll_dropout(1051.0);
    EXPECT_EQ(d.kind, DecisionKind::kVetoDropout);
    EXPECT_EQ(d.requirement_id, "REQ-011");
    EXPECT_TRUE(core.is_held());

    // Repeated polling while still silent does not re-trigger.
    EXPECT_EQ(core.poll_dropout(1060.0).kind, DecisionKind::kNone);
}

TEST(SupervisorCore, DropoutHoldTimingIsMeasuredFromWindowCrossingNotFromNow) {
    TRACE_REQ("REQ-014");
    auto core = make_core();
    core.decide_pose(Pose{8, 1000.0, 50, 30, 20}, 1000.0);
    // Poll fires 3ms after the 50ms window would have crossed (simulating a
    // coarse poll interval) -- fault_onset is the window-crossing instant
    // (1050.0), not the poll instant (1053.0), so hold_ms reports the
    // detection latency actually incurred, 3ms, comfortably under 20ms.
    Decision d = core.poll_dropout(1053.0);
    ASSERT_EQ(d.kind, DecisionKind::kVetoDropout);
    EXPECT_NEAR(d.fault_onset_wall_ms, 1050.0, 1e-9);
    EXPECT_NEAR(d.hold_issued_wall_ms, 1053.0, 1e-9);
    EXPECT_LT(d.hold_issued_wall_ms - d.fault_onset_wall_ms, 20.0);
}

TEST(SupervisorCore, NoBaselinePoseMeansDropoutNeverFires) {
    TRACE_REQ("REQ-011");
    auto core = make_core();
    EXPECT_FALSE(core.has_received_any());
    EXPECT_EQ(core.poll_dropout(999999.0).kind, DecisionKind::kNone);
}

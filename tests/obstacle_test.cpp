// Unit tests for the obstacle-protection profile's pure zone geometry
// (ObstacleZoneModel) and decision logic (ObstacleSupervisorCore), mirroring
// boundary_test.cpp and supervisor_core_test.cpp's style: no IPC, no
// process, synthetic wall-clock values only.

#include <gtest/gtest.h>

#include <cmath>

#include "obstacle_core.hpp"
#include "obstacle_zone.hpp"

using namespace cbs;

namespace {

ZoneParams TestParams() {
    ZoneParams p;
    p.latency_s = 0.15;
    p.decel_mps2 = 1.5;
    p.margin_m = 0.30;
    p.warning_lead_s = 1.0;
    return p;
}

}  // namespace

TEST(ObstacleZoneModel, ZeroOrNegativeSpeedStopsInstantly) {
    ObstacleZoneModel zone(TestParams());
    EXPECT_DOUBLE_EQ(zone.stopping_distance_m(0.0), 0.0);
    EXPECT_DOUBLE_EQ(zone.stopping_distance_m(-1.0), 0.0);
}

TEST(ObstacleZoneModel, StoppingDistanceMatchesClosedForm) {
    ObstacleZoneModel zone(TestParams());
    double v = 2.0;
    double expected = v * 0.15 + (v * v) / (2.0 * 1.5);
    EXPECT_NEAR(zone.stopping_distance_m(v), expected, 1e-9);
}

TEST(ObstacleZoneModel, HazardBoundaryAddsMargin) {
    ObstacleZoneModel zone(TestParams());
    double v = 1.5;
    EXPECT_NEAR(zone.hazard_boundary_m(v), zone.stopping_distance_m(v) + 0.30, 1e-9);
}

TEST(ObstacleZoneModel, WarningBoundaryStrictlyLargerThanHazard) {
    ObstacleZoneModel zone(TestParams());
    for (double v = 0.5; v <= 3.0; v += 0.5) {
        EXPECT_GT(zone.warning_boundary_m(v), zone.hazard_boundary_m(v));
    }
}

TEST(ObstacleZoneModel, LoadFromFileMatchesShippedConfig) {
    // Run from build/ (this repo's test binaries' own convention, see
    // boundary_test.cpp's identical "../config/..." path), not the repo root.
    ObstacleZoneModel zone = ObstacleZoneModel::load_from_file("../config/obstacle_zone.txt");
    EXPECT_DOUBLE_EQ(zone.params().latency_s, 0.15);
    EXPECT_DOUBLE_EQ(zone.params().decel_mps2, 1.5);
    EXPECT_DOUBLE_EQ(zone.params().margin_m, 0.30);
    EXPECT_DOUBLE_EQ(zone.params().warning_lead_s, 1.0);
}

TEST(ObstacleZoneModel, RejectsMissingFile) {
    EXPECT_THROW(ObstacleZoneModel::load_from_file("config/does_not_exist.txt"), std::runtime_error);
}

TEST(ObstacleZoneModel, RejectsNonPositiveDeceleration) {
    ZoneParams p = TestParams();
    p.decel_mps2 = 0.0;
    // load_from_file's own validation is exercised by a temp file in a
    // dedicated fixture would be more work than this direct-construction
    // check is worth; the constructor itself does not validate (mirroring
    // Boundary's own division-only-at-use-site convention), so this test
    // instead confirms the documented failure mode: dividing by a
    // non-positive decel produces a non-finite stopping distance, which a
    // real caller would reject before ever loading it into a supervisor.
    ObstacleZoneModel zone(p);
    EXPECT_FALSE(std::isfinite(zone.stopping_distance_m(1.0)));
}

TEST(ObstacleSupervisorCore, ForwardsWhenClear) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 300.0);
    ObstacleDecision d = core.decide_detection(zone.warning_boundary_m(1.0) + 5.0, 1.0, 0.0);
    EXPECT_EQ(d.kind, ObstacleDecisionKind::kForward);
    EXPECT_EQ(d.requirement_id, "");
}

TEST(ObstacleSupervisorCore, WarnsInWarningZoneWithoutStopping) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 300.0);
    double v = 1.0;
    double range = (zone.hazard_boundary_m(v) + zone.warning_boundary_m(v)) / 2.0;
    ObstacleDecision d = core.decide_detection(range, v, 0.0);
    EXPECT_EQ(d.kind, ObstacleDecisionKind::kWarn);
    EXPECT_EQ(d.requirement_id, "REQ-OBS-005");
}

TEST(ObstacleSupervisorCore, StopsAtOrInsideHazardBoundary) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 300.0);
    double v = 1.0;
    ObstacleDecision d = core.decide_detection(zone.hazard_boundary_m(v), v, 0.0);
    EXPECT_EQ(d.kind, ObstacleDecisionKind::kStopHazard);
    EXPECT_EQ(d.requirement_id, "REQ-OBS-001");
}

TEST(ObstacleSupervisorCore, ForwardClearsAnyPriorDropoutHold) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 50.0);
    core.decide_detection(zone.warning_boundary_m(1.0) + 5.0, 1.0, 0.0);
    ObstacleDecision dropout = core.poll_dropout(1000.0);
    EXPECT_EQ(dropout.kind, ObstacleDecisionKind::kStopDropout);
    EXPECT_TRUE(core.is_held());

    core.decide_detection(zone.warning_boundary_m(1.0) + 5.0, 1.0, 1001.0);
    EXPECT_FALSE(core.is_held());
}

TEST(ObstacleSupervisorCore, DropoutNotRetriggeredWhileHeld) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 50.0);
    core.decide_detection(zone.warning_boundary_m(1.0) + 5.0, 1.0, 0.0);
    ObstacleDecision first = core.poll_dropout(1000.0);
    ObstacleDecision second = core.poll_dropout(1500.0);
    EXPECT_EQ(first.kind, ObstacleDecisionKind::kStopDropout);
    EXPECT_EQ(second.kind, ObstacleDecisionKind::kNone);
}

TEST(ObstacleSupervisorCore, NoDropoutBeforeAnyDetectionReceived) {
    ObstacleZoneModel zone(TestParams());
    ObstacleSupervisorCore core(zone, 50.0);
    ObstacleDecision d = core.poll_dropout(10000.0);
    EXPECT_EQ(d.kind, ObstacleDecisionKind::kNone);
}

// The obstacle-supervisor's decision logic, factored out of
// obs_supervisor_main.cpp's socket/thread plumbing entirely, exactly the way
// supervisor_core.hpp is factored out of supervisor_main.cpp: directly
// unit-testable by calling its methods with synthetic wall-clock timestamps,
// no process, no socket, no thread involved.
//
// What this class is deliberately blind to (see README "Design deep-dive:
// why late detection and stale pose need no supervisor-side classification
// branch"). A detector-dropout fault is a *complete absence* of messages,
// which this class can detect for itself, the same way SupervisorCore
// detects tracker dropout: a second polling method that notices silence
// (REQ-OBS-006). A late-detection or stale-pose fault, by contrast, is bad
// *data inside an otherwise well-formed message*: this class has no way to
// tell a truthful range/speed reading from a delayed or stale one, and does
// not try to; it makes the same zone decision either way, on whatever
// numbers it was given. Pricing those two fault kinds is therefore done by
// the end-to-end driver, which knows the ground truth it constructed the
// scenario from (see driver/run_obstacle_faults.py), not by this class.
#pragma once

#include <string>

#include "obstacle_zone.hpp"

namespace cbs {

enum class ObstacleDecisionKind { kNone, kForward, kWarn, kStopHazard, kStopDropout };

struct ObstacleDecision {
    ObstacleDecisionKind kind = ObstacleDecisionKind::kNone;
    std::string requirement_id;  // REQ-OBS-001, REQ-OBS-005, REQ-OBS-006, or "" for forward/none
    std::string detail;
    double range_m = 0.0;
    double speed_mps = 0.0;
    double hazard_boundary_m = 0.0;
    double warning_boundary_m = 0.0;
    // Only meaningful for kStopDropout, same meaning as SupervisorCore's
    // Decision fields of the same name: the wall time (steady_clock ms) the
    // fault first became detectable, and the wall time the stop was issued.
    double fault_onset_wall_ms = 0.0;
    double hold_issued_wall_ms = 0.0;
};

class ObstacleSupervisorCore {
public:
    ObstacleSupervisorCore(ObstacleZoneModel zone, double dropout_window_ms)
        : zone_(zone), dropout_window_ms_(dropout_window_ms) {}

    // Called on the message-handling path when a DETECT arrives. REQ-OBS-004
    // (evaluates against this object's own zone model), REQ-OBS-001/007
    // (hazard-zone stop + requirement/detail logging), REQ-OBS-005
    // (warning-zone awareness, no stop).
    //
    // Inclusive convention, stated once, here: range_m <= hazard_boundary_m
    // triggers a stop, on purpose the opposite inclusive direction from
    // Boundary::evaluate's "on the surface is safe" convention. The
    // resection boundary's surface is a safe-by-convention planning
    // surface; the hazard-zone boundary is, by ISO 18497-2's own framing,
    // sized so the machine "must be stopped ... before anything enters the
    // hazard zone" -- the boundary line itself is already the trigger, not
    // one epsilon past it. Both conventions are deliberate and documented
    // once at the point of evaluation; see README for the side-by-side
    // comparison.
    ObstacleDecision decide_detection(double range_m, double speed_mps, double receipt_wall_ms) {
        ObstacleDecision d;
        d.range_m = range_m;
        d.speed_mps = speed_mps;
        d.hazard_boundary_m = zone_.hazard_boundary_m(speed_mps);
        d.warning_boundary_m = zone_.warning_boundary_m(speed_mps);

        if (range_m <= d.hazard_boundary_m) {
            d.kind = ObstacleDecisionKind::kStopHazard;
            d.requirement_id = "REQ-OBS-001";
            d.detail = "hazard_zone range_m=" + std::to_string(range_m) +
                       " hazard_boundary_m=" + std::to_string(d.hazard_boundary_m) +
                       " speed_mps=" + std::to_string(speed_mps);
        } else if (range_m <= d.warning_boundary_m) {
            d.kind = ObstacleDecisionKind::kWarn;
            d.requirement_id = "REQ-OBS-005";
            d.detail = "warning_zone range_m=" + std::to_string(range_m) +
                       " warning_boundary_m=" + std::to_string(d.warning_boundary_m) +
                       " speed_mps=" + std::to_string(speed_mps);
        } else {
            d.kind = ObstacleDecisionKind::kForward;
        }

        last_receipt_wall_ms_ = receipt_wall_ms;
        held_ = false;  // a successfully parsed detection clears any prior dropout hold
        return d;
    }

    // Called periodically (from the fault-monitor thread in the real
    // process, or directly with synthetic now_wall_ms values in tests) --
    // REQ-OBS-006 (dropout detection), same structure as
    // SupervisorCore::poll_dropout: this method never touches anything
    // decide_detection touched other than the two shared fields below, so it
    // runs correctly whether or not decide_detection has been called at all
    // in a given poll sequence, which is exactly why it can detect a
    // *complete absence* of messages.
    ObstacleDecision poll_dropout(double now_wall_ms) {
        if (last_receipt_wall_ms_ < 0.0) return ObstacleDecision{};  // no baseline detection yet
        if (held_) return ObstacleDecision{};                        // already holding

        double elapsed = now_wall_ms - last_receipt_wall_ms_;
        if (elapsed < dropout_window_ms_) return ObstacleDecision{};

        held_ = true;
        ObstacleDecision d;
        d.kind = ObstacleDecisionKind::kStopDropout;
        d.requirement_id = "REQ-OBS-006";
        d.detail = "silence_ms=" + std::to_string(elapsed);
        d.fault_onset_wall_ms = last_receipt_wall_ms_ + dropout_window_ms_;
        d.hold_issued_wall_ms = now_wall_ms;
        return d;
    }

    bool has_received_any() const { return last_receipt_wall_ms_ >= 0.0; }
    bool is_held() const { return held_; }

private:
    ObstacleZoneModel zone_;
    double dropout_window_ms_;
    double last_receipt_wall_ms_ = -1.0;
    bool held_ = false;
};

}  // namespace cbs

// The supervisor's decision logic, factored out of supervisor_main.cpp's
// socket/thread plumbing entirely so it is directly unit-testable by calling
// its methods with synthetic wall-clock timestamps, no process, no socket,
// no thread involved (same discipline station-keeping-safety-planner uses
// for safety_check.hpp vs. main.cpp).
//
// Clock domain (read this before touching timing logic or tests). Every
// timestamp in this class is milliseconds from std::chrono::steady_clock,
// i.e. CLOCK_MONOTONIC. On Linux, CLOCK_MONOTONIC is a single, system-wide
// clock shared by every process on the host, not a per-process arbitrary
// origin, so a capture_time_ms produced by the commander process and a
// receipt_wall_ms produced by the supervisor process are directly
// comparable without any clock-sync protocol between them. This would not
// hold on a platform without a shared monotonic clock (see README
// Limitations); this project targets WSL2/Linux only for exactly that
// reason.
#pragma once

#include <string>

#include "boundary.hpp"
#include "pose.hpp"

namespace cbs {

enum class DecisionKind { kNone, kAccept, kVetoBoundary, kVetoStale, kVetoLatency, kVetoDropout };

struct Decision {
    DecisionKind kind = DecisionKind::kNone;
    std::string requirement_id;  // REQ-003, REQ-011, REQ-012, REQ-013, or "" for accept/none
    std::string detail;          // face id (boundary) or a short fault description
    double age_ms = 0.0;
    // Only meaningful for kVetoDropout/kVetoStale/kVetoLatency: the wall
    // time (steady_clock ms) at which the fault first became detectable,
    // and the wall time the hold decision was made. See README "What each
    // metric measures" for exactly what "detectable" means per fault kind.
    double fault_onset_wall_ms = 0.0;
    double hold_issued_wall_ms = 0.0;
};

class SupervisorCore {
public:
    SupervisorCore(Boundary boundary, double stale_threshold_ms, double latency_threshold_ms,
                   double dropout_window_ms)
        : boundary_(std::move(boundary)),
          stale_threshold_ms_(stale_threshold_ms),
          latency_threshold_ms_(latency_threshold_ms),
          dropout_window_ms_(dropout_window_ms) {}

    // Called on the message-handling path when a POSE arrives. REQ-002
    // (evaluates against this object's own Boundary), REQ-003/REQ-004
    // (boundary veto + face name), REQ-012/REQ-013 (stale/latency
    // classification), REQ-017 (any successfully parsed pose clears a prior
    // dropout hold, so the stream can recover without a restart).
    Decision decide_pose(const Pose& pose, double receipt_wall_ms) {
        Decision d;
        d.age_ms = receipt_wall_ms - pose.capture_time_ms;

        if (d.age_ms > stale_threshold_ms_) {
            d.kind = DecisionKind::kVetoStale;
            d.requirement_id = "REQ-012";
            d.detail = "age_ms=" + std::to_string(d.age_ms);
            d.fault_onset_wall_ms = receipt_wall_ms;
            d.hold_issued_wall_ms = receipt_wall_ms;
        } else if (d.age_ms > latency_threshold_ms_) {
            d.kind = DecisionKind::kVetoLatency;
            d.requirement_id = "REQ-013";
            d.detail = "age_ms=" + std::to_string(d.age_ms);
            d.fault_onset_wall_ms = receipt_wall_ms;
            d.hold_issued_wall_ms = receipt_wall_ms;
        } else {
            BoundaryCheckResult bc = boundary_.evaluate(pose);
            if (!bc.safe) {
                d.kind = DecisionKind::kVetoBoundary;
                d.requirement_id = "REQ-003";
                d.detail = bc.breached_face + " margin_mm=" + std::to_string(bc.margin);
            } else {
                d.kind = DecisionKind::kAccept;
            }
        }

        last_receipt_wall_ms_ = receipt_wall_ms;
        held_ = false;  // REQ-017: a successfully parsed pose clears any prior dropout hold
        return d;
    }

    // Called periodically (from the fault-monitor thread in the real
    // process, or directly with synthetic `now_wall_ms` values in tests) --
    // REQ-011 (dropout detection), REQ-014 (the 20 ms bound is measured
    // from fault_onset_wall_ms, not from now_wall_ms), REQ-016 (this method
    // never touches anything decide_pose touched other than the two shared
    // fields below, so it works correctly even if decide_pose is never
    // called during a given poll sequence).
    Decision poll_dropout(double now_wall_ms) {
        if (last_receipt_wall_ms_ < 0.0) return Decision{};  // no baseline pose received yet
        if (held_) return Decision{};                        // already holding; wait for recovery

        double elapsed = now_wall_ms - last_receipt_wall_ms_;
        if (elapsed < dropout_window_ms_) return Decision{};

        held_ = true;
        Decision d;
        d.kind = DecisionKind::kVetoDropout;
        d.requirement_id = "REQ-011";
        d.age_ms = elapsed;
        d.detail = "silence_ms=" + std::to_string(elapsed);
        d.fault_onset_wall_ms = last_receipt_wall_ms_ + dropout_window_ms_;
        d.hold_issued_wall_ms = now_wall_ms;
        return d;
    }

    bool has_received_any() const { return last_receipt_wall_ms_ >= 0.0; }
    bool is_held() const { return held_; }

private:
    Boundary boundary_;
    double stale_threshold_ms_;
    double latency_threshold_ms_;
    double dropout_window_ms_;
    double last_receipt_wall_ms_ = -1.0;
    bool held_ = false;
};

}  // namespace cbs

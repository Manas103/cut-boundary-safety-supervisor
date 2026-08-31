// A commanded tool pose. Simplification (stated once, here): this project
// models the cutting tool tip as a single 3-DOF position in the resection
// plan frame (millimeters), not a full 6-DOF position+orientation pose. The
// boundary-containment question that matters for a cut-boundary interlock
// ("is the point currently doing the cutting inside the planned envelope?")
// is fully answered by the tip position; orientation would matter for a
// tool-body collision check, which this project does not build. Extending
// `Boundary` to a full 6-DOF check would mean checking a small swept volume
// around the tip instead of a point, not a different half-space
// representation.
#pragma once

namespace cbs {

struct Pose {
    long seq = 0;
    // Wall-clock time (milliseconds, std::chrono::steady_clock, see README
    // "Clock domain") at which this pose was captured by the (simulated)
    // tracker, as opposed to when the supervisor actually receives it.
    double capture_time_ms = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

}  // namespace cbs

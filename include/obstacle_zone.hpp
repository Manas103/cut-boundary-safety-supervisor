// The obstacle-protection profile's zone-geometry model: a pure function of
// ground speed (REQ-OBS-003), mirroring how include/boundary.hpp's
// Boundary/HalfSpace are pure and unit-tested before ever touching IPC. No
// mesh, no multi-face shape here either: a stopping-distance envelope and
// two concentric zone boundaries extending ahead of the machine in its
// direction of travel, per this repo's own simplification of ISO 18497-2
// (see README "Obstacle-protection profile ... honest framing" for the full
// paraphrase and why a concentric-zone model, not an arbitrary shape, is
// what is claimed here).
#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace cbs {

struct ZoneParams {
    // Fixed detection + processing latency assumption, seconds: the time
    // between an obstacle becoming detectable and the machine actually
    // starting to brake.
    double latency_s = 0.0;
    // Fixed braking deceleration assumption, m/s^2.
    double decel_mps2 = 0.0;
    // Fixed additional safety margin added past the computed stopping
    // distance, meters.
    double margin_m = 0.0;
    // Extra lead time, seconds, converted to a distance via the current
    // ground speed and added past the hazard boundary to form the outer
    // warning-zone boundary.
    double warning_lead_s = 0.0;
};

// REQ-OBS-003: pure function of (speed, these four parameters). No IPC, no
// globals, no side effects -- directly unit-testable, exactly like
// Boundary::evaluate.
class ObstacleZoneModel {
public:
    explicit ObstacleZoneModel(ZoneParams params) : params_(params) {}

    const ZoneParams& params() const { return params_; }

    // Distance (meters) the machine travels from the instant a stop is
    // commanded to the instant it is fully stopped, given ground speed
    // speed_mps: the latency distance (still moving at speed_mps for
    // latency_s before braking begins) plus the braking distance
    // (v^2 / (2*decel), the standard constant-deceleration stopping-distance
    // formula). Speeds <= 0 stop instantly (0 m).
    double stopping_distance_m(double speed_mps) const {
        if (speed_mps <= 0.0) return 0.0;
        double latency_distance = speed_mps * params_.latency_s;
        double braking_distance = (speed_mps * speed_mps) / (2.0 * params_.decel_mps2);
        return latency_distance + braking_distance;
    }

    // REQ-OBS-001: the hazard-zone boundary, distance ahead of the machine.
    // An obstacle at or inside this distance must already have produced a
    // stop command -- this is the "must always be able to stop before the
    // hazard-zone boundary" sizing ISO 18497-2 asks for, plus a fixed
    // safety margin so the sizing is not exact-to-the-centimeter.
    double hazard_boundary_m(double speed_mps) const {
        return stopping_distance_m(speed_mps) + params_.margin_m;
    }

    // REQ-OBS-005: the warning-zone boundary, strictly larger than the
    // hazard boundary -- an obstacle between the two is a "be aware, do not
    // yet need to react" state, not a stop.
    double warning_boundary_m(double speed_mps) const {
        double lead = speed_mps > 0.0 ? speed_mps * params_.warning_lead_s : 0.0;
        return hazard_boundary_m(speed_mps) + lead;
    }

    // REQ-OBS-004: loads the four zone parameters from a plaintext config
    // file (config/obstacle_zone.txt), not hardcoded in source, mirroring
    // Boundary::load_from_file. Throws std::runtime_error on a malformed or
    // incomplete file rather than silently starting with a partial or
    // zeroed-out envelope.
    static ObstacleZoneModel load_from_file(const std::string& path) {
        std::ifstream in(path);
        if (!in) {
            throw std::runtime_error("obstacle zone config not found: " + path);
        }
        ZoneParams p;
        bool have_latency = false, have_decel = false, have_margin = false, have_lead = false;
        std::string line;
        while (std::getline(in, line)) {
            std::string trimmed = trim(line);
            if (trimmed.empty() || trimmed[0] == '#') continue;
            std::istringstream iss(trimmed);
            std::string key;
            double value = 0.0;
            if (!(iss >> key >> value)) {
                throw std::runtime_error("obstacle zone config: malformed line: " + line);
            }
            if (key == "LATENCY_S") {
                p.latency_s = value;
                have_latency = true;
            } else if (key == "DECEL_MPS2") {
                p.decel_mps2 = value;
                have_decel = true;
            } else if (key == "MARGIN_M") {
                p.margin_m = value;
                have_margin = true;
            } else if (key == "WARNING_LEAD_S") {
                p.warning_lead_s = value;
                have_lead = true;
            } else {
                throw std::runtime_error("obstacle zone config: unknown key '" + key + "' in line: " + line);
            }
        }
        if (!(have_latency && have_decel && have_margin && have_lead)) {
            throw std::runtime_error("obstacle zone config: missing one or more required keys "
                                      "(LATENCY_S, DECEL_MPS2, MARGIN_M, WARNING_LEAD_S): " + path);
        }
        if (p.decel_mps2 <= 0.0) {
            throw std::runtime_error("obstacle zone config: DECEL_MPS2 must be > 0: " + path);
        }
        return ObstacleZoneModel(p);
    }

private:
    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    ZoneParams params_;
};

}  // namespace cbs

// The convex resection boundary: an ordered list of half-space constraints,
// and the pure containment check the whole veto decision rests on (REQ-003,
// REQ-007, REQ-008, REQ-009). Deliberately the only geometry in this repo:
// no mesh, no BVH, nothing that isn't needed to state "is this point inside
// the intersection of these half-spaces".
#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pose.hpp"

namespace cbs {

struct HalfSpace {
    std::string face_id;
    double nx = 0.0, ny = 0.0, nz = 0.0;
    double d = 0.0;
};

struct BoundaryCheckResult {
    bool safe = true;
    // Empty when safe. Set to the first (in declaration order) breached
    // face's id when unsafe -- order is part of the contract, see
    // Boundary::evaluate.
    std::string breached_face;
    // dot(normal, position) - d for the breached face when unsafe (positive
    // = how far outside); 0.0 when safe.
    double margin = 0.0;
};

class Boundary {
public:
    void add_half_space(HalfSpace hs) { half_spaces_.push_back(std::move(hs)); }

    const std::vector<HalfSpace>& half_spaces() const { return half_spaces_; }

    // REQ-009: dot(normal, position) == d (to within a small numerical
    // tolerance, since these are floating-point comparisons on values
    // loaded from a text file) is classified as inside/safe. The boundary
    // surface itself belongs to the safe side, i.e. every FACE constraint
    // is "<=", not "<". A resection boundary is a planning surface with its
    // own finite width in reality; treating the mathematical surface as
    // safe is the conservative-by-convention choice documented here rather
    // than left implicit, and `BoundaryValueTest` in tests/boundary_test.cpp
    // pins it down exactly.
    static constexpr double kTolerance = 1e-9;

    // REQ-003, REQ-007: pure function of (pose, this boundary's half-spaces).
    // No IPC, no globals, no side effects -- directly unit-testable.
    // Faces are checked in declaration order and the first breach found is
    // reported (deterministic, and cheap since there is no expensive rule
    // to defer the way station-keeping-safety-planner defers its corridor
    // check; every face here is one dot product).
    BoundaryCheckResult evaluate(const Pose& p) const {
        for (const auto& hs : half_spaces_) {
            double value = hs.nx * p.x + hs.ny * p.y + hs.nz * p.z;
            double margin = value - hs.d;
            if (margin > kTolerance) {
                return BoundaryCheckResult{false, hs.face_id, margin};
            }
        }
        return BoundaryCheckResult{true, "", 0.0};
    }

    // REQ-008: loads the half-space list from a plaintext config file, see
    // config/boundary.txt for the format and the shipped envelope. Throws
    // std::runtime_error on a malformed line rather than silently skipping
    // it -- a boundary file that fails to parse should stop the supervisor
    // from starting, not start it with an incomplete envelope.
    static Boundary load_from_file(const std::string& path) {
        std::ifstream in(path);
        if (!in) {
            throw std::runtime_error("boundary file not found: " + path);
        }
        Boundary b;
        std::string line;
        while (std::getline(in, line)) {
            std::string trimmed = trim(line);
            if (trimmed.empty() || trimmed[0] == '#') continue;
            std::istringstream iss(trimmed);
            std::string tag;
            iss >> tag;
            if (tag != "FACE") {
                throw std::runtime_error("boundary file: expected FACE, got '" + tag + "' in line: " + line);
            }
            HalfSpace hs;
            if (!(iss >> hs.face_id >> hs.nx >> hs.ny >> hs.nz >> hs.d)) {
                throw std::runtime_error("boundary file: malformed FACE line: " + line);
            }
            b.add_half_space(hs);
        }
        if (b.half_spaces().empty()) {
            throw std::runtime_error("boundary file defines zero half-spaces: " + path);
        }
        return b;
    }

private:
    static std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    std::vector<HalfSpace> half_spaces_;
};

}  // namespace cbs

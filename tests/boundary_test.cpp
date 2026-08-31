// Boundary geometry: REQ-003, REQ-007, REQ-008, REQ-009.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "boundary.hpp"
#include "req_trace.hpp"

using namespace cbs;

namespace {

Boundary make_test_box() {
    // A simple 10x10x10 box, [0,10]^3, no slanted face -- kept separate
    // from config/boundary.txt (the shipped envelope) so this test's
    // expected values don't depend on the shipped file's exact numbers.
    Boundary b;
    b.add_half_space({"x_min", -1, 0, 0, 0});
    b.add_half_space({"x_max", 1, 0, 0, 10});
    b.add_half_space({"y_min", 0, -1, 0, 0});
    b.add_half_space({"y_max", 0, 1, 0, 10});
    b.add_half_space({"z_min", 0, 0, -1, 0});
    b.add_half_space({"z_max", 0, 0, 1, 10});
    return b;
}

}  // namespace

TEST(Boundary, InteriorPointIsSafe) {
    TRACE_REQ("REQ-003");
    TRACE_REQ("REQ-007");
    Boundary b = make_test_box();
    auto r = b.evaluate(Pose{1, 0.0, 5, 5, 5});
    EXPECT_TRUE(r.safe);
    EXPECT_EQ(r.breached_face, "");
}

TEST(Boundary, PointOutsideEachFaceIsUnsafeAndNamesThatFace) {
    TRACE_REQ("REQ-003");
    TRACE_REQ("REQ-007");
    Boundary b = make_test_box();

    struct Case {
        Pose pose;
        std::string expected_face;
    };
    std::vector<Case> cases = {
        {{1, 0.0, -1, 5, 5}, "x_min"},  {{2, 0.0, 11, 5, 5}, "x_max"}, {{3, 0.0, 5, -1, 5}, "y_min"},
        {{4, 0.0, 5, 11, 5}, "y_max"},  {{5, 0.0, 5, 5, -1}, "z_min"}, {{6, 0.0, 5, 5, 11}, "z_max"},
    };
    for (const auto& c : cases) {
        auto r = b.evaluate(c.pose);
        EXPECT_FALSE(r.safe) << "face " << c.expected_face;
        EXPECT_EQ(r.breached_face, c.expected_face);
        EXPECT_GT(r.margin, 0.0);
    }
}

TEST(Boundary, FirstBreachedFaceInDeclarationOrderIsReported) {
    TRACE_REQ("REQ-003");
    Boundary b = make_test_box();
    // Violates x_min (declared first) and z_max (declared last) simultaneously.
    auto r = b.evaluate(Pose{7, 0.0, -5, 5, 50});
    EXPECT_FALSE(r.safe);
    EXPECT_EQ(r.breached_face, "x_min");
}

// REQ-009: the boundary surface itself (dot(n,p) == d exactly) is inside/safe.
TEST(Boundary, ExactBoundaryValueIsClassifiedSafe) {
    TRACE_REQ("REQ-009");
    Boundary b = make_test_box();
    auto r = b.evaluate(Pose{8, 0.0, 10, 5, 5});  // exactly on x_max's surface
    EXPECT_TRUE(r.safe);
    // One unit past the surface is unsafe -- confirms this isn't a wide
    // tolerance band swallowing real violations.
    auto r2 = b.evaluate(Pose{9, 0.0, 10.001, 5, 5});
    EXPECT_FALSE(r2.safe);
}

TEST(Boundary, NonAxisAlignedHalfSpaceIsEvaluatedCorrectly) {
    TRACE_REQ("REQ-003");
    TRACE_REQ("REQ-007");
    Boundary b = make_test_box();
    b.add_half_space({"cut_plane", 0.5, 0.5, 0.5, 12.0});  // x+y+z <= 24
    auto safe = b.evaluate(Pose{10, 0.0, 5, 5, 5});         // sum=15 <= 24
    EXPECT_TRUE(safe.safe);
    auto unsafe = b.evaluate(Pose{11, 0.0, 9, 9, 9});       // sum=27 > 24, still inside the box
    EXPECT_FALSE(unsafe.safe);
    EXPECT_EQ(unsafe.breached_face, "cut_plane");
}

// REQ-008: loaded from the shipped plaintext config, not hardcoded.
TEST(Boundary, LoadsShippedBoundaryConfigFile) {
    TRACE_REQ("REQ-008");
    Boundary b = Boundary::load_from_file("../config/boundary.txt");
    ASSERT_EQ(b.half_spaces().size(), 7u);
    EXPECT_EQ(b.half_spaces()[0].face_id, "x_min");
    // Interior of the shipped [0,100]x[0,60]x[0,40] envelope, away from the
    // slanted cut_plane corner.
    auto r = b.evaluate(Pose{1, 0.0, 50, 30, 20});
    EXPECT_TRUE(r.safe);
    // Outside on x_max.
    auto r2 = b.evaluate(Pose{2, 0.0, 150, 30, 20});
    EXPECT_FALSE(r2.safe);
    EXPECT_EQ(r2.breached_face, "x_max");
}

TEST(Boundary, MissingFileThrows) {
    TRACE_REQ("REQ-008");
    EXPECT_THROW(Boundary::load_from_file("does_not_exist.txt"), std::runtime_error);
}

TEST(Boundary, MalformedFaceLineThrows) {
    TRACE_REQ("REQ-008");
    // Written to a temp file next to the build dir so this test is
    // self-contained and doesn't depend on any other fixture file.
    const char* path = "malformed_boundary_test_fixture.txt";
    {
        std::ofstream out(path);
        out << "FACE x_min -1 0 0\n";  // missing the d value
    }
    EXPECT_THROW(Boundary::load_from_file(path), std::runtime_error);
    std::remove(path);
}

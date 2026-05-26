#include <catch2/catch_all.hpp>

#include <libslic3r/FiveAxis/GCodeStitcher.hpp>

using namespace Slic3r;
using namespace Slic3r::FiveAxis;

namespace {

StitchInput sample_two_region_input()
{
    StitchInput in;
    in.tool_length_LB_mm = 50.0;
    in.tool_length_LC_mm = 20.0;
    in.transition_speed_mm_per_min = 600.0;
    in.park_x_mm = 5.0;
    in.park_y_mm = 5.0;
    in.safety_margin_mm = 10.0;

    // Region 1: background — identity build direction → B=0, C=0
    StitchRegion bg;
    bg.id = "background";
    bg.gcode = "; bg region\nG1 X10 Y10 Z0.2 E1.0 F1200\nG1 X20 Y10 Z0.2 E2.0\n";
    bg.build_direction.rotation = Eigen::Matrix3d::Identity();
    bg.world_bbox = BoundingBoxf3({0.0, 0.0, 0.0}, {30.0, 30.0, 20.0});
    in.regions.push_back(std::move(bg));

    // Region 2: arm — rotated 90° about Y, so local +Z → world +X → B=90, C=0
    StitchRegion arm;
    arm.id = "modifier_arm";
    arm.gcode = "; arm region\nG1 X10 Y10 Z0.2 E1.0 F1200\n";
    arm.build_direction.rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    arm.world_bbox = BoundingBoxf3({30.0, 0.0, 0.0}, {50.0, 30.0, 20.0});
    in.regions.push_back(std::move(arm));

    return in;
}

} // anonymous

TEST_CASE("GCodeStitcher: emits G43.4 exactly once near the top", "[five_axis][stitcher]")
{
    GCodeStitcher s;
    const std::string out = s.stitch(sample_two_region_input());

    const auto first = out.find("G43.4");
    REQUIRE(first != std::string::npos);
    const auto second = out.find("G43.4", first + 5);
    REQUIRE(second == std::string::npos);
}

TEST_CASE("GCodeStitcher: appends B/C to every G1 motion in each region", "[five_axis][stitcher]")
{
    GCodeStitcher s;
    const std::string out = s.stitch(sample_two_region_input());

    // Background region's G1 lines carry B0 C0
    const auto bg_pos = out.find("; bg region");
    REQUIRE(bg_pos != std::string::npos);
    const auto bg_g1 = out.find("G1 X10 Y10 Z0.2", bg_pos);
    REQUIRE(bg_g1 != std::string::npos);
    const auto bg_eol = out.find("\n", bg_g1);
    const std::string bg_line = out.substr(bg_g1, bg_eol - bg_g1);
    REQUIRE(bg_line.find(" B0") != std::string::npos);
    REQUIRE(bg_line.find(" C0") != std::string::npos);

    // Arm region's G1 lines carry B90 C0
    const auto arm_pos = out.find("; arm region");
    REQUIRE(arm_pos != std::string::npos);
    const auto arm_g1 = out.find("G1", arm_pos);
    REQUIRE(arm_g1 != std::string::npos);
    const auto arm_eol = out.find("\n", arm_g1);
    const std::string arm_line = out.substr(arm_g1, arm_eol - arm_g1);
    REQUIRE(arm_line.find(" B90") != std::string::npos);
    REQUIRE(arm_line.find(" C0") != std::string::npos);
}

TEST_CASE("GCodeStitcher: transition sequence is retract → lift → park → rotate → deretract → marker",
          "[five_axis][stitcher]")
{
    GCodeStitcher s;
    const std::string out = s.stitch(sample_two_region_input());

    const auto marker = out.find("; INTERFACE LAYER");
    REQUIRE(marker != std::string::npos);

    // Walk back from the marker — every preceding step must appear in the right order.
    const auto deretract = out.rfind("G11", marker);
    REQUIRE(deretract != std::string::npos);
    REQUIRE(deretract < marker);

    const auto rotate = out.rfind("rotate to new orientation", deretract);
    REQUIRE(rotate != std::string::npos);
    REQUIRE(rotate < deretract);

    const auto park = out.rfind("park at bed corner", rotate);
    REQUIRE(park != std::string::npos);
    REQUIRE(park < rotate);

    const auto lift = out.rfind("lift straight up", park);
    REQUIRE(lift != std::string::npos);
    REQUIRE(lift < park);

    const auto retract = out.rfind("G10", lift);
    REQUIRE(retract != std::string::npos);
    REQUIRE(retract < lift);
}

TEST_CASE("GCodeStitcher: safe-Z rises above the tallest printed region + tool arm + margin",
          "[five_axis][stitcher]")
{
    // bg world_bbox max.z = 20 → safe_z = 20 + tool_LB(50) + margin(10) = 80
    GCodeStitcher s;
    const std::string out = s.stitch(sample_two_region_input());

    const auto lift = out.find("lift straight up");
    REQUIRE(lift != std::string::npos);

    const auto line_start = out.rfind('\n', lift) + 1;
    const std::string lift_line = out.substr(line_start, lift - line_start);
    REQUIRE(lift_line.find("G0 Z80") != std::string::npos);
}

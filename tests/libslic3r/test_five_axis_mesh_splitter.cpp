#include <catch2/catch_all.hpp>
#include "test_utils.hpp"

#include <libslic3r/FiveAxis/MeshSplitter.hpp>
#include <libslic3r/TriangleMesh.hpp>

using namespace Slic3r;
using namespace Slic3r::FiveAxis;

namespace {

// L-shape: 20x10x40 vertical post (the "I") + 30x10x10 horizontal arm (the "_")
// attached to its right side. Origin at lower-left of the I, bed = z=0.
TriangleMesh make_l_shape()
{
    TriangleMesh I = make_cube(20.0, 10.0, 40.0);
    TriangleMesh arm = make_cube(30.0, 10.0, 10.0);
    arm.translate(20.0f, 0.0f, 0.0f);
    I.merge(arm);
    return I;
}

// Modifier covering just the horizontal arm of the L, rotated 90° about Y so the
// modifier's local +Z axis points to world +X — the "print sideways" direction.
ModifierVolume make_arm_modifier()
{
    ModifierVolume m;
    m.id = "arm";
    m.mesh = make_cube(32.0, 12.0, 12.0);
    m.mesh.translate(19.0f, -1.0f, -1.0f);
    m.transform = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return m;
}

} // anonymous

TEST_CASE("MeshSplitter: empty modifier list yields background-only", "[five_axis][mesh_splitter]")
{
    MeshSplitter splitter;
    const TriangleMesh L = make_l_shape();
    const std::vector<ModifierVolume> mods;
    const std::vector<Region> regions = splitter.split(L, mods);

    REQUIRE(regions.size() == 1);
    REQUIRE(regions[0].id == "background");
    REQUIRE(regions[0].source_modifier_id.empty());
    REQUIRE(regions[0].mesh.its.vertices.size() > 0);
}

TEST_CASE("MeshSplitter: one modifier yields two regions (background + modified)", "[five_axis][mesh_splitter]")
{
    MeshSplitter splitter;
    const TriangleMesh L = make_l_shape();
    const std::vector<ModifierVolume> mods = { make_arm_modifier() };
    const std::vector<Region> regions = splitter.split(L, mods);

    REQUIRE(regions.size() == 2);

    const Region* bg = nullptr;
    const Region* arm = nullptr;
    for (const auto& r : regions) {
        if (r.id == "background") bg = &r;
        else if (r.id == "modifier_arm") arm = &r;
    }
    REQUIRE(bg != nullptr);
    REQUIRE(arm != nullptr);

    // Background should be approximately the I (vertical post). Its bbox max.x ~= 20.
    const BoundingBoxf3 bg_bb = bg->mesh.bounding_box();
    REQUIRE(bg_bb.max.x() < 22.0);

    // Arm region's bbox should reach world x ~ 50 (arm extends to x=20+30=50).
    const BoundingBoxf3 arm_bb = arm->mesh.bounding_box();
    REQUIRE(arm_bb.max.x() > 45.0);

    // Arm's build direction reports B ≈ 90, C ≈ 0
    REQUIRE(std::abs(arm->build_direction.B_degrees() - 90.0) < 0.5);
    REQUIRE(std::abs(arm->build_direction.C_degrees()) < 0.5);
}

TEST_CASE("MeshSplitter: produced sub-meshes are non-empty", "[five_axis][mesh_splitter]")
{
    MeshSplitter splitter;
    const TriangleMesh L = make_l_shape();
    const std::vector<ModifierVolume> mods = { make_arm_modifier() };
    const std::vector<Region> regions = splitter.split(L, mods);

    for (const auto& r : regions) {
        REQUIRE(r.mesh.its.vertices.size() > 0);
        REQUIRE(r.mesh.its.indices.size() > 0);
    }
}

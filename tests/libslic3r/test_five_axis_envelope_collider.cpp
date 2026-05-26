#include <catch2/catch_all.hpp>

#include <libslic3r/FiveAxis/EnvelopeCollider.hpp>
#include <libslic3r/TriangleMesh.hpp>

using namespace Slic3r;
using namespace Slic3r::FiveAxis;

namespace {

Region make_box_region(const std::string& id,
                       double xmin, double ymin, double zmin,
                       double xmax, double ymax, double zmax)
{
    Region r;
    r.id = id;
    TriangleMesh box = make_cube(xmax - xmin, ymax - ymin, zmax - zmin);
    box.translate((float)xmin, (float)ymin, (float)zmin);
    r.mesh = std::move(box);
    return r;
}

} // anonymous

TEST_CASE("EnvelopeCollider: clear space → no collision", "[five_axis][envelope]")
{
    EnvelopeCollider c;
    ToolheadEnvelope e{25.0, 60.0, 15.0};
    std::vector<Region> prior = { make_box_region("I", 0, 0, 0, 20, 10, 40) };

    REQUIRE_FALSE(c.collides(e, Eigen::Vector3d(200.0, 200.0, 10.0), prior));
}

TEST_CASE("EnvelopeCollider: cylinder overlapping bbox → collision", "[five_axis][envelope]")
{
    EnvelopeCollider c;
    ToolheadEnvelope e{25.0, 60.0, 15.0};
    std::vector<Region> prior = { make_box_region("I", 0, 0, 0, 20, 10, 40) };

    // Nozzle 5mm past the right edge of I — 25mm head radius reaches into the I's bbox.
    REQUIRE(c.collides(e, Eigen::Vector3d(25.0, 5.0, 35.0), prior));
}

TEST_CASE("EnvelopeCollider: sweep reports each colliding step", "[five_axis][envelope]")
{
    EnvelopeCollider c;
    ToolheadEnvelope e{25.0, 60.0, 15.0};
    std::vector<Region> prior = { make_box_region("I", 0, 0, 0, 20, 10, 40) };
    std::vector<Eigen::Vector3d> path = {
        {200, 200, 10},   // far
        {25,    5, 35},   // close
        {25,    5, 30},   // close
        {200, 200, 10},   // far
    };

    const auto evs = c.sweep(e, "arm", path, prior);
    REQUIRE(evs.size() == 2);
    REQUIRE(evs[0].region_collided_with == "I");
    REQUIRE(evs[0].region_being_printed == "arm");
}

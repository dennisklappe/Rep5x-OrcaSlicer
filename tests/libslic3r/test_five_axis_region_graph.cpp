#include <catch2/catch_all.hpp>

#include <libslic3r/FiveAxis/RegionGraph.hpp>
#include <libslic3r/TriangleMesh.hpp>

using namespace Slic3r;
using namespace Slic3r::FiveAxis;

namespace {

// Build a region whose mesh occupies a simple cube spanning (-10..10, -10..10, zmin..zmax).
Region make_region_at(const std::string& id, double zmin, double zmax)
{
    Region r;
    r.id = id;
    TriangleMesh box = make_cube(20.0, 20.0, zmax - zmin);
    box.translate(-10.0f, -10.0f, (float)zmin);
    r.mesh = std::move(box);
    return r;
}

} // anonymous

TEST_CASE("RegionGraph: bed-touching region orders first", "[five_axis][region_graph]")
{
    std::vector<Region> regions;
    regions.push_back(make_region_at("a", 0.0, 20.0));     // touches bed
    regions.push_back(make_region_at("b", 20.0, 30.0));    // sits above a

    RegionGraph g;
    g.compute_default_order(regions);

    REQUIRE(regions[0].id == "a");
    REQUIRE(regions[0].order == 0);
    REQUIRE(regions[1].order == 1);
}

TEST_CASE("RegionGraph: warns when user order leaves a region floating", "[five_axis][region_graph]")
{
    std::vector<Region> regions;
    regions.push_back(make_region_at("a", 0.0, 20.0));
    regions.push_back(make_region_at("b", 20.0, 30.0));

    regions[0].order = 1;   // bed region prints second
    regions[1].order = 0;   // floating region prints first — physically wrong

    RegionGraph g;
    const std::vector<std::string> warnings = g.validate_order(regions);

    REQUIRE(warnings.size() >= 1);
    REQUIRE(warnings[0].find("'b'") != std::string::npos);
}

TEST_CASE("RegionGraph: stable when all regions touch bed", "[five_axis][region_graph]")
{
    std::vector<Region> regions;
    regions.push_back(make_region_at("a", 0.0, 10.0));
    regions.push_back(make_region_at("b", 0.0, 10.0));

    RegionGraph g;
    g.compute_default_order(regions);

    REQUIRE(regions[0].order < regions[1].order);
}

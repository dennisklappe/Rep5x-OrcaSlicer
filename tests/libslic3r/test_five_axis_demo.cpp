#include <catch2/catch_all.hpp>

#include <libslic3r/FiveAxis/Rep5xDemo.hpp>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace Slic3r::FiveAxis;

// Tagged [.slow] because Rep5xDemo::run_l_shape spawns subprocesses (real Orca slicer
// per region). Excluded from default runs. Also skipped automatically when run from
// the test binary rather than the real orca-slicer binary, because run_l_shape resolves
// the slicer binary via /proc/self/exe.
TEST_CASE("Rep5xDemo: run_l_shape produces a valid Rep5x G-code via real slicer",
          "[five_axis][demo][.slow]")
{
    char self[4096] = {0};
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    REQUIRE(n > 0);
    const std::string self_path(self);
    if (self_path.find("orca-slicer") == std::string::npos) {
        SUCCEED("Skipped: only runs when self-exe is orca-slicer; use --rep5x-demo-l CLI flag.");
        return;
    }

    const std::string out = "/tmp/rep5x_demo_l_test.gcode";
    std::remove(out.c_str());

    const int rc = Rep5xDemo::run_l_shape(out);
    REQUIRE(rc == 0);

    std::ifstream f(out);
    REQUIRE(f.good());
    std::stringstream ss; ss << f.rdbuf();
    const std::string g = ss.str();

    REQUIRE(g.size() > 5000);
    REQUIRE(g.find("G43.4") != std::string::npos);
    REQUIRE(g.find("region: background") != std::string::npos);
    REQUIRE(g.find("region: modifier_arm") != std::string::npos);
    REQUIRE(g.find("; INTERFACE LAYER") != std::string::npos);
    REQUIRE(g.find(" B90") != std::string::npos);
    REQUIRE((g.find(";LAYER_CHANGE") != std::string::npos
          || g.find("; layer ") != std::string::npos));
}

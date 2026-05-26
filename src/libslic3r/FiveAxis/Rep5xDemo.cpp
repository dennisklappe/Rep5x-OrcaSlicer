#include "Rep5xDemo.hpp"
#include "Rep5xPipeline.hpp"
#include "MeshSplitter.hpp"
#include <libslic3r/TriangleMesh.hpp>

#include <filesystem>
#include <iostream>
#include <unistd.h>

namespace Slic3r {
namespace FiveAxis {

namespace {

// Returns the L positioned in the middle of a 220x220 bed,
// safely clear of the start-G-code prime line at Y=10..140 along X≈2.
TriangleMesh make_l_shape()
{
    constexpr float ORIGIN_X = 90.0f;
    constexpr float ORIGIN_Y = 100.0f;
    TriangleMesh I = make_cube(20.0, 10.0, 40.0);
    TriangleMesh arm = make_cube(30.0, 10.0, 10.0);
    arm.translate(20.0f, 0.0f, 0.0f);
    I.merge(arm);
    I.translate(ORIGIN_X, ORIGIN_Y, 0.0f);
    return I;
}

bool resolve_paths(std::filesystem::path& bin_path, std::filesystem::path& repo_root)
{
    char self[4096] = {0};
    const ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n < 0) return false;
    bin_path = std::filesystem::path(self);
    repo_root = bin_path.parent_path().parent_path().parent_path().parent_path();
    return true;
}

} // anonymous

int Rep5xDemo::run_l_shape(const std::string& out_path)
{
    std::filesystem::path bin_path, repo_root;
    if (!resolve_paths(bin_path, repo_root)) {
        std::cerr << "rep5x-demo-l: cannot resolve binary path via /proc/self/exe\n";
        return 4;
    }

    Rep5xScene scene;
    scene.main_mesh = make_l_shape();

    ModifierVolume arm_mod;
    arm_mod.id = "arm";
    arm_mod.mesh = make_cube(32.0, 12.0, 12.0);
    arm_mod.mesh.translate(90.0f + 19.0f, 100.0f - 1.0f, -1.0f);
    arm_mod.transform = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    scene.modifiers.push_back(std::move(arm_mod));

    scene.orca_binary_path      = bin_path.string();
    scene.machine_profile_path  = (repo_root / "resources/profiles/Rep5x/machine/Rep5x Ender 5 Pro 0.4 nozzle.json").string();
    scene.process_profile_path  = (repo_root / "resources/profiles/Rep5x/process/0.20mm Standard @Rep5x.json").string();
    scene.filament_profile_path = (repo_root / "resources/profiles/Rep5x/filament/Generic PLA @Rep5x.json").string();

    return Rep5xPipeline::run(scene, out_path);
}

} // namespace FiveAxis
} // namespace Slic3r

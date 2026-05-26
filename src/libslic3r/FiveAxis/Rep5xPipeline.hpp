#pragma once

#include "MeshSplitter.hpp"
#include <libslic3r/TriangleMesh.hpp>
#include <string>
#include <vector>

namespace Slic3r {
namespace FiveAxis {

// All inputs needed to run the 5-axis pipeline once.
struct Rep5xScene
{
    TriangleMesh main_mesh;
    std::vector<ModifierVolume> modifiers;

    // Absolute paths to slicer binary + system profiles. Caller resolves these.
    std::string orca_binary_path;
    std::string machine_profile_path;
    std::string process_profile_path;
    std::string filament_profile_path;

    // Stitcher params (sensible Rep5x defaults).
    double tool_length_LB_mm = 50.0;
    double tool_length_LC_mm = 20.0;
    // (legacy: safe_z_mm has been replaced by dynamic computation in GCodeStitcher
    // from world-bbox + tool length + safety margin. Field removed.)
    double transition_speed_mm_per_min = 600.0;
};

class Rep5xPipeline
{
public:
    // Runs MeshSplitter → RegionGraph → Rep5xSlicerInvoker → GCodeStitcher
    // and writes the result to <out_path>. Returns 0 on success, non-zero on error.
    static int run(const Rep5xScene& scene, const std::string& out_path);
};

} // namespace FiveAxis
} // namespace Slic3r

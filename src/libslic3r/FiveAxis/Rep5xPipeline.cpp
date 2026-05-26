#include "Rep5xPipeline.hpp"
#include "MeshSplitter.hpp"
#include "RegionGraph.hpp"
#include "GCodeStitcher.hpp"
#include "EnvelopeCollider.hpp"
#include "Rep5xSlicerInvoker.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace Slic3r {
namespace FiveAxis {

int Rep5xPipeline::run(const Rep5xScene& scene, const std::string& out_path)
{
    MeshSplitter splitter;
    std::vector<Region> regions = splitter.split(scene.main_mesh, scene.modifiers);
    if (regions.empty()) {
        std::cerr << "rep5x: MeshSplitter produced no regions\n";
        return 2;
    }
    RegionGraph graph;
    graph.compute_default_order(regions);
    std::sort(regions.begin(), regions.end(),
              [](const Region& a, const Region& b) { return a.order < b.order; });

    Rep5xSlicerInvoker::Config cfg;
    cfg.orca_binary_path      = scene.orca_binary_path;
    cfg.machine_profile_path  = scene.machine_profile_path;
    cfg.process_profile_path  = scene.process_profile_path;
    cfg.filament_profile_path = scene.filament_profile_path;
    cfg.scratch_dir           = "/tmp/rep5x-region";
    Rep5xSlicerInvoker invoker(std::move(cfg));

    StitchInput stitch_in;
    stitch_in.tool_length_LB_mm = scene.tool_length_LB_mm;
    stitch_in.tool_length_LC_mm = scene.tool_length_LC_mm;
    stitch_in.transition_speed_mm_per_min = scene.transition_speed_mm_per_min;

    // Bonding offset: shift non-first regions by 0.1mm along the negative build direction
    // so the first layer presses INTO the previously-printed region for mechanical bond.
    constexpr double BONDING_PRESS_MM = 0.1;

    bool is_first_region = true;
    for (const auto& r : regions) {
        StitchRegion sr;
        sr.id = r.id;
        sr.build_direction = r.build_direction;
        sr.world_bbox = r.mesh.bounding_box();   // used by stitcher for dynamic safe-Z
        try {
            const double bonding = is_first_region ? 0.0 : BONDING_PRESS_MM;
            sr.gcode = invoker.slice_region(r, bonding);
        } catch (const std::exception& e) {
            std::cerr << "rep5x: slicer failed for region '" << r.id
                      << "': " << e.what() << "\n";
            return 5;
        }
        if (!is_first_region) {
            const auto first_layer = sr.gcode.find(";LAYER_CHANGE");
            if (first_layer != std::string::npos) sr.gcode = sr.gcode.substr(first_layer);
        }
        is_first_region = false;
        stitch_in.regions.push_back(std::move(sr));
    }

    if (regions.size() >= 2) {
        EnvelopeCollider col;
        ToolheadEnvelope env;
        const auto& bb = regions[1].mesh.bounding_box();
        const std::vector<Eigen::Vector3d> sample = {
            { bb.min.x(), bb.min.y(), bb.min.z() },
            { bb.max.x(), bb.max.y(), bb.max.z() }
        };
        const auto evs = col.sweep(env, regions[1].id, sample, std::vector<Region>{ regions[0] });
        if (!evs.empty()) {
            std::cerr << "rep5x: envelope collision detected (" << evs.size()
                      << " events). G-code still written for inspection.\n";
        }
    }

    GCodeStitcher stitcher;
    const std::string gcode = stitcher.stitch(stitch_in);

    std::ofstream f(out_path);
    if (!f) {
        std::cerr << "rep5x: cannot write to " << out_path << "\n";
        return 3;
    }
    f << gcode;
    f.close();
    std::cerr << "rep5x: wrote " << out_path
              << " (" << gcode.size() << " bytes)\n";
    return 0;
}

} // namespace FiveAxis
} // namespace Slic3r

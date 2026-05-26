#pragma once

#include "RegionTypes.hpp"
#include <libslic3r/TriangleMesh.hpp>
#include <string>

namespace Slic3r {
namespace FiveAxis {

// Drives Orca's planar slicer as a subprocess on a per-region sub-mesh.
class Rep5xSlicerInvoker
{
public:
    struct Config
    {
        std::string orca_binary_path;
        std::string machine_profile_path;
        std::string process_profile_path;
        std::string filament_profile_path;
        std::string scratch_dir = "/tmp/rep5x-region";
    };

    Rep5xSlicerInvoker() = default;
    explicit Rep5xSlicerInvoker(Config cfg) : m_cfg(std::move(cfg)) {}

    // Slices one region:
    //   1. Writes region.mesh to <scratch_dir>/<region.id>.stl
    //   2. Runs orca-slicer CLI with the configured profiles
    //   3. Reads the produced .gcode (Orca writes "plate_1.gcode") and returns it
    //
    // bonding_offset_mm: shift every output coordinate by -offset along the region's
    //   build direction in world space. Used to press a region's first layer slightly
    //   INTO the previously-printed region for mechanical bonding. 0 = no shift.
    //
    // Throws std::runtime_error on subprocess failure or missing output.
    std::string slice_region(const Region& region, double bonding_offset_mm = 0.0) const;

private:
    Config m_cfg;
};

} // namespace FiveAxis
} // namespace Slic3r

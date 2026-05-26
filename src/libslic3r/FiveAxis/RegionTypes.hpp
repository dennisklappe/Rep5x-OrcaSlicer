#pragma once

#include <libslic3r/TriangleMesh.hpp>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace Slic3r {
namespace FiveAxis {

// The modifier's local +Z axis in world coordinates IS the build direction.
// Identity = print straight up (default Z).
struct BuildDirection
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    // B = angle off vertical (degrees, 0..180). C = rotation around vertical (degrees, -180..180).
    double B_degrees() const;
    double C_degrees() const;

    // Apply transposed rotation to a sub-mesh so its build direction becomes +Z locally.
    Eigen::Matrix3d inverse() const { return rotation.transpose(); }
};

// One region produced by MeshSplitter. Background region has empty source_modifier_id.
struct Region
{
    std::string id;                 // "background" or "modifier_<id>"
    std::string source_modifier_id; // empty for background
    TriangleMesh mesh;              // sub-mesh in WORLD coordinates
    BuildDirection build_direction; // identity for background
    int order = -1;                 // filled by RegionGraph::compute_default_order
};

} // namespace FiveAxis
} // namespace Slic3r

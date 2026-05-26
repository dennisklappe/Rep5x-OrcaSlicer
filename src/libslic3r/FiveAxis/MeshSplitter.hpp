#pragma once

#include "RegionTypes.hpp"
#include <libslic3r/TriangleMesh.hpp>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace Slic3r {
namespace FiveAxis {

// A user-defined modifier volume — a mesh in world coordinates plus a rotation
// that defines the build direction for any geometry inside the volume.
struct ModifierVolume
{
    std::string id;
    TriangleMesh mesh;
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
};

class MeshSplitter
{
public:
    // Splits the main mesh against the modifier volumes:
    //   - For each modifier M: region_M = (main_mesh ∩ M.mesh), build_direction = M.transform
    //   - Background: main_mesh - union(all modifier meshes), build_direction = identity
    // Returns regions ordered as [background, modifier1, modifier2, ...].
    // If modifiers is empty, returns one background region equal to main_mesh.
    std::vector<Region> split(
        const TriangleMesh& main_mesh,
        const std::vector<ModifierVolume>& modifiers) const;
};

} // namespace FiveAxis
} // namespace Slic3r

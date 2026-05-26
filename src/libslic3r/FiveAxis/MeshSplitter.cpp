#include "MeshSplitter.hpp"
#include <libslic3r/MeshBoolean.hpp>

namespace Slic3r {
namespace FiveAxis {

std::vector<Region> MeshSplitter::split(
    const TriangleMesh& main_mesh,
    const std::vector<ModifierVolume>& modifiers) const
{
    std::vector<Region> out;

    if (modifiers.empty()) {
        Region r;
        r.id = "background";
        r.mesh = main_mesh;
        out.push_back(std::move(r));
        return out;
    }

    // Union of all modifier meshes for the background subtraction
    TriangleMesh modifier_union = modifiers.front().mesh;
    for (size_t i = 1; i < modifiers.size(); ++i) {
        MeshBoolean::cgal::plus(modifier_union, modifiers[i].mesh);
    }

    {
        TriangleMesh bg = main_mesh;
        MeshBoolean::cgal::minus(bg, modifier_union);
        Region r;
        r.id = "background";
        r.mesh = std::move(bg);
        out.push_back(std::move(r));
    }

    for (const auto& m : modifiers) {
        TriangleMesh region_mesh = main_mesh;
        MeshBoolean::cgal::intersect(region_mesh, m.mesh);
        Region r;
        r.id = "modifier_" + m.id;
        r.source_modifier_id = m.id;
        r.mesh = std::move(region_mesh);
        r.build_direction.rotation = m.transform;
        out.push_back(std::move(r));
    }

    return out;
}

} // namespace FiveAxis
} // namespace Slic3r

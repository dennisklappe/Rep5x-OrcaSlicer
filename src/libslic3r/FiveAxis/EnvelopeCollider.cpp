#include "EnvelopeCollider.hpp"
#include <libslic3r/BoundingBox.hpp>
#include <algorithm>

namespace Slic3r {
namespace FiveAxis {

namespace {

bool cylinder_intersects_bbox(double cx, double cy, double zmin, double zmax, double r,
                              const BoundingBoxf3& b)
{
    if (zmax < b.min.z() || zmin > b.max.z()) return false;
    const double dx = std::max({b.min.x() - cx, 0.0, cx - b.max.x()});
    const double dy = std::max({b.min.y() - cy, 0.0, cy - b.max.y()});
    return (dx * dx + dy * dy) <= r * r;
}

} // anonymous

bool EnvelopeCollider::collides(
    const ToolheadEnvelope& e,
    const Eigen::Vector3d& nozzle,
    const std::vector<Region>& prior) const
{
    const double cz_lo = nozzle.z() + e.z_offset_mm - e.height_mm * 0.5;
    const double cz_hi = nozzle.z() + e.z_offset_mm + e.height_mm * 0.5;
    for (const auto& r : prior) {
        if (cylinder_intersects_bbox(nozzle.x(), nozzle.y(), cz_lo, cz_hi, e.radius_mm,
                                     r.mesh.bounding_box()))
            return true;
    }
    return false;
}

std::vector<CollisionEvent> EnvelopeCollider::sweep(
    const ToolheadEnvelope& e,
    const std::string& current,
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Region>& prior) const
{
    std::vector<CollisionEvent> events;
    for (const auto& pos : positions) {
        const double cz_lo = pos.z() + e.z_offset_mm - e.height_mm * 0.5;
        const double cz_hi = pos.z() + e.z_offset_mm + e.height_mm * 0.5;
        for (const auto& r : prior) {
            if (cylinder_intersects_bbox(pos.x(), pos.y(), cz_lo, cz_hi, e.radius_mm,
                                         r.mesh.bounding_box())) {
                events.push_back({current, r.id, pos});
            }
        }
    }
    return events;
}

} // namespace FiveAxis
} // namespace Slic3r

#pragma once

#include "RegionTypes.hpp"
#include <libslic3r/TriangleMesh.hpp>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace Slic3r {
namespace FiveAxis {

struct ToolheadEnvelope
{
    double radius_mm = 25.0;
    double height_mm = 60.0;
    // Offset from nozzle tip to the centre of the head cylinder, +Z = upward.
    double z_offset_mm = 15.0;
};

struct CollisionEvent
{
    std::string region_being_printed;
    std::string region_collided_with;
    Eigen::Vector3d position_mm;
};

class EnvelopeCollider
{
public:
    // True if the toolhead cylinder centred on (nozzle_pos + (0,0,z_offset))
    // intersects the AABB of any prior region.
    bool collides(
        const ToolheadEnvelope& envelope,
        const Eigen::Vector3d& nozzle_pos_mm,
        const std::vector<Region>& previously_printed_regions) const;

    // Sweeps a series of nozzle positions, returning every collision event.
    std::vector<CollisionEvent> sweep(
        const ToolheadEnvelope& envelope,
        const std::string& current_region_id,
        const std::vector<Eigen::Vector3d>& nozzle_positions_mm,
        const std::vector<Region>& previously_printed_regions) const;
};

} // namespace FiveAxis
} // namespace Slic3r

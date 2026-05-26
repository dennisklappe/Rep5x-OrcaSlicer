#pragma once

#include "RegionTypes.hpp"
#include <vector>
#include <string>

namespace Slic3r {
namespace FiveAxis {

class RegionGraph
{
public:
    // Any region whose bbox.min.z <= this counts as touching the bed.
    static constexpr double BED_TOUCH_EPSILON_MM = 0.05;

    // Fills in `region.order` in-place. Default order: bed-touching regions first
    // (stable by input order), then BFS outward by bbox adjacency. Disconnected
    // regions get appended at the end.
    void compute_default_order(std::vector<Region>& regions) const;

    // Validates a user-specified order. Returns human-readable warnings for any
    // region that has no earlier-ordered neighbour to print onto. Empty = OK.
    std::vector<std::string> validate_order(const std::vector<Region>& regions) const;
};

} // namespace FiveAxis
} // namespace Slic3r

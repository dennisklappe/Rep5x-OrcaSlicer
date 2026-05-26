#include "RegionGraph.hpp"
#include <libslic3r/BoundingBox.hpp>
#include <queue>
#include <set>

namespace Slic3r {
namespace FiveAxis {

namespace {

bool bboxes_touch(const BoundingBoxf3& a, const BoundingBoxf3& b, double eps = 0.5)
{
    return  a.min.x() - eps <= b.max.x() && b.min.x() - eps <= a.max.x()
         && a.min.y() - eps <= b.max.y() && b.min.y() - eps <= a.max.y()
         && a.min.z() - eps <= b.max.z() && b.min.z() - eps <= a.max.z();
}

} // anonymous

void RegionGraph::compute_default_order(std::vector<Region>& regions) const
{
    if (regions.empty()) return;

    std::vector<BoundingBoxf3> bboxes;
    bboxes.reserve(regions.size());
    for (const auto& r : regions) bboxes.push_back(r.mesh.bounding_box());

    std::queue<size_t> queue;
    std::set<size_t> visited;
    for (size_t i = 0; i < regions.size(); ++i) {
        if (bboxes[i].min.z() <= BED_TOUCH_EPSILON_MM) {
            queue.push(i);
            visited.insert(i);
        }
    }
    if (queue.empty()) {
        queue.push(0);
        visited.insert(0);
    }

    int order_counter = 0;
    while (!queue.empty()) {
        const size_t cur = queue.front();
        queue.pop();
        regions[cur].order = order_counter++;
        for (size_t j = 0; j < regions.size(); ++j) {
            if (visited.count(j)) continue;
            if (bboxes_touch(bboxes[cur], bboxes[j])) {
                queue.push(j);
                visited.insert(j);
            }
        }
    }
    for (size_t i = 0; i < regions.size(); ++i) {
        if (regions[i].order < 0) regions[i].order = order_counter++;
    }
}

std::vector<std::string> RegionGraph::validate_order(const std::vector<Region>& regions) const
{
    std::vector<std::string> warnings;
    if (regions.empty()) return warnings;

    std::vector<BoundingBoxf3> bboxes;
    bboxes.reserve(regions.size());
    for (const auto& r : regions) bboxes.push_back(r.mesh.bounding_box());

    for (size_t i = 0; i < regions.size(); ++i) {
        if (bboxes[i].min.z() <= BED_TOUCH_EPSILON_MM) continue;
        bool has_earlier_neighbour = false;
        for (size_t j = 0; j < regions.size(); ++j) {
            if (i == j) continue;
            if (bboxes_touch(bboxes[i], bboxes[j]) && regions[j].order < regions[i].order) {
                has_earlier_neighbour = true;
                break;
            }
        }
        if (!has_earlier_neighbour) {
            warnings.push_back("Region '" + regions[i].id + "' floats — no earlier-ordered neighbour to print onto.");
        }
    }
    return warnings;
}

} // namespace FiveAxis
} // namespace Slic3r

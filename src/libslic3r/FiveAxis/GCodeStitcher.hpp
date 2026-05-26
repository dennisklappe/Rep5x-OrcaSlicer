#pragma once

#include "RegionTypes.hpp"
#include <libslic3r/BoundingBox.hpp>
#include <string>
#include <vector>

namespace Slic3r {
namespace FiveAxis {

struct StitchRegion
{
    std::string id;
    std::string gcode;                  // per-region G-code (output of Orca's planar slicer)
    BuildDirection build_direction;
    BoundingBoxf3 world_bbox;           // pre-transform world AABB — used for safe-height computation
};

struct StitchInput
{
    std::vector<StitchRegion> regions;  // in print order
    double tool_length_LB_mm = 0.0;
    double tool_length_LC_mm = 0.0;
    double transition_speed_mm_per_min = 600.0;

    // Bed-corner park position used during the orientation change between regions.
    // The head moves here BEFORE rotating B/C so the arm-swing happens over empty bed.
    double park_x_mm = 5.0;
    double park_y_mm = 5.0;

    // Vertical clearance above the tallest printed geometry, including the toolhead
    // arm length, before any XY motion or B/C rotation happens during a transition.
    double safety_margin_mm = 10.0;
};

class GCodeStitcher
{
public:
    // Combines per-region G-code into one Rep5x-flavored stream:
    //   - G43.4 LB<…> LC<…> once at top
    //   - Each motion line suffixed with B<deg> C<deg> for that region
    //   - Between regions: retract → straight-up lift to dynamic safe Z (above all printed
    //     geometry + tool arm + margin) → XY park at bed corner → B/C rotation in free
    //     space → descend → deretract → ; INTERFACE LAYER marker
    std::string stitch(const StitchInput& input) const;
};

} // namespace FiveAxis
} // namespace Slic3r

#pragma once

#include <string>

namespace Slic3r {
namespace FiveAxis {

class Rep5xDemo
{
public:
    // Builds a hardcoded L-shape mesh + one modifier covering the horizontal arm rotated
    // 90° about Y, runs the full FiveAxis pipeline (split → order → stub planar slice per
    // region → stitch), performs an envelope-collision check, and writes the result to
    // <out_path>. Returns 0 on success, non-zero on pipeline error.
    //
    // NOTE: the per-region planar slice is STUBBED — it emits a rectilinear-bbox path per
    // layer rather than calling Orca's full Print pipeline. Plan 4 swaps in the real slicer.
    static int run_l_shape(const std::string& out_path);
};

} // namespace FiveAxis
} // namespace Slic3r

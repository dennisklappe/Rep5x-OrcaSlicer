#include "RegionTypes.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r {
namespace FiveAxis {

// Rep5x firmware convention:
//   B = signed pitch off vertical, B=0 means nozzle pointing straight down (normal printing).
//   C = signed yaw (rotation about world +Z). Continuous in Rep5x firmware.
//
// We express the build direction (local +Z of the modifier in world coords) as:
//   B in [-90, +90], C in [-90, +90]. The "other half" of azimuth is absorbed into B's sign.
//   This keeps both axes inside the Rep5x firmware's reachable range and matches the
//   "B=0 means nozzle down" zero convention.
double BuildDirection::B_degrees() const
{
    const Eigen::Vector3d local_z = rotation.col(2);
    const double cos_b = std::clamp(local_z.z(), -1.0, 1.0);
    double b = std::acos(cos_b) * 180.0 / M_PI;     // 0..180 (unsigned)
    // Negate B when the original azimuth would be > ±90° — keeps signed B in [-90, +90].
    const double c_raw = std::atan2(local_z.y(), local_z.x()) * 180.0 / M_PI;
    if (std::abs(c_raw) > 90.0) b = -b;
    return b;
}

double BuildDirection::C_degrees() const
{
    const Eigen::Vector3d local_z = rotation.col(2);
    double c = std::atan2(local_z.y(), local_z.x()) * 180.0 / M_PI;   // -180..180
    if (c >  90.0) c -= 180.0;
    if (c < -90.0) c += 180.0;
    return c;
}

} // namespace FiveAxis
} // namespace Slic3r

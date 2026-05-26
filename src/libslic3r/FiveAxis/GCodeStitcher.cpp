#include "GCodeStitcher.hpp"
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace Slic3r {
namespace FiveAxis {

namespace {

std::string fmt_deg(double d)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << d;
    std::string s = oss.str();
    while (s.size() > 1 && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    return s;
}

std::string suffix_bc(const std::string& line, double B, double C)
{
    return line + " B" + fmt_deg(B) + " C" + fmt_deg(C);
}

bool is_motion_line(const std::string& line)
{
    return line.rfind("G0 ", 0) == 0 || line.rfind("G1 ", 0) == 0
        || line.rfind("G0\t", 0) == 0 || line.rfind("G1\t", 0) == 0;
}

bool already_has_bc(const std::string& line)
{
    return line.find(" B") != std::string::npos && line.find(" C") != std::string::npos;
}

} // anonymous

std::string GCodeStitcher::stitch(const StitchInput& input) const
{
    std::ostringstream out;
    out << "; OrcaSlicer-Rep5x — 5-axis G-code\n";
    out << "; tool length LB=" << input.tool_length_LB_mm
        << " LC=" << input.tool_length_LC_mm << "\n";
    out << "G43.4 LB" << input.tool_length_LB_mm
        << " LC" << input.tool_length_LC_mm
        << " ; enable TCP inverse kinematics\n";

    // Running max-Z over already-printed regions — drives the dynamic safe-Z.
    double printed_max_z_mm = 0.0;

    for (size_t i = 0; i < input.regions.size(); ++i) {
        const StitchRegion& region = input.regions[i];
        const double B = region.build_direction.B_degrees();
        const double C = region.build_direction.C_degrees();

        if (i > 0) {
            // Dynamic safe Z above EVERYTHING printed so far, plus the toolhead arm
            // length (so the arm can swing freely during the B/C rotation), plus margin.
            const double safe_z = printed_max_z_mm
                                + input.tool_length_LB_mm
                                + input.safety_margin_mm;

            out << "G10 ; retract before transition\n";
            out << "G0 Z" << fmt_deg(safe_z)
                << " F" << input.transition_speed_mm_per_min
                << " ; lift straight up above all printed material\n";
            out << "G0 X" << fmt_deg(input.park_x_mm)
                << " Y" << fmt_deg(input.park_y_mm)
                << " ; park at bed corner — clear of the print\n";
            out << "G0 B" << fmt_deg(B) << " C" << fmt_deg(C)
                << " ; rotate to new orientation in free space\n";
            out << "G11 ; deretract / prime\n";
            out << "; INTERFACE LAYER — section '"
                << input.regions[i-1].id << "' -> '" << region.id << "'\n";
        }

        out << "; --- region: " << region.id
            << " (B=" << fmt_deg(B) << " C=" << fmt_deg(C) << ") ---\n";
        std::istringstream lines(region.gcode);
        std::string line;
        while (std::getline(lines, line)) {
            if (is_motion_line(line) && !already_has_bc(line)) {
                out << suffix_bc(line, B, C) << "\n";
            } else {
                out << line << "\n";
            }
        }

        // Update printed-bbox tracker so the NEXT transition lifts above this region too.
        printed_max_z_mm = std::max(printed_max_z_mm, region.world_bbox.max.z());
    }
    return out.str();
}

} // namespace FiveAxis
} // namespace Slic3r

#include "Rep5xSlicerInvoker.hpp"
#include <libslic3r/Format/STL.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace Slic3r {
namespace FiveAxis {

namespace {

std::string shell_escape(const std::string& s)
{
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

std::string read_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("rep5x slicer: cannot read " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Force `layer_change_gcode` to contain `G92 E0` in a temporary copy of the user's machine
// profile, so the subprocess's validator doesn't reject relative-E profiles whose layer-reset
// gcode is inherited from a parent that --load-settings doesn't resolve.
//
// Returns the path to the prepared temp file. Source must be a JSON profile.
std::string ensure_layer_gcode_g92(const std::string& src_path, const std::string& scratch_dir)
{
    std::filesystem::create_directories(scratch_dir);
    const std::string dst_path = scratch_dir + "/active_machine.json";
    const std::string src = read_file(src_path);

    // If `layer_change_gcode` already contains G92 E0, we still write a copy (to keep all CLI
    // invocations on the same temp path regardless of source).
    const auto needle = src.find("\"layer_change_gcode\"");
    std::string out;
    if (needle == std::string::npos) {
        // No layer_change_gcode field — inject one right before the closing brace.
        const auto last_brace = src.rfind('}');
        if (last_brace == std::string::npos) {
            out = src;   // give up and pass through
        } else {
            std::string prefix = src.substr(0, last_brace);
            // Strip trailing whitespace
            while (!prefix.empty() && std::isspace((unsigned char)prefix.back())) prefix.pop_back();
            // Append a comma if the last non-whitespace char isn't already a comma or {
            if (!prefix.empty() && prefix.back() != '{' && prefix.back() != ',') prefix.push_back(',');
            out = prefix
                + "\n    \"layer_change_gcode\": \";LAYER_CHANGE\\n;[layer_z]\\nG92 E0\",\n"
                + "    \"before_layer_change_gcode\": \"\"\n"
                + "}";
        }
    } else {
        // Replace the existing value (whatever it is) with one that contains G92 E0.
        // Find the start of the value (after `"layer_change_gcode"` + `:` + first `"`)
        const auto colon = src.find(':', needle);
        if (colon == std::string::npos) { out = src; }
        else {
            const auto val_start = src.find('"', colon);
            if (val_start == std::string::npos) { out = src; }
            else {
                // Walk to the matching closing quote, respecting \" escapes
                size_t val_end = val_start + 1;
                while (val_end < src.size() && src[val_end] != '"') {
                    if (src[val_end] == '\\' && val_end + 1 < src.size()) val_end += 2;
                    else ++val_end;
                }
                if (val_end >= src.size()) { out = src; }
                else {
                    out = src.substr(0, val_start)
                        + "\";LAYER_CHANGE\\n;[layer_z]\\nG92 E0\""
                        + src.substr(val_end + 1);
                }
            }
        }
    }

    std::ofstream f(dst_path);
    f << out;
    f.close();
    return dst_path;
}

// One X/Y/Z token found in a G-code line: "X12.345" → axis='X', value=12.345, [start..end).
struct AxisToken {
    char   axis;
    size_t start;
    size_t end;
    double value;
};

std::vector<AxisToken> find_axis_tokens(const std::string& line)
{
    std::vector<AxisToken> out;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c != 'X' && c != 'Y' && c != 'Z') continue;
        // Must be preceded by start-of-line or whitespace, otherwise we'd grab the X in e.g. "MAXVAL"
        if (i > 0 && !std::isspace((unsigned char)line[i - 1])) continue;
        size_t j = i + 1;
        if (j >= line.size()) continue;
        if (line[j] == '-' || line[j] == '+') ++j;
        const size_t num_start = (line[i + 1] == '-' || line[i + 1] == '+') ? i + 1 : i + 1;
        bool has_digit = false;
        while (j < line.size() && (std::isdigit((unsigned char)line[j]) || line[j] == '.')) {
            if (std::isdigit((unsigned char)line[j])) has_digit = true;
            ++j;
        }
        if (!has_digit) continue;
        AxisToken t;
        t.axis  = c;
        t.start = i;
        t.end   = j;
        t.value = std::strtod(line.c_str() + num_start, nullptr);
        out.push_back(t);
    }
    return out;
}

bool starts_with_cmd(const std::string& line, const char* cmd)
{
    const size_t n = std::strlen(cmd);
    if (line.size() < n) return false;
    if (std::memcmp(line.data(), cmd, n) != 0) return false;
    if (line.size() == n) return true;
    const char next = line[n];
    return next == ' ' || next == '\t';
}

// Transforms each G0/G1 motion line: emits ALL THREE of X/Y/Z in world coordinates,
// computed by applying R to (current_local_pos + local_translation_offset).
//
// "local_translation_offset" lets us shift the slicer-frame back to where the rotated
// sub-mesh originally sat in local frame before we translated it to bed-origin.
//
// Non-motion lines are emitted verbatim. G92 updates the position state but does NOT
// move the tool (so we emit it verbatim too).
std::string transform_gcode_coords(const std::string& in,
                                   const Eigen::Matrix3d& R,
                                   const Eigen::Vector3d& local_translation_offset)
{
    const bool is_identity_R = R.isApprox(Eigen::Matrix3d::Identity());
    const bool is_zero_off = local_translation_offset.isZero(1e-9);
    if (is_identity_R && is_zero_off) return in;

    std::ostringstream out;
    double lx = 0, ly = 0, lz = 0;
    std::istringstream lines(in);
    std::string line;

    while (std::getline(lines, line)) {
        const bool is_motion = starts_with_cmd(line, "G0") || starts_with_cmd(line, "G1");
        const bool is_g92    = starts_with_cmd(line, "G92");

        if (!is_motion && !is_g92) { out << line << '\n'; continue; }

        std::vector<AxisToken> tokens = find_axis_tokens(line);
        // Update local state with any present axes
        for (const auto& t : tokens) {
            if      (t.axis == 'X') lx = t.value;
            else if (t.axis == 'Y') ly = t.value;
            else if (t.axis == 'Z') lz = t.value;
        }

        if (is_g92 || tokens.empty()) {
            out << line << '\n';
            continue;
        }

        // World position = R * (slicer_pos + local_translation_offset)
        const Eigen::Vector3d slicer_pos(lx, ly, lz);
        const Eigen::Vector3d world = R * (slicer_pos + local_translation_offset);

        // Rebuild the line:
        //   - strip out existing X/Y/Z tokens (and one trailing space each)
        //   - inject " X<wx> Y<wy> Z<wz>" right after the G command word
        std::sort(tokens.begin(), tokens.end(),
                  [](const AxisToken& a, const AxisToken& b) { return a.start < b.start; });

        std::string stripped;
        size_t prev = 0;
        for (const auto& t : tokens) {
            stripped.append(line, prev, t.start - prev);
            prev = t.end;
            // Gobble one trailing space if any (avoid double-spaces in the rebuilt line)
            if (prev < line.size() && line[prev] == ' ') ++prev;
        }
        stripped.append(line, prev, std::string::npos);

        // Find end of the G command word (G0 / G1)
        size_t cmd_end = 0;
        while (cmd_end < stripped.size() && !std::isspace((unsigned char)stripped[cmd_end])) ++cmd_end;

        std::ostringstream inj;
        inj << " X" << std::fixed << std::setprecision(3) << world.x()
            << " Y" << std::fixed << std::setprecision(3) << world.y()
            << " Z" << std::fixed << std::setprecision(3) << world.z();

        out << stripped.substr(0, cmd_end) << inj.str() << stripped.substr(cmd_end) << '\n';
    }
    return out.str();
}

} // anonymous

std::string Rep5xSlicerInvoker::slice_region(const Region& region, double bonding_offset_mm) const
{
    std::filesystem::create_directories(m_cfg.scratch_dir);
    const std::string stl_path = m_cfg.scratch_dir + "/" + region.id + ".stl";
    const std::string out_dir  = m_cfg.scratch_dir + "/" + region.id + "_out";
    std::filesystem::create_directories(out_dir);
    const std::string gcode_path = out_dir + "/plate_1.gcode";

    // --- Build-direction transform pipeline ---
    // Goal: present the slicer with a mesh whose build direction (modifier's +Z) faces world +Z,
    // sitting on the slicer's notional bed (Z >= 0). After slicing, transform every motion coord
    // back into world space so the firmware (with TCP-comp) puts the nozzle tip where we mean.
    //
    // 1. Rotate the world-coord sub-mesh by R^T so the modifier's build direction faces +Z locally.
    // 2. Translate the rotated mesh so its bbox.min is at (0,0,0) — slicer-friendly.
    // 3. After slicing, transform each motion coord by:  world = R * (slicer + local_translation_offset)
    //    where local_translation_offset = -translate_to_origin = bbox_min_local.
    const Eigen::Matrix3d R       = region.build_direction.rotation;
    const Eigen::Matrix3d inv_R   = R.transpose();
    const bool is_identity        = R.isApprox(Eigen::Matrix3d::Identity());

    // Padding inside the slicer-frame so the outer-wall extrusion (which extends slightly
    // beyond the bbox) doesn't trip Orca's "unprintable area" check. 10mm is comfortable.
    constexpr double SLICER_FRAME_PAD_MM = 10.0;

    TriangleMesh slicer_mesh = region.mesh;
    Eigen::Vector3d local_translation_offset = Eigen::Vector3d::Zero();
    if (!is_identity) {
        slicer_mesh.transform(inv_R);
        const BoundingBoxf3 bb_local = slicer_mesh.bounding_box();
        const double tx = SLICER_FRAME_PAD_MM - bb_local.min.x();
        const double ty = SLICER_FRAME_PAD_MM - bb_local.min.y();
        const double tz =                       -bb_local.min.z();
        slicer_mesh.translate(float(tx), float(ty), float(tz));
        // slicer_pos + local_translation_offset = pre-translate-rotated pos
        //   = (slicer - (tx,ty,tz)) = slicer + (bb_local.min - (PAD,PAD,0))
        local_translation_offset = Eigen::Vector3d(bb_local.min.x() - SLICER_FRAME_PAD_MM,
                                                   bb_local.min.y() - SLICER_FRAME_PAD_MM,
                                                   bb_local.min.z());
    }

    if (!Slic3r::store_stl(stl_path.c_str(), &slicer_mesh, /*binary=*/true))
        throw std::runtime_error("rep5x slicer: failed to write STL " + stl_path);

    // Patch the machine profile: ensure layer_change_gcode contains G92 E0 so the slicer's
    // relative-E validator passes regardless of the user's selected profile.
    const std::string patched_machine = ensure_layer_gcode_g92(m_cfg.machine_profile_path, m_cfg.scratch_dir);

    // --arrange 0 + --orient 0: keep the mesh exactly where we put it.
    // Otherwise Orca centers the model on the bed, breaking our build-direction
    // transform math (which depends on the slicer respecting input mesh coordinates).
    std::ostringstream cmd;
    cmd << shell_escape(m_cfg.orca_binary_path)
        << " --slice 0"
        << " --arrange 0"
        << " --orient 0"
        << " --load-settings " << shell_escape(patched_machine + ";" + m_cfg.process_profile_path)
        << " --load-filaments " << shell_escape(m_cfg.filament_profile_path)
        << " --outputdir "      << shell_escape(out_dir)
        << " "                  << shell_escape(stl_path)
        << " > "                << shell_escape(out_dir + "/slicer.log")
        << " 2>&1";

    const int rc = std::system(cmd.str().c_str());
    if (rc != 0)
        throw std::runtime_error("rep5x slicer: subprocess exit=" + std::to_string(rc)
                                 + " for region '" + region.id + "', see " + out_dir + "/slicer.log");

    if (!std::filesystem::exists(gcode_path))
        throw std::runtime_error("rep5x slicer: expected G-code missing at " + gcode_path);

    std::string gcode = read_file(gcode_path);

    if (!is_identity) {
        // Apply bonding offset: shift the world output by -bonding along build direction.
        // build_dir_world = R.col(2). World shift = -bonding * build_dir_world.
        // Since world = R * (slicer + offset), the equivalent local-frame shift is:
        // R^T * world_shift = R^T * (-bonding * R.col(2)) = -bonding * (R^T * R.col(2))
        //                   = -bonding * (0, 0, 1) (R^T*R = I, col(2) maps to identity's col 2)
        // So add (0, 0, -bonding) to local_translation_offset.
        Eigen::Vector3d offset = local_translation_offset;
        if (bonding_offset_mm > 0.0)
            offset.z() -= bonding_offset_mm;
        gcode = transform_gcode_coords(gcode, R, offset);
    } else if (bonding_offset_mm > 0.0) {
        // Identity rotation — bonding just shifts world Z down by `bonding`.
        // Use the existing transform helper with rotation=Identity and z-offset.
        Eigen::Vector3d offset(0.0, 0.0, -bonding_offset_mm);
        gcode = transform_gcode_coords(gcode, Eigen::Matrix3d::Identity(), offset);
    }

    return gcode;
}

} // namespace FiveAxis
} // namespace Slic3r

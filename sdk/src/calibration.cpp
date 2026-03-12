// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — CalibrationBundle YAML I/O
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/calibration.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <charconv>

// ─────────────────────────────────────────────────────────────────────────────
// Minimal YAML parser / emitter — dependency-free
// ─────────────────────────────────────────────────────────────────────────────
//
// The calibration YAML schema is flat enough that a full YAML library
// (yaml-cpp etc.) is not required.  This keeps the SDK dependency-free
// and avoids ABI issues with yaml-cpp across distros.
//
// Supported subset:
//   • Scalar values (double, int64, bool, string)
//   • Inline arrays [a, b, c]
//   • Nested maps with indentation (2-space)
//   • Sequence items with "- "
//   • Comments beginning with #
//
// NOT supported:  multi-line strings, anchors/aliases, flow maps.
// ─────────────────────────────────────────────────────────────────────────────

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

namespace {

// ── Helpers ─────────────────────────────────────────────────────────────────

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return {};
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Parse an inline array like "[1.0, 0.0, 0.0, 0.0]" into doubles.
std::vector<double> parse_double_array(const std::string& value) {
    std::vector<double> result;
    std::string inner = value;
    // Strip brackets
    auto b = inner.find('[');
    auto e = inner.rfind(']');
    if (b != std::string::npos && e != std::string::npos)
        inner = inner.substr(b + 1, e - b - 1);

    std::istringstream ss(inner);
    std::string token;
    while (std::getline(ss, token, ',')) {
        std::string t = trim(token);
        if (!t.empty()) {
            try {
                result.push_back(std::stod(t));
            } catch (...) {}
        }
    }
    return result;
}

double parse_double(const std::string& s, double def = 0.0) {
    std::string t = trim(s);
    if (t.empty()) return def;
    try { return std::stod(t); }
    catch (...) { return def; }
}

int64_t parse_int64(const std::string& s, int64_t def = 0) {
    std::string t = trim(s);
    if (t.empty()) return def;
    try { return std::stoll(t); }
    catch (...) { return def; }
}

uint32_t parse_uint32(const std::string& s, uint32_t def = 0) {
    std::string t = trim(s);
    if (t.empty()) return def;
    try { return static_cast<uint32_t>(std::stoul(t)); }
    catch (...) { return def; }
}

bool parse_bool(const std::string& s, bool def = false) {
    std::string t = trim(s);
    if (t == "true" || t == "True" || t == "1") return true;
    if (t == "false" || t == "False" || t == "0") return false;
    return def;
}

DistortionModel parse_distortion_model(const std::string& s) {
    std::string t = trim(s);
    // Strip matching quotes at both ends (either "..." or '...')
    if (t.size() >= 2 &&
        ((t.front() == '"' && t.back() == '"') ||
         (t.front() == '\'' && t.back() == '\''))) {
        t = t.substr(1, t.size() - 2);
    }
    if (t == "radtan" || t == "radial_tangential" || t == "RadialTangential")
        return DistortionModel::RadialTangential;
    if (t == "equidistant" || t == "Equidistant")
        return DistortionModel::Equidistant;
    if (t == "fov" || t == "FieldOfView")
        return DistortionModel::FieldOfView;
    return DistortionModel::None;
}

// Simple key-value map per indentation level
struct YamlLine {
    int    indent;
    bool   is_seq_item;   // starts with "- "
    std::string key;
    std::string value;
};

std::vector<YamlLine> tokenize_yaml(std::istream& in) {
    std::vector<YamlLine> lines;
    std::string raw;
    while (std::getline(in, raw)) {
        // Strip comment
        auto hash = raw.find('#');
        if (hash != std::string::npos) raw = raw.substr(0, hash);

        // Measure indent
        int indent = 0;
        while (indent < static_cast<int>(raw.size()) && raw[indent] == ' ')
            ++indent;

        std::string content = trim(raw);
        if (content.empty()) continue;

        bool is_seq = false;
        if (content.size() >= 2 && content[0] == '-' && content[1] == ' ') {
            is_seq = true;
            content = content.substr(2);
            content = trim(content);
        }

        auto colon = content.find(':');
        if (colon != std::string::npos) {
            std::string key = trim(content.substr(0, colon));
            std::string val = trim(content.substr(colon + 1));
            lines.push_back({indent, is_seq, key, val});
        } else {
            lines.push_back({indent, is_seq, content, {}});
        }
    }
    return lines;
}

// ── Emitter helpers ─────────────────────────────────────────────────────────

std::string indent(int level) {
    return std::string(static_cast<size_t>(level * 2), ' ');
}

void emit_extrinsic(std::ostream& out, int lvl,
                    const SensorExtrinsic& ext) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "[%.10g, %.10g, %.10g, %.10g]",
                  ext.rotation[0], ext.rotation[1],
                  ext.rotation[2], ext.rotation[3]);
    out << indent(lvl) << "rotation: " << buf << "\n";

    std::snprintf(buf, sizeof(buf), "[%.10g, %.10g, %.10g]",
                  ext.translation[0], ext.translation[1],
                  ext.translation[2]);
    out << indent(lvl) << "translation: " << buf << "\n";
}

void emit_intrinsics(std::ostream& out, int lvl,
                     const CameraIntrinsics& ci) {
    out << indent(lvl) << "fx: " << ci.fx << "\n";
    out << indent(lvl) << "fy: " << ci.fy << "\n";
    out << indent(lvl) << "cx: " << ci.cx << "\n";
    out << indent(lvl) << "cy: " << ci.cy << "\n";
    out << indent(lvl) << "width: " << ci.width << "\n";
    out << indent(lvl) << "height: " << ci.height << "\n";
    out << indent(lvl) << "distortion_model: "
        << distortion_model_name(ci.distortion_model) << "\n";

    out << indent(lvl) << "distortion_coeffs: [";
    int n = 0;
    switch (ci.distortion_model) {
        case DistortionModel::RadialTangential: n = 5; break;
        case DistortionModel::Equidistant:      n = 4; break;
        case DistortionModel::FieldOfView:      n = 1; break;
        case DistortionModel::None:             n = 0; break;
    }
    for (int i = 0; i < n; ++i) {
        if (i > 0) out << ", ";
        out << ci.distortion_coeffs[i];
    }
    out << "]\n";
}

} // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
//  CalibrationBundle::load_yaml
// ═════════════════════════════════════════════════════════════════════════════

bool CalibrationBundle::load_yaml(const std::string& path) {
    std::ifstream fin(path);
    if (!fin.is_open()) return false;

    auto lines = tokenize_yaml(fin);
    if (lines.empty()) return false;

    // Replace current bundle state on each load.
    imu_T_lidar = SensorExtrinsic{};
    refine_imu_T_lidar = false;
    imu_noise = ImuNoiseParams{};
    // Use zero baseline so "loaded something" checks reflect YAML content.
    imu_noise.gyro_noise = 0.0;
    imu_noise.accel_noise = 0.0;
    imu_noise.gyro_bias_rw = 0.0;
    imu_noise.accel_bias_rw = 0.0;
    cameras.clear();

    // State machine: track which section we're in
    enum class Section {
        Root, ImuTLidar, ImuNoise, Cameras, CameraItem,
        CameraIntrinsics, CameraExtrinsic
    };
    Section section = Section::Root;
    CameraCalibration current_cam;
    bool in_camera = false;

    for (size_t i = 0; i < lines.size(); ++i) {
        const auto& l = lines[i];

        // Root-level keys
        if (l.indent == 0 && !l.is_seq_item) {
            // Flush any in-progress camera
            if (in_camera) {
                cameras.push_back(std::move(current_cam));
                current_cam = {};
                in_camera = false;
            }

            if (l.key == "imu_T_lidar") {
                section = Section::ImuTLidar;
            } else if (l.key == "refine_imu_T_lidar") {
                refine_imu_T_lidar = parse_bool(l.value);
                section = Section::Root;
            } else if (l.key == "imu_noise") {
                section = Section::ImuNoise;
            } else if (l.key == "cameras") {
                section = Section::Cameras;
            } else {
                section = Section::Root;
            }
            continue;
        }

        // imu_T_lidar sub-keys
        if (section == Section::ImuTLidar && l.indent >= 2) {
            if (l.key == "rotation") {
                auto r = parse_double_array(l.value);
                if (r.size() >= 4)
                    imu_T_lidar.rotation = {r[0], r[1], r[2], r[3]};
            } else if (l.key == "translation") {
                auto t = parse_double_array(l.value);
                if (t.size() >= 3)
                    imu_T_lidar.translation = {t[0], t[1], t[2]};
            }
            continue;
        }

        // imu_noise sub-keys
        if (section == Section::ImuNoise && l.indent >= 2) {
            if (l.key == "gyro_noise")    imu_noise.gyro_noise = parse_double(l.value);
            else if (l.key == "accel_noise")   imu_noise.accel_noise = parse_double(l.value);
            else if (l.key == "gyro_bias_rw")  imu_noise.gyro_bias_rw = parse_double(l.value);
            else if (l.key == "accel_bias_rw") imu_noise.accel_bias_rw = parse_double(l.value);
            continue;
        }

        // cameras section — sequence items
        if (section == Section::Cameras || section == Section::CameraItem ||
            section == Section::CameraIntrinsics || section == Section::CameraExtrinsic) {

            // New camera item
            if (l.is_seq_item) {
                if (in_camera) {
                    cameras.push_back(std::move(current_cam));
                    current_cam = {};
                }
                in_camera = true;
                section = Section::CameraItem;

                // The seq item line may have a key on it
                if (l.key == "label") {
                    current_cam.label = trim(l.value);
                    // Strip matching quotes at both ends
                    if (current_cam.label.size() >= 2 &&
                        ((current_cam.label.front() == '"' && current_cam.label.back() == '"') ||
                         (current_cam.label.front() == '\'' && current_cam.label.back() == '\'')))
                        current_cam.label = current_cam.label.substr(1, current_cam.label.size() - 2);
                }
                continue;
            }

            // Camera sub-keys
            if (in_camera && l.indent >= 4) {
                if (l.key == "label") {
                    current_cam.label = trim(l.value);
                    if (current_cam.label.size() >= 2 &&
                        ((current_cam.label.front() == '"' && current_cam.label.back() == '"') ||
                         (current_cam.label.front() == '\'' && current_cam.label.back() == '\'')))
                        current_cam.label = current_cam.label.substr(1, current_cam.label.size() - 2);
                } else if (l.key == "time_offset_ns") {
                    current_cam.time_offset_ns = parse_int64(l.value);
                } else if (l.key == "intrinsics") {
                    section = Section::CameraIntrinsics;
                } else if (l.key == "imu_T_camera") {
                    section = Section::CameraExtrinsic;
                }
            }

            // Camera intrinsics sub-keys
            if (section == Section::CameraIntrinsics && l.indent >= 6) {
                auto& ci = current_cam.intrinsics;
                if (l.key == "fx") ci.fx = parse_double(l.value);
                else if (l.key == "fy") ci.fy = parse_double(l.value);
                else if (l.key == "cx") ci.cx = parse_double(l.value);
                else if (l.key == "cy") ci.cy = parse_double(l.value);
                else if (l.key == "width")  ci.width = parse_uint32(l.value);
                else if (l.key == "height") ci.height = parse_uint32(l.value);
                else if (l.key == "distortion_model")
                    ci.distortion_model = parse_distortion_model(l.value);
                else if (l.key == "distortion_coeffs") {
                    auto d = parse_double_array(l.value);
                    for (size_t j = 0; j < d.size() && j < ci.distortion_coeffs.size(); ++j)
                        ci.distortion_coeffs[j] = d[j];
                }
                continue;
            }

            // Camera extrinsic sub-keys
            if (section == Section::CameraExtrinsic && l.indent >= 6) {
                auto& ext = current_cam.imu_T_camera;
                if (l.key == "rotation") {
                    auto r = parse_double_array(l.value);
                    if (r.size() >= 4)
                        ext.rotation = {r[0], r[1], r[2], r[3]};
                } else if (l.key == "translation") {
                    auto t = parse_double_array(l.value);
                    if (t.size() >= 3)
                        ext.translation = {t[0], t[1], t[2]};
                }
                continue;
            }

            // If indent drops back to camera item level, switch back
            if (l.indent <= 4 && !l.is_seq_item &&
                (section == Section::CameraIntrinsics || section == Section::CameraExtrinsic)) {
                section = Section::CameraItem;
                // Re-process this line
                if (l.key == "label") {
                    current_cam.label = trim(l.value);
                } else if (l.key == "time_offset_ns") {
                    current_cam.time_offset_ns = parse_int64(l.value);
                } else if (l.key == "intrinsics") {
                    section = Section::CameraIntrinsics;
                } else if (l.key == "imu_T_camera") {
                    section = Section::CameraExtrinsic;
                }
            }

            continue;
        }
    }

    // Flush last camera
    if (in_camera) {
        cameras.push_back(std::move(current_cam));
    }

    // Reject if nothing meaningful was loaded
    bool has_extrinsic = !(imu_T_lidar.rotation[0] == 1.0 &&
                           imu_T_lidar.rotation[1] == 0.0 &&
                           imu_T_lidar.rotation[2] == 0.0 &&
                           imu_T_lidar.rotation[3] == 0.0 &&
                           imu_T_lidar.translation[0] == 0.0 &&
                           imu_T_lidar.translation[1] == 0.0 &&
                           imu_T_lidar.translation[2] == 0.0);
    bool has_imu_noise = (imu_noise.gyro_noise != 0.0 ||
                          imu_noise.accel_noise != 0.0 ||
                          imu_noise.gyro_bias_rw != 0.0 ||
                          imu_noise.accel_bias_rw != 0.0);
    bool has_refine_flag = refine_imu_T_lidar;
    if (!has_extrinsic && !has_imu_noise && cameras.empty() && !has_refine_flag)
        return false;

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  CalibrationBundle::save_yaml
// ═════════════════════════════════════════════════════════════════════════════

bool CalibrationBundle::save_yaml(const std::string& path) const {
    std::ofstream fout(path);
    if (!fout.is_open()) return false;

    fout << "# Thunderbird SDK — Calibration Bundle\n";
    fout << "# Generated by CalibrationBundle::save_yaml()\n\n";

    // imu_T_lidar
    fout << "imu_T_lidar:\n";
    emit_extrinsic(fout, 1, imu_T_lidar);

    fout << "refine_imu_T_lidar: "
         << (refine_imu_T_lidar ? "true" : "false") << "\n\n";

    // imu_noise
    fout << "imu_noise:\n";
    fout << "  gyro_noise: "    << imu_noise.gyro_noise    << "\n";
    fout << "  accel_noise: "   << imu_noise.accel_noise   << "\n";
    fout << "  gyro_bias_rw: "  << imu_noise.gyro_bias_rw  << "\n";
    fout << "  accel_bias_rw: " << imu_noise.accel_bias_rw << "\n\n";

    // cameras
    if (!cameras.empty()) {
        fout << "cameras:\n";
        for (const auto& cam : cameras) {
            fout << "  - label: \"" << cam.label << "\"\n";
            fout << "    time_offset_ns: " << cam.time_offset_ns << "\n";
            fout << "    intrinsics:\n";
            emit_intrinsics(fout, 3, cam.intrinsics);
            fout << "    imu_T_camera:\n";
            emit_extrinsic(fout, 3, cam.imu_T_camera);
        }
    }

    fout.flush();
    return fout.good();
}

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird

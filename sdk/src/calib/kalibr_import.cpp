// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Kalibr Result Import Implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Parses Kalibr's output YAML files and maps to CalibrationBundle.
//
// Kalibr camchain-imucam YAML layout:
//
//   cam0:
//     camera_model: pinhole
//     distortion_coeffs: [k1, k2, p1, p2]
//     distortion_model: radtan
//     intrinsics: [fx, fy, cx, cy]
//     resolution: [w, h]
//     rostopic: /cam0/image_raw
//     T_cam_imu:
//     - [r00, r01, r02, tx]
//     - [r10, r11, r12, ty]
//     - [r20, r21, r22, tz]
//     - [0.0, 0.0, 0.0, 1.0]
//     timeshift_cam_imu: -0.00581...
//   cam1:
//     ...
//     T_cn_cnm1:
//     - [...]
//
// Kalibr imu YAML layout:
//
//   imu0:
//     accelerometer_noise_density: 0.01
//     accelerometer_random_walk: 0.0002
//     gyroscope_noise_density: 0.001
//     gyroscope_random_walk: 1.0e-05
//     model: calibrated
//     rostopic: /imu0
//     update_rate: 200.0
//
// ─────────────────────────────────────────────────────────────────────────────
#include "kalibr_import.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace thunderbird::calib {
namespace {

// ── String helpers ──────────────────────────────────────────────────────────

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::vector<double> parseInlineArray(const std::string& val) {
    std::vector<double> result;
    std::string inner = val;
    auto b = inner.find('[');
    auto e = inner.rfind(']');
    if (b != std::string::npos && e != std::string::npos)
        inner = inner.substr(b + 1, e - b - 1);
    std::istringstream ss(inner);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        std::string t = trim(tok);
        if (!t.empty()) {
            try { result.push_back(std::stod(t)); } catch (...) {}
        }
    }
    return result;
}

double parseDouble(const std::string& s, double def = 0.0) {
    std::string t = trim(s);
    if (t.empty()) return def;
    try { return std::stod(t); } catch (...) { return def; }
}

int indentLevel(const std::string& line) {
    int n = 0;
    for (char c : line) {
        if (c == ' ') ++n;
        else break;
    }
    return n;
}

std::string stripQuotes(const std::string& s) {
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\'')))
        return s.substr(1, s.size() - 2);
    return s;
}

// ── Rotation matrix → Hamilton quaternion [w,x,y,z] ────────────────────────

struct Quat { double w, x, y, z; };

Quat rotMatToQuat(const double R[9]) {
    // Shepperd's method for numerical stability
    double tr = R[0] + R[4] + R[8];
    Quat q;
    if (tr > 0.0) {
        double s = 0.5 / std::sqrt(tr + 1.0);
        q.w = 0.25 / s;
        q.x = (R[7] - R[5]) * s;  // R(2,1) - R(1,2)
        q.y = (R[2] - R[6]) * s;  // R(0,2) - R(2,0)
        q.z = (R[3] - R[1]) * s;  // R(1,0) - R(0,1)
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
        q.w = (R[7] - R[5]) / s;
        q.x = 0.25 * s;
        q.y = (R[1] + R[3]) / s;
        q.z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
        q.w = (R[2] - R[6]) / s;
        q.x = (R[1] + R[3]) / s;
        q.y = 0.25 * s;
        q.z = (R[5] + R[7]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
        q.w = (R[3] - R[1]) / s;
        q.x = (R[2] + R[6]) / s;
        q.y = (R[5] + R[7]) / s;
        q.z = 0.25 * s;
    }
    // Normalize
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n > 0.0) { q.w /= n; q.x /= n; q.y /= n; q.z /= n; }
    // Convention: positive w
    if (q.w < 0.0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    return q;
}

// ── Invert a 4×4 rigid transform ───────────────────────────────────────────
// T = [R t; 0 1]  →  T_inv = [R^T  -R^T t; 0 1]

void invertTransform(const double T[16], double T_inv[16]) {
    // R^T
    T_inv[0] = T[0]; T_inv[1] = T[4]; T_inv[2]  = T[8];
    T_inv[4] = T[1]; T_inv[5] = T[5]; T_inv[6]  = T[9];
    T_inv[8] = T[2]; T_inv[9] = T[6]; T_inv[10] = T[10];

    // -R^T * t
    T_inv[3]  = -(T_inv[0]*T[3] + T_inv[1]*T[7] + T_inv[2]*T[11]);
    T_inv[7]  = -(T_inv[4]*T[3] + T_inv[5]*T[7] + T_inv[6]*T[11]);
    T_inv[11] = -(T_inv[8]*T[3] + T_inv[9]*T[7] + T_inv[10]*T[11]);

    // Bottom row
    T_inv[12] = 0; T_inv[13] = 0; T_inv[14] = 0; T_inv[15] = 1;
}

SensorExtrinsic transformToExtrinsic(const double T[16]) {
    double R[9] = {T[0], T[1], T[2],
                   T[4], T[5], T[6],
                   T[8], T[9], T[10]};
    Quat q = rotMatToQuat(R);
    SensorExtrinsic ext;
    ext.rotation[0] = q.w; ext.rotation[1] = q.x;
    ext.rotation[2] = q.y; ext.rotation[3] = q.z;
    ext.translation[0] = T[3]; ext.translation[1] = T[7]; ext.translation[2] = T[11];
    return ext;
}

DistortionModel mapDistortionModel(const std::string& kalibr_model) {
    std::string m = trim(stripQuotes(kalibr_model));
    if (m == "radtan" || m == "plumb_bob") return DistortionModel::RadialTangential;
    if (m == "equidistant" || m == "equi") return DistortionModel::Equidistant;
    if (m == "fov") return DistortionModel::FieldOfView;
    if (m == "none") return DistortionModel::None;
    return DistortionModel::None;
}

// ── Kalibr camchain parser ──────────────────────────────────────────────────

struct KalibrCamera {
    std::string label;
    std::string distortion_model;
    std::vector<double> distortion_coeffs;
    std::vector<double> intrinsics;  // [fx, fy, cx, cy]
    std::vector<double> resolution;  // [w, h]
    double T_cam_imu[16] = {};       // 4×4 row-major
    bool   has_T_cam_imu = false;
    double timeshift_cam_imu = 0.0;
};

// Parse YAML lines for a Kalibr camchain file
std::vector<KalibrCamera> parseCamchain(const std::vector<std::string>& lines) {
    std::vector<KalibrCamera> cams;

    int current_cam = -1;
    bool in_T_cam_imu = false;
    int T_row = 0;

    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string& raw = lines[i];
        if (raw.empty()) continue;

        int indent = indentLevel(raw);
        std::string stripped = trim(raw);
        if (stripped.empty() || stripped[0] == '#') continue;

        // Top-level cam section: "cam0:", "cam1:", etc.
        if (indent == 0 && stripped.size() >= 4 &&
            stripped.substr(0, 3) == "cam" && stripped.back() == ':') {
            in_T_cam_imu = false;
            try {
                current_cam = std::stoi(stripped.substr(3, stripped.size() - 4));
            } catch (...) { continue; }
            // Ensure vector is large enough
            while (static_cast<int>(cams.size()) <= current_cam)
                cams.emplace_back();
            cams[static_cast<size_t>(current_cam)].label = stripped.substr(0, stripped.size() - 1);
            continue;
        }

        if (current_cam < 0) continue;
        auto& cam = cams[static_cast<size_t>(current_cam)];

        // Check for T_cam_imu matrix rows (indented "- [...]" lines)
        if (in_T_cam_imu) {
            if (indent >= 2 && stripped[0] == '-') {
                auto vals = parseInlineArray(stripped.substr(1));
                if (vals.size() == 4 && T_row < 4) {
                    for (int c = 0; c < 4; ++c)
                        cam.T_cam_imu[T_row * 4 + c] = vals[static_cast<size_t>(c)];
                    T_row++;
                    if (T_row == 4) {
                        cam.has_T_cam_imu = true;
                        in_T_cam_imu = false;
                    }
                }
                continue;
            } else {
                in_T_cam_imu = false;
                // Fall through to parse current line as a key
            }
        }

        // Key: value pairs
        auto colon = stripped.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(stripped.substr(0, colon));
        std::string val = trim(stripped.substr(colon + 1));

        if (key == "distortion_model") {
            cam.distortion_model = stripQuotes(val);
        } else if (key == "distortion_coeffs") {
            cam.distortion_coeffs = parseInlineArray(val);
        } else if (key == "intrinsics") {
            cam.intrinsics = parseInlineArray(val);
        } else if (key == "resolution") {
            cam.resolution = parseInlineArray(val);
        } else if (key == "timeshift_cam_imu") {
            cam.timeshift_cam_imu = parseDouble(val);
        } else if (key == "T_cam_imu" || key == "T_cn_cnm1") {
            if (key == "T_cam_imu") {
                in_T_cam_imu = true;
                T_row = 0;
            }
        }
    }

    return cams;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

bool importKalibrCamchain(const std::string& path, CalibrationBundle& bundle) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line))
        lines.push_back(line);

    auto cams = parseCamchain(lines);
    if (cams.empty()) return false;

    bundle.cameras.clear();
    bundle.cameras.reserve(cams.size());

    for (const auto& kcam : cams) {
        CameraCalibration cc;
        cc.label = kcam.label;

        // Intrinsics: Kalibr stores [fx, fy, cx, cy]
        if (kcam.intrinsics.size() >= 4) {
            cc.intrinsics.fx = kcam.intrinsics[0];
            cc.intrinsics.fy = kcam.intrinsics[1];
            cc.intrinsics.cx = kcam.intrinsics[2];
            cc.intrinsics.cy = kcam.intrinsics[3];
        }

        // Resolution
        if (kcam.resolution.size() >= 2) {
            cc.intrinsics.width  = static_cast<uint32_t>(kcam.resolution[0]);
            cc.intrinsics.height = static_cast<uint32_t>(kcam.resolution[1]);
        }

        // Distortion
        cc.intrinsics.distortion_model = mapDistortionModel(kcam.distortion_model);
        for (size_t i = 0; i < kcam.distortion_coeffs.size() && i < 8; ++i)
            cc.intrinsics.distortion_coeffs[i] = kcam.distortion_coeffs[i];

        // Extrinsic: Kalibr gives T_cam_imu, we need imu_T_camera = T_cam_imu^{-1}
        if (kcam.has_T_cam_imu) {
            double T_imu_cam[16];
            invertTransform(kcam.T_cam_imu, T_imu_cam);
            cc.imu_T_camera = transformToExtrinsic(T_imu_cam);
        }

        // Time offset: Kalibr's timeshift_cam_imu is in seconds → convert to ns.
        // Kalibr convention: t_imu = t_cam + timeshift_cam_imu
        // Our convention: camera_timestamp = imu_timestamp + time_offset_ns
        // Therefore: time_offset_ns = -timeshift_cam_imu * 1e9
        cc.time_offset_ns = static_cast<int64_t>(-kcam.timeshift_cam_imu * 1.0e9);

        bundle.cameras.push_back(std::move(cc));
    }

    return true;
}

bool importKalibrImu(const std::string& path, CalibrationBundle& bundle) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;
    bool in_imu = false;

    while (std::getline(file, line)) {
        std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#') continue;

        int indent = indentLevel(line);

        // Top-level: imu0:
        if (indent == 0 && stripped.size() >= 4 &&
            stripped.substr(0, 3) == "imu" && stripped.back() == ':') {
            in_imu = true;
            continue;
        }

        if (!in_imu) continue;
        if (indent == 0) break;  // New top-level section

        auto colon = stripped.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(stripped.substr(0, colon));
        std::string val = trim(stripped.substr(colon + 1));

        if (key == "gyroscope_noise_density")
            bundle.imu_noise.gyro_noise = parseDouble(val, 1.0e-3);
        else if (key == "accelerometer_noise_density")
            bundle.imu_noise.accel_noise = parseDouble(val, 1.0e-2);
        else if (key == "gyroscope_random_walk")
            bundle.imu_noise.gyro_bias_rw = parseDouble(val, 1.0e-5);
        else if (key == "accelerometer_random_walk")
            bundle.imu_noise.accel_bias_rw = parseDouble(val, 1.0e-4);
    }

    return true;
}

bool importKalibrFull(const std::string& camchain_path,
                      const std::string& imu_path,
                      CalibrationBundle& bundle) {
    return importKalibrCamchain(camchain_path, bundle) &&
           importKalibrImu(imu_path, bundle);
}

}  // namespace thunderbird::calib

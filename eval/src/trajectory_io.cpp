// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — SLAM Evaluation Framework: TUM Trajectory I/O
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/trajectory_io.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace eval::tum {

// ─────────────────────────────────────────────────────────────────────────────
//  Write estimated poses → TUM format
//  Format: timestamp tx ty tz qx qy qz qw
//  SDK uses [w,x,y,z] → TUM uses [x,y,z,w]
// ─────────────────────────────────────────────────────────────────────────────

bool write(const std::vector<StampedPose>& poses, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    f << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";
    f << std::fixed << std::setprecision(9);

    for (const auto& p : poses) {
        const double ts = p.timestamp_ns / 1.0e9;
        f << ts
          << " " << p.position[0]
          << " " << p.position[1]
          << " " << p.position[2]
          << " " << p.quaternion[1]   // qx (SDK index 1)
          << " " << p.quaternion[2]   // qy (SDK index 2)
          << " " << p.quaternion[3]   // qz (SDK index 3)
          << " " << p.quaternion[0]   // qw (SDK index 0)
          << "\n";
    }
    return true;
}

bool write(const std::vector<GtPose>& poses, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    f << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";
    f << std::fixed << std::setprecision(9);

    for (const auto& p : poses) {
        const double ts = p.timestamp_ns / 1.0e9;
        f << ts
          << " " << p.position[0]
          << " " << p.position[1]
          << " " << p.position[2]
          << " " << p.quaternion[1]
          << " " << p.quaternion[2]
          << " " << p.quaternion[3]
          << " " << p.quaternion[0]
          << "\n";
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Read TUM file → GtPose vector
// ─────────────────────────────────────────────────────────────────────────────

std::vector<GtPose> read(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};

    std::vector<GtPose> poses;
    std::string line;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double ts, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;

        GtPose gp;
        gp.timestamp_ns  = static_cast<int64_t>(ts * 1.0e9);
        gp.position      = {tx, ty, tz};
        // TUM [qx,qy,qz,qw] → SDK [w,x,y,z]
        gp.quaternion    = {qw, qx, qy, qz};
        poses.push_back(gp);
    }
    return poses;
}

} // namespace eval::tum

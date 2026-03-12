// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Import Kalibr Calibration Results
// ─────────────────────────────────────────────────────────────────────────────
//
// CLI tool that reads Kalibr output YAML files and writes a Thunderbird-
// compatible calibration.yaml.
//
// Usage:
//   import_kalibr_demo --camchain camchain-imucam.yaml \
//                      --imu imu.yaml \
//                      -o calibration.yaml
//
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/calibration.h"

// Internal headers (in sdk/src/ — not installed)
#include "../sdk/src/calib/kalibr_import.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

static void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s --camchain <camchain.yaml> [--imu <imu.yaml>] [-o output.yaml]\n"
        "\n"
        "Import Kalibr calibration results into Thunderbird calibration.yaml.\n"
        "\n"
        "Options:\n"
        "  --camchain <path>  Kalibr camera chain YAML (required)\n"
        "  --imu <path>       Kalibr IMU parameters YAML (optional)\n"
        "  -o <path>          Output file (default: calibration.yaml)\n"
        "  --help             Show this help\n",
        prog);
}

int main(int argc, char* argv[]) {
    std::string camchain_path;
    std::string imu_path;
    std::string output_path = "calibration.yaml";

    int i = 1;
    while (i < argc) {
        std::string arg = argv[i];
        if (arg == "--camchain" && i + 1 < argc) {
            camchain_path = argv[i + 1];
            i += 2;
        } else if (arg == "--imu" && i + 1 < argc) {
            imu_path = argv[i + 1];
            i += 2;
        } else if (arg == "-o" && i + 1 < argc) {
            output_path = argv[i + 1];
            i += 2;
        } else if (arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (camchain_path.empty()) {
        std::fprintf(stderr, "Error: --camchain is required.\n");
        usage(argv[0]);
        return 1;
    }

    // Validate input files exist before opening (mitigate uncontrolled path usage)
    auto resolve_input = [](const std::string& p, const char* label, std::string& out) -> bool {
        std::error_code ec;
        auto canonical = std::filesystem::canonical(p, ec);
        if (ec || !std::filesystem::is_regular_file(canonical, ec)) {
            std::fprintf(stderr, "Error: %s path does not exist or is not a regular file: %s\n",
                         label, p.c_str());
            return false;
        }
        out = canonical.string();
        return true;
    };

    std::string resolved;
    if (!resolve_input(camchain_path, "camchain", resolved)) return 1;
    camchain_path = resolved;

    if (!imu_path.empty()) {
        if (!resolve_input(imu_path, "imu", resolved)) return 1;
        imu_path = resolved;
    }

    thunderbird::CalibrationBundle bundle;

    // Import camera chain
    std::printf("Importing camera chain from: %s\n", camchain_path.c_str());
    if (!thunderbird::calib::importKalibrCamchain(camchain_path, bundle)) {
        std::fprintf(stderr, "Error: Failed to parse camchain file: %s\n",
                     camchain_path.c_str());
        return 1;
    }

    std::printf("  Loaded %zu camera(s):\n", bundle.cameras.size());
    for (size_t i = 0; i < bundle.cameras.size(); ++i) {
        const auto& cam = bundle.cameras[i];
        std::printf("    %s: %.1f×%.1f  fx=%.1f fy=%.1f  cx=%.1f cy=%.1f",
                    cam.label.c_str(),
                    static_cast<double>(cam.intrinsics.width),
                    static_cast<double>(cam.intrinsics.height),
                    cam.intrinsics.fx, cam.intrinsics.fy,
                    cam.intrinsics.cx, cam.intrinsics.cy);
        if (cam.time_offset_ns != 0) {
            std::printf("  time_offset=%.3f ms",
                        static_cast<double>(cam.time_offset_ns) / 1.0e6);
        }
        std::printf("\n");
    }

    // Import IMU parameters (optional)
    if (!imu_path.empty()) {
        std::printf("Importing IMU parameters from: %s\n", imu_path.c_str());
        if (!thunderbird::calib::importKalibrImu(imu_path, bundle)) {
            std::fprintf(stderr, "Warning: Failed to parse IMU file: %s\n",
                         imu_path.c_str());
        } else {
            std::printf("  gyro_noise=%.2e  accel_noise=%.2e\n",
                        bundle.imu_noise.gyro_noise, bundle.imu_noise.accel_noise);
            std::printf("  gyro_bias_rw=%.2e  accel_bias_rw=%.2e\n",
                        bundle.imu_noise.gyro_bias_rw, bundle.imu_noise.accel_bias_rw);
        }
    }

    // Validate output path: resolve parent directory to prevent path traversal
    {
        std::error_code ec;
        auto parent = std::filesystem::absolute(output_path, ec).parent_path();
        if (ec) {
            std::fprintf(stderr, "Error: invalid output path: %s\n", output_path.c_str());
            return 1;
        }
        auto canon_parent = std::filesystem::canonical(parent, ec);
        if (ec || !std::filesystem::is_directory(canon_parent, ec)) {
            std::fprintf(stderr, "Error: output directory does not exist: %s\n",
                         parent.string().c_str());
            return 1;
        }
        output_path = (canon_parent / std::filesystem::path(output_path).filename()).string();
    }

    // Save to Thunderbird YAML
    std::printf("Saving to: %s\n", output_path.c_str());
    if (!bundle.save_yaml(output_path)) {
        std::fprintf(stderr, "Error: Failed to write output file: %s\n",
                     output_path.c_str());
        return 1;
    }

    std::printf("Done.\n");
    return 0;
}

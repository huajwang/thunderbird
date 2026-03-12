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

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camchain" && i + 1 < argc) {
            camchain_path = argv[++i];
        } else if (arg == "--imu" && i + 1 < argc) {
            imu_path = argv[++i];
        } else if (arg == "-o" && i + 1 < argc) {
            output_path = argv[++i];
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

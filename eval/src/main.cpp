// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — slam_eval: SLAM Evaluation CLI
// ─────────────────────────────────────────────────────────────────────────────
//
// Usage:
//   slam_eval [OPTIONS] <dataset_path>
//
// Examples:
//   slam_eval /data/kitti/sequences/00
//   slam_eval -f kitti --max_frames 100 /data/kitti/sequences/00
//   slam_eval --csv-only --timeout 300 -o results/ /data/kitti/sequences/00
//
// ─────────────────────────────────────────────────────────────────────────────

#include "eval/dataset_adapter.h"
#include "eval/evaluator.h"
#include "eval/frame_stream.h"
#include "eval/imu_interpolator.h"
#include "eval/metrics.h"
#include "eval/perf_timer.h"
#include "eval/pose_recorder.h"
#include "eval/resource_monitor.h"
#include "eval/fault_injector.h"
#include "eval/stress_test_adapter.h"
#include "eval/trajectory_io.h"

#include "thunderbird/odom/slam_engine.h"
#include "thunderbird/odom/slam_types.h"
#include "thunderbird/version.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace fs = std::filesystem;
using namespace eval;
using namespace thunderbird::odom;

// ─────────────────────────────────────────────────────────────────────────────
//  CLI argument parsing
// ─────────────────────────────────────────────────────────────────────────────

struct CliArgs {
    std::string dataset_path;
    std::string format      = "auto";
    std::string output_dir  = "./eval_output";
    std::string gt_override;
    std::string config_file;            // YAML config for SlamEngineConfig
    std::string stress_preset;          // stress-test preset name
    size_t      max_frames  = 0;        // 0 = unlimited
    double      rpe_delta_m = 100.0;
    double      imu_rate_hz = 200.0;    // synth IMU rate
    int         timeout_s   = 0;        // 0 = none
    bool        synth_imu   = false;    // enable synthetic IMU interpolation
    bool        csv_only    = false;
    bool        no_plot     = false;
    bool        verbose     = false;
    bool        help        = false;
    bool        version     = false;
    bool        no_align    = false;    // disable trajectory alignment
    bool        scale_align = false;    // enable Sim(3) scale recovery
    bool        outlier_reject = false; // enable outlier rejection

    // ── Fault injection (robustness testing) ────────────────────────────
    double fault_lidar_drop  = 0.0;     // probability [0.0, 1.0]
    double fault_imu_noise   = 0.0;     // noise multiplier (0.0 = off)
    int64_t fault_ts_jitter  = 0;       // nanoseconds (0 = off)
    int    fault_rate_div    = 1;       // LiDAR rate divisor (1 = off)

    // ── Profiling ───────────────────────────────────────────────────────
    bool   profile          = false;    // enable runtime profiling output
};

static void print_usage() {
    std::fprintf(stderr, R"(
slam_eval — Thunderbird SLAM Evaluation Tool

USAGE:
  slam_eval [OPTIONS] <dataset_path>
  slam_eval --stress <preset>

POSITIONAL:
  dataset_path              Path to dataset root directory or .tbrec file

OPTIONS:
  -f, --format <fmt>        Dataset format: kitti | euroc | tbrec | auto (default: auto)
  -o, --output <dir>        Output directory (default: ./eval_output/)
      --max_frames <N>      Maximum LiDAR frames to process (default: 0 = all)
      --gt <path>           Override ground truth file (TUM format)
      --rpe-delta <metres>  RPE segment length in metres (default: 100)
      --timeout <seconds>   Abort if sequence takes longer than this (default: 0 = none)
  -c, --config <yaml>       YAML config file for SlamEngineConfig (drone.yaml / car.yaml)
      --synth-imu           Generate synthetic IMU from GT (for LiDAR-only datasets)
      --imu-rate <Hz>       Synthetic IMU rate (default: 200; requires --synth-imu)
      --stress <preset>     Run stress test: aggressive_drone | fast_car |
                            degenerate_corridor | spinning_top | custom
      --no-align            Disable SE(3) trajectory alignment before ATE
      --scale-align         Enable Sim(3) alignment (estimate scale)
      --outlier-reject      Enable iterative outlier rejection during alignment
      --csv-only            Only emit metrics.csv (skip JSON, TUM, plot)
      --no-plot             Skip PNG trajectory plot generation
  -v, --verbose             Print per-frame timing to stderr
  -h, --help                Show help
      --version             Print version

FAULT INJECTION (robustness testing):
      --fault-drop <rate>   Drop LiDAR frames with probability [0.0-1.0]
      --fault-imu-noise <x> Inject additive IMU noise (multiplier, e.g. 2.0)
      --fault-jitter <ns>   Uniform timestamp jitter ± nanoseconds
      --fault-rate-div <N>  Decimate LiDAR to 1/N rate (2 = 50%%)

PROFILING:
      --profile             Enable module-level runtime profiling output

EXIT CODES:
  0   All metrics computed successfully
  1   Dataset open failed
  2   Engine initialization failed
  3   Timeout exceeded
)");
}

static CliArgs parse_args(int argc, char* argv[]) {
    CliArgs args;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        if (a == "-h" || a == "--help")     { args.help = true; return args; }
        if (a == "--version")               { args.version = true; return args; }
        if (a == "-v" || a == "--verbose")   { args.verbose = true; continue; }
        if (a == "--csv-only")              { args.csv_only = true; continue; }
        if (a == "--no-plot")               { args.no_plot = true; continue; }
        if (a == "--synth-imu")             { args.synth_imu = true; continue; }
        if (a == "--no-align")              { args.no_align = true; continue; }
        if (a == "--scale-align")           { args.scale_align = true; continue; }
        if (a == "--outlier-reject")        { args.outlier_reject = true; continue; }
        if (a == "--profile")               { args.profile = true; continue; }

        if ((a == "-f" || a == "--format") && i+1 < argc) {
            args.format = argv[++i]; continue;
        }
        if ((a == "-o" || a == "--output") && i+1 < argc) {
            args.output_dir = argv[++i]; continue;
        }
        if (a == "--max_frames" && i+1 < argc) {
            args.max_frames = static_cast<size_t>(std::atol(argv[++i])); continue;
        }
        if (a == "--gt" && i+1 < argc) {
            args.gt_override = argv[++i]; continue;
        }
        if (a == "--rpe-delta" && i+1 < argc) {
            args.rpe_delta_m = std::atof(argv[++i]); continue;
        }
        if (a == "--timeout" && i+1 < argc) {
            args.timeout_s = std::atoi(argv[++i]); continue;
        }
        if (a == "--imu-rate" && i+1 < argc) {
            args.imu_rate_hz = std::atof(argv[++i]); continue;
        }
        if ((a == "-c" || a == "--config") && i+1 < argc) {
            args.config_file = argv[++i]; continue;
        }
        if (a == "--stress" && i+1 < argc) {
            args.stress_preset = argv[++i]; continue;
        }
        if (a == "--fault-drop" && i+1 < argc) {
            args.fault_lidar_drop = std::atof(argv[++i]); continue;
        }
        if (a == "--fault-imu-noise" && i+1 < argc) {
            args.fault_imu_noise = std::atof(argv[++i]); continue;
        }
        if (a == "--fault-jitter" && i+1 < argc) {
            args.fault_ts_jitter = std::atoll(argv[++i]); continue;
        }
        if (a == "--fault-rate-div" && i+1 < argc) {
            args.fault_rate_div = std::atoi(argv[++i]); continue;
        }

        // Positional: dataset path.
        if (args.dataset_path.empty() && a[0] != '-') {
            args.dataset_path = a; continue;
        }

        std::fprintf(stderr, "[slam_eval] unknown option: %s\n", a.c_str());
    }

    return args;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv);

    if (args.help)    { print_usage(); return 0; }
    if (args.version) { std::printf("slam_eval %s\n", THUNDERBIRD_VERSION_STRING); return 0; }

    // ── Handle stress-test mode ─────────────────────────────────────────
    const bool stress_mode = !args.stress_preset.empty();

    if (!stress_mode && args.dataset_path.empty()) {
        std::fprintf(stderr, "[slam_eval] error: no dataset path specified\n");
        print_usage();
        return 1;
    }

    // ── 1. Create output directory ──────────────────────────────────────
    fs::create_directories(args.output_dir);

    // ── 2. Create adapter ───────────────────────────────────────────────
    std::unique_ptr<DatasetAdapter> adapter;
    if (stress_mode) {
        // Stress-test: use preset name as format.
        adapter = createAdapter(args.stress_preset, false, {});
        if (!adapter) {
            std::fprintf(stderr, "[slam_eval] unknown stress preset: %s\n",
                         args.stress_preset.c_str());
            return 1;
        }
        // Open with preset name as path identifier.
        if (!adapter->open(args.stress_preset, args.max_frames)) {
            std::fprintf(stderr, "[slam_eval] failed to init stress preset: %s\n",
                         args.stress_preset.c_str());
            return 1;
        }
    } else {
        ImuInterpolatorConfig imu_cfg;
        imu_cfg.imu_rate_hz = args.imu_rate_hz;

        if (args.format == "auto") {
            adapter = createAdapterFromPath(args.dataset_path,
                                             args.synth_imu, imu_cfg);
        } else {
            adapter = createAdapter(args.format, args.synth_imu, imu_cfg);
        }
        if (!adapter) return 1;

        if (!adapter->open(args.dataset_path, args.max_frames)) {
            std::fprintf(stderr, "[slam_eval] failed to open dataset: %s\n",
                         args.dataset_path.c_str());
            return 1;
        }
    }

    // ── 2b. Wrap with fault injector if any fault flags are set ──────────
    FaultConfig fault;
    fault.lidar_drop_rate    = args.fault_lidar_drop;
    fault.imu_noise_scale    = args.fault_imu_noise;
    fault.timestamp_jitter_ns = args.fault_ts_jitter;
    fault.lidar_rate_divisor = args.fault_rate_div;

    FaultInjectorAdapter* fault_adapter_ptr = nullptr;
    if (fault.active()) {
        auto faulty = std::make_unique<FaultInjectorAdapter>(
            std::move(adapter), fault);
        fault_adapter_ptr = faulty.get();
        adapter = std::move(faulty);
        std::fprintf(stderr, "[slam_eval] fault injection: %s\n",
                     fault.label().c_str());
    }

    auto di = adapter->info();
    std::fprintf(stderr, "[slam_eval] dataset: %s  format: %s  lidar: %s  imu: %s  gt: %s\n",
                 di.name.c_str(), di.format.c_str(),
                 di.has_lidar ? "yes" : "no",
                 di.has_imu   ? "yes" : "no",
                 di.has_ground_truth ? "yes" : "no");

    // ── 3. Load ground truth ────────────────────────────────────────────
    PoseRecorder recorder;
    if (!args.gt_override.empty()) {
        auto gt = tum::read(args.gt_override);
        std::fprintf(stderr, "[slam_eval] loaded GT override: %zu poses from %s\n",
                     gt.size(), args.gt_override.c_str());
        recorder.setGroundTruth(std::move(gt));
    } else {
        recorder.setGroundTruth(adapter->loadGroundTruth());
    }

    // ── 4. Init engine ──────────────────────────────────────────────────
    SlamEngineConfig config;
    config.publish_propagated_poses = false;  // we only need corrected poses

    // Load YAML config if specified.
    if (!args.config_file.empty()) {
        if (!fs::is_regular_file(args.config_file)) {
            std::fprintf(stderr, "[slam_eval] config file not found: %s\n",
                         args.config_file.c_str());
            return 1;
        }
        // ── Minimal YAML key:value parser (no dependency required) ──────
        // Supports the flat fields used in drone.yaml / car.yaml.
        std::ifstream cfg_f(args.config_file);
        std::string line;
        while (std::getline(cfg_f, line)) {
            // Strip comments and leading whitespace.
            auto hash = line.find('#');
            if (hash != std::string::npos) line.erase(hash);
            // Trim leading spaces.
            size_t start = line.find_first_not_of(" \t\r\n");
            if (start == std::string::npos) continue;
            line = line.substr(start);

            auto colon = line.find(':');
            if (colon == std::string::npos) continue;

            std::string key = line.substr(0, colon);
            std::string val = line.substr(colon + 1);
            // Trim key/val.
            while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
            size_t vs = val.find_first_not_of(" \t");
            if (vs == std::string::npos) continue;
            val = val.substr(vs);
            // Remove trailing whitespace.
            while (!val.empty() && (val.back() == ' ' || val.back() == '\t' || val.back() == '\r'))
                val.pop_back();

            // Map known keys to config fields.
            if (key == "gyro_noise")      config.imu_noise.gyro_noise     = std::stod(val);
            else if (key == "accel_noise")config.imu_noise.accel_noise    = std::stod(val);
            else if (key == "gyro_bias_rw")  config.imu_noise.gyro_bias_rw  = std::stod(val);
            else if (key == "accel_bias_rw") config.imu_noise.accel_bias_rw = std::stod(val);
            else if (key == "refine_online") config.extrinsic.refine_online = (val == "true");
            else if (key == "max_iterations")    config.esikf.max_iterations     = std::stoi(val);
            else if (key == "convergence_eps")   config.esikf.convergence_eps    = std::stod(val);
            else if (key == "plane_noise_sigma") config.esikf.plane_noise_sigma  = std::stod(val);
            else if (key == "min_correspondences") config.esikf.min_correspondences = std::stoi(val);
            else if (key == "max_residual")      config.esikf.max_residual       = std::stod(val);
            else if (key == "enable")            config.deskew.enable            = (val == "true");
            else if (key == "imu_integration_substeps") config.deskew.imu_integration_substeps = std::stoi(val);
            else if (key == "gravity_duration_s")  config.init.init_gravity_duration_s = std::stod(val);
            else if (key == "min_imu_samples")     config.init.init_min_imu_samples    = std::stoi(val);
            else if (key == "gravity_tolerance")   config.init.init_gravity_tolerance   = std::stod(val);
            else if (key == "voxel_resolution")    config.map.voxel_resolution    = std::stod(val);
            else if (key == "map_radius")          config.map.map_radius          = std::stod(val);
            else if (key == "max_map_points")      config.map.max_map_points      = static_cast<size_t>(std::stol(val));
            else if (key == "delete_ratio")        config.map.delete_ratio        = std::stod(val);
            else if (key == "tree_rebalance_interval") config.map.tree_rebalance_interval = std::stoi(val);
            else if (key == "imu_rate_hz")         config.imu_rate_hz             = std::stod(val);
            else if (key == "lidar_rate_hz")       config.lidar_rate_hz           = std::stod(val);
            else if (key == "enable_drift_compensation") config.enable_drift_compensation = (val == "true");
            else if (key == "sort_window_ns")      config.sort_window_ns          = std::stoll(val);
            else if (key == "publish_propagated_poses") config.publish_propagated_poses = (val == "true");
        }
        std::fprintf(stderr, "[slam_eval] loaded config: %s\n",
                     args.config_file.c_str());
    }

    AcmeSlamEngine engine;
    if (!engine.initialize(config)) {
        std::fprintf(stderr, "[slam_eval] engine initialization failed\n");
        return 2;
    }

    // Register pose recording callback.
    engine.onSlamOutput([&recorder](std::shared_ptr<const SlamOutput> out) {
        recorder.record(*out);
    });

    // ── 5. Start resource monitor ───────────────────────────────────────
    ResourceMonitor resources;
    resources.start();

    // ── 6. Replay ───────────────────────────────────────────────────────
    std::fprintf(stderr, "[slam_eval] starting replay...\n");

    PerfTimer overall;
    overall.start("total");

    FrameStream stream;
    auto stats = stream.replay(*adapter, engine,
        args.verbose ? [](size_t fed, size_t total) {
            std::fprintf(stderr, "[slam_eval] progress: %zu frames fed\n", fed);
        } : std::function<void(size_t, size_t)>{});

    // Give engine time to finish processing remaining data.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Drain any remaining outputs.
    std::vector<SlamOutput> remaining;
    engine.drainOutputs(remaining);
    for (const auto& out : remaining) {
        recorder.record(out);
    }

    double total_ms = overall.stop_ms("total");

    // ── 7. Capture profiling snapshot (before shutdown) ─────────────────
    ProfileSnapshot prof_snap;
    if (args.profile) {
        prof_snap = engine.profileSnapshot();
    }

    // ── 8. Stop ─────────────────────────────────────────────────────────
    engine.shutdown();
    resources.stop();

    std::fprintf(stderr,
        "[slam_eval] replay done: %zu lidar, %zu imu in %.1f s (%.1f lidar fps)\n",
        stats.lidar_fed, stats.imu_fed, stats.wall_time_s, stats.replay_fps);
    std::fprintf(stderr,
        "[slam_eval] recorded %zu estimated poses\n", recorder.size());

    // ── 9. Evaluate ─────────────────────────────────────────────────────
    Evaluator evaluator({
        .rpe_delta_m            = args.rpe_delta_m,
        .align_trajectories     = !args.no_align,
        .estimate_scale         = args.scale_align,
        .outlier_rejection      = args.outlier_reject,
    });
    auto metrics = evaluator.compute(recorder, resources, stats.timings);
    metrics.dataset_name = di.name;

    // Print summary to stderr.
    std::fprintf(stderr, "\n════════════════════════════════════════\n");
    std::fprintf(stderr, "  SLAM Evaluation Results: %s\n", di.name.c_str());
    std::fprintf(stderr, "════════════════════════════════════════\n");
    std::fprintf(stderr, "  ATE RMSE:        %.4f m\n", metrics.ate.rmse_m);
    std::fprintf(stderr, "  ATE max:         %.4f m\n", metrics.ate.max_m);
    std::fprintf(stderr, "  RPE trans:       %.4f m\n", metrics.rpe.trans_rmse_m);
    std::fprintf(stderr, "  RPE rot:         %.4f deg\n", metrics.rpe.rot_rmse_deg);
    std::fprintf(stderr, "  Drift:           %.2f %%\n", metrics.drift.drift_pct);
    std::fprintf(stderr, "  Drift/100m:      %.4f m\n", metrics.drift.drift_per_100m);
    std::fprintf(stderr, "  Avg frame time:  %.2f ms\n", metrics.runtime.avg_ms);
    std::fprintf(stderr, "  P95 frame time:  %.2f ms\n", metrics.runtime.p95_ms);
    std::fprintf(stderr, "  Peak RSS:        %.1f MB\n",
                 metrics.resources.peak_rss_bytes / (1024.0 * 1024.0));
    std::fprintf(stderr, "  Avg CPU:         %.1f %%\n", metrics.resources.avg_cpu_pct);
    std::fprintf(stderr, "════════════════════════════════════════\n\n");

    // ── Print profiling results (if --profile) ──────────────────────────
    if (args.profile) {
        std::fprintf(stderr, "════════════════════════════════════════════════════════════════\n");
        std::fprintf(stderr, "  Runtime Profiling — Module-Level Breakdown\n");
        std::fprintf(stderr, "════════════════════════════════════════════════════════════════\n");
        std::fprintf(stderr, "  %-16s %8s %8s %8s %8s %8s %8s %7s\n",
                     "Module", "Avg(us)", "P50(us)", "P95(us)", "P99(us)",
                     "Max(us)", "Tot(ms)", "CPU%%");
        std::fprintf(stderr, "  %-16s %8s %8s %8s %8s %8s %8s %7s\n",
                     "────────────────", "────────", "────────", "────────",
                     "────────", "────────", "────────", "───────");

        for (size_t i = 0; i < kProfileModuleCount; ++i) {
            const auto& m = prof_snap.modules[i];
            if (m.invocations == 0) continue;
            std::fprintf(stderr, "  %-16s %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %6.1f%%\n",
                         m.name, m.avg_us, m.p50_us, m.p95_us, m.p99_us,
                         m.max_us, m.total_ms, m.cpu_pct);
        }

        std::fprintf(stderr, "\n  Worker utilisation:  %.1f%%\n",
                     prof_snap.worker_utilization_pct);
        std::fprintf(stderr, "  Scan latency:       avg=%.2f  p50=%.2f  p95=%.2f  p99=%.2f  max=%.2f ms\n",
                     prof_snap.avg_scan_latency_ms,
                     prof_snap.p50_scan_latency_ms,
                     prof_snap.p95_scan_latency_ms,
                     prof_snap.p99_scan_latency_ms,
                     prof_snap.max_scan_latency_ms);
        std::fprintf(stderr, "  Peak RSS:           %.1f MB\n",
                     prof_snap.peak_rss_bytes / (1024.0 * 1024.0));
        std::fprintf(stderr, "  Total scans:        %zu    IMU samples: %zu\n",
                     prof_snap.total_scans, prof_snap.total_imu_samples);
        std::fprintf(stderr, "  Wall time:          %.2f s\n", prof_snap.wall_time_s);

        // ── Identify bottleneck ─────────────────────────────────────────
        size_t bottleneck_idx = 0;
        double bottleneck_pct = 0;
        for (size_t i = 0; i < kProfileModuleCount; ++i) {
            auto mod = static_cast<ProfileModule>(i);
            if (mod == ProfileModule::ScanTotal ||
                mod == ProfileModule::WorkerIdle ||
                mod == ProfileModule::ImuPropagate) continue;
            if (prof_snap.modules[i].cpu_pct > bottleneck_pct) {
                bottleneck_pct = prof_snap.modules[i].cpu_pct;
                bottleneck_idx = i;
            }
        }
        if (bottleneck_pct > 0) {
            std::fprintf(stderr, "\n  >>> BOTTLENECK: %s (%.1f%% of scan time)\n",
                         prof_snap.modules[bottleneck_idx].name, bottleneck_pct);
        }
        std::fprintf(stderr, "════════════════════════════════════════════════════════════════\n\n");

        // ── Write profiling CSV ─────────────────────────────────────────
        {
            std::ofstream pcsv(args.output_dir + "/profile.csv");
            if (pcsv.is_open()) {
                pcsv << "module,invocations,avg_us,min_us,max_us,"
                     << "p50_us,p95_us,p99_us,total_ms,cpu_pct\n";
                for (size_t i = 0; i < kProfileModuleCount; ++i) {
                    const auto& m = prof_snap.modules[i];
                    if (m.invocations == 0) continue;
                    pcsv << m.name
                         << "," << m.invocations
                         << "," << m.avg_us
                         << "," << m.min_us
                         << "," << m.max_us
                         << "," << m.p50_us
                         << "," << m.p95_us
                         << "," << m.p99_us
                         << "," << m.total_ms
                         << "," << m.cpu_pct
                         << "\n";
                }
                // Summary row
                pcsv << "\n# Summary\n";
                pcsv << "# worker_utilization_pct," << prof_snap.worker_utilization_pct << "\n";
                pcsv << "# peak_rss_mb," << prof_snap.peak_rss_bytes / (1024.0 * 1024.0) << "\n";
                pcsv << "# avg_scan_latency_ms," << prof_snap.avg_scan_latency_ms << "\n";
                pcsv << "# p95_scan_latency_ms," << prof_snap.p95_scan_latency_ms << "\n";
                pcsv << "# p99_scan_latency_ms," << prof_snap.p99_scan_latency_ms << "\n";
                pcsv << "# max_scan_latency_ms," << prof_snap.max_scan_latency_ms << "\n";
                pcsv << "# total_scans," << prof_snap.total_scans << "\n";
                pcsv << "# wall_time_s," << prof_snap.wall_time_s << "\n";
            }
        }

        // ── Write profiling JSON ────────────────────────────────────────
        if (!args.csv_only) {
            std::ofstream pjson(args.output_dir + "/profile.json");
            if (pjson.is_open()) {
                pjson << "{\n";
                pjson << "  \"wall_time_s\": " << prof_snap.wall_time_s << ",\n";
                pjson << "  \"worker_utilization_pct\": " << prof_snap.worker_utilization_pct << ",\n";
                pjson << "  \"peak_rss_bytes\": " << prof_snap.peak_rss_bytes << ",\n";
                pjson << "  \"total_scans\": " << prof_snap.total_scans << ",\n";
                pjson << "  \"total_imu_samples\": " << prof_snap.total_imu_samples << ",\n";
                pjson << "  \"scan_latency\": {\n";
                pjson << "    \"avg_ms\": " << prof_snap.avg_scan_latency_ms << ",\n";
                pjson << "    \"p50_ms\": " << prof_snap.p50_scan_latency_ms << ",\n";
                pjson << "    \"p95_ms\": " << prof_snap.p95_scan_latency_ms << ",\n";
                pjson << "    \"p99_ms\": " << prof_snap.p99_scan_latency_ms << ",\n";
                pjson << "    \"max_ms\": " << prof_snap.max_scan_latency_ms << "\n";
                pjson << "  },\n";
                pjson << "  \"modules\": [\n";
                bool first = true;
                for (size_t i = 0; i < kProfileModuleCount; ++i) {
                    const auto& m = prof_snap.modules[i];
                    if (m.invocations == 0) continue;
                    if (!first) pjson << ",\n";
                    first = false;
                    pjson << "    {\n";
                    pjson << "      \"name\": \"" << m.name << "\",\n";
                    pjson << "      \"invocations\": " << m.invocations << ",\n";
                    pjson << "      \"avg_us\": " << m.avg_us << ",\n";
                    pjson << "      \"min_us\": " << m.min_us << ",\n";
                    pjson << "      \"max_us\": " << m.max_us << ",\n";
                    pjson << "      \"p50_us\": " << m.p50_us << ",\n";
                    pjson << "      \"p95_us\": " << m.p95_us << ",\n";
                    pjson << "      \"p99_us\": " << m.p99_us << ",\n";
                    pjson << "      \"total_ms\": " << m.total_ms << ",\n";
                    pjson << "      \"cpu_pct\": " << m.cpu_pct << "\n";
                    pjson << "    }";
                }
                pjson << "\n  ]\n";
                pjson << "}\n";
            }
        }
    }

    // ── 10. Write outputs ───────────────────────────────────────────────
    Evaluator::writeCsv(metrics, args.output_dir + "/metrics.csv");

    // Write fault stats if fault injection was active.
    if (fault_adapter_ptr) {
        const auto& fs = fault_adapter_ptr->faultStats();
        const auto& fc = fault_adapter_ptr->faultConfig();

        std::fprintf(stderr, "\n── Fault Injection Summary ──────────────\n");
        std::fprintf(stderr, "  Config:          %s\n", fc.label().c_str());
        std::fprintf(stderr, "  LiDAR total:     %zu\n", fs.lidar_total);
        std::fprintf(stderr, "  LiDAR dropped:   %zu\n", fs.lidar_dropped);
        std::fprintf(stderr, "  LiDAR decimated: %zu\n", fs.lidar_decimated);
        std::fprintf(stderr, "  LiDAR delivered: %zu\n", fs.lidar_delivered);
        std::fprintf(stderr, "  IMU total:       %zu\n", fs.imu_total);
        std::fprintf(stderr, "  Events jittered: %zu\n", fs.events_jittered);
        std::fprintf(stderr, "────────────────────────────────────────\n\n");

        // Append fault columns to metrics.csv for automation parsing.
        std::ofstream fault_csv(args.output_dir + "/fault_stats.csv");
        if (fault_csv.is_open()) {
            fault_csv << "fault_label,lidar_drop_rate,imu_noise_scale,"
                      << "ts_jitter_ns,rate_divisor,"
                      << "lidar_total,lidar_dropped,lidar_decimated,"
                      << "lidar_delivered,imu_total\n";
            fault_csv << fc.label()
                      << "," << fc.lidar_drop_rate
                      << "," << fc.imu_noise_scale
                      << "," << fc.timestamp_jitter_ns
                      << "," << fc.lidar_rate_divisor
                      << "," << fs.lidar_total
                      << "," << fs.lidar_dropped
                      << "," << fs.lidar_decimated
                      << "," << fs.lidar_delivered
                      << "," << fs.imu_total
                      << "\n";
        }
    }

    if (!args.csv_only) {
        Evaluator::writeJson(metrics, args.output_dir + "/summary.json");

        auto est = recorder.estimated();
        Evaluator::writeTumTrajectory(est, args.output_dir + "/trajectory_est.tum");

        const auto& gt = recorder.groundTruth();
        if (!gt.empty()) {
            Evaluator::writeTumTrajectory(gt, args.output_dir + "/trajectory_gt.tum");
        }

        // ── Plot generation (requires python3 + matplotlib + numpy) ──
        if (!args.no_plot) {
            std::string script;
#ifdef EVAL_SCRIPTS_DIR
            script = std::string(EVAL_SCRIPTS_DIR) + "/plot_eval.py";
#endif
            // Fallback: search relative to executable.
            if (script.empty() || !fs::exists(script)) {
                auto exe_dir = fs::path(argv[0]).parent_path();
                for (auto& candidate : {
                    exe_dir / ".." / "scripts" / "plot_eval.py",
                    exe_dir / "scripts" / "plot_eval.py",
                    exe_dir / ".." / "eval" / "scripts" / "plot_eval.py",
                }) {
                    if (fs::exists(candidate)) {
                        script = candidate.string();
                        break;
                    }
                }
            }

            if (!script.empty() && fs::exists(script)) {
                // Try python3 first, then python.
                for (const char* py : {"python3", "python"}) {
                    std::string cmd = std::string(py) + " \""
                        + script + "\" \"" + args.output_dir + "\"";
                    int rc = std::system(cmd.c_str());
                    if (rc == 0) {
                        std::fprintf(stderr,
                            "[slam_eval] plots saved to %s/\n",
                            args.output_dir.c_str());
                        break;
                    }
                }
            } else {
                std::fprintf(stderr,
                    "[slam_eval] plot script not found — "
                    "skipping PNG generation\n"
                    "             install matplotlib+numpy or run "
                    "plot_eval.py manually\n");
            }
        }
    }

    std::fprintf(stderr, "[slam_eval] all outputs written to %s/\n",
                 args.output_dir.c_str());
    return 0;
}

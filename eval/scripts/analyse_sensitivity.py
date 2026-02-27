#!/usr/bin/env python3
"""
Thunderbird SLAM — Sensitivity Sweep Analyser
Reads sensitivity_results.csv and produces:
  1. Per-parameter sensitivity tables (markdown)
  2. Optimal value recommendations for drone and car
  3. Updated YAML configs
"""
import csv
import sys
import os
from collections import defaultdict

# ── Thresholds for grading ──────────────────────────────────────────────────
ATE_GOOD, ATE_WARN = 2.0, 10.0      # metres
DRIFT_GOOD, DRIFT_WARN_UNUSED = 2.0, 5.0   # m/100m
RT_GOOD, _RT_WARN_UNUSED = 15.0, 30.0       # ms avg frame

# Presets grouped by platform
DRONE_PRESETS = ["aggressive_drone", "spinning_top"]
CAR_PRESETS   = ["fast_car", "degenerate_corridor"]
ALL_PRESETS   = DRONE_PRESETS + CAR_PRESETS

# Human-friendly dimension labels
DIMENSIONS = {
    "gyro_noise":               "IMU Noise Covariance",
    "accel_noise":              "IMU Noise Covariance",
    "gyro_bias_rw":             "IMU Noise Covariance",
    "accel_bias_rw":            "IMU Noise Covariance",
    "voxel_resolution":         "Voxel Filter Resolution",
    "convergence_eps":          "Keyframe Selection Threshold",
    "min_correspondences":      "Keyframe Selection Threshold",
    "max_iterations":           "Motion Model Constraints",
    "imu_integration_substeps": "Motion Model Constraints",
    "plane_noise_sigma":        "Motion Model Constraints",
}

def grade(val, good, warn):
    if val <= good: return "good"
    if val <= warn: return "warn"
    return "fail"

def fmt_sci(v):
    """Format small numbers in scientific notation, larger ones normally."""
    f = float(v)
    if abs(f) < 0.01 or abs(f) >= 10000:
        return f"{f:.1e}"
    return f"{f:g}"

def load_results(csv_path):
    """Load sweep results CSV into a list of dicts."""
    rows = []
    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for r in reader:
            # Convert numeric fields
            for k in ["ate_rmse", "ate_max", "rpe_trans", "drift_pct",
                       "drift_100m", "rt_avg", "rt_p95", "rt_max",
                       "num_poses", "distance"]:
                try:
                    r[k] = float(r.get(k, 0))
                except (ValueError, TypeError):
                    r[k] = 0.0
            rows.append(r)
    return rows

def compute_composite_score(rows_for_point):
    """
    Composite score = weighted sum across presets.
    Lower is better.
    Weights: ATE×1.0 + Drift×0.5 + RT×0.1
    We average across the presets present.
    """
    if not rows_for_point:
        return 1e9
    total = 0.0
    for r in rows_for_point:
        total += r["ate_rmse"] * 1.0 + r["drift_100m"] * 0.5 + r["rt_avg"] * 0.1
    return total / len(rows_for_point)

def compute_platform_score(rows_for_point, platform_presets):
    """Same as composite but only for a specific platform's presets."""
    filtered = [r for r in rows_for_point if r["preset"] in platform_presets]
    return compute_composite_score(filtered)


def analyse(csv_path, output_dir):
    rows = load_results(csv_path)
    if not rows:
        print("ERROR: No results found in", csv_path)
        return

    # ── Separate baseline from sweep rows ───────────────────────────────
    baseline_rows = [r for r in rows if r["sweep_param"] == "baseline"]
    sweep_rows = [r for r in rows if r["sweep_param"] != "baseline"]

    # Baseline metrics by preset
    baseline = {}
    for r in baseline_rows:
        baseline[r["preset"]] = r

    # ── Group sweep rows: param → value → [rows per preset] ─────────────
    grouped = defaultdict(lambda: defaultdict(list))
    for r in sweep_rows:
        grouped[r["sweep_param"]][r["sweep_value"]].append(r)

    # ── Build report ────────────────────────────────────────────────────
    lines = []
    lines.append("# Thunderbird SLAM — Parameter Sensitivity Analysis\n")
    lines.append("**Method:** One-parameter-at-a-time sweep over stress-test presets")
    lines.append("**Presets:** aggressive_drone (EuRoC proxy), fast_car (KITTI proxy), degenerate_corridor, spinning_top")
    lines.append("**Baseline:** Engine compiled defaults")
    lines.append("**Scoring:** Composite = ATE_RMSE x 1.0 + Drift/100m x 0.5 + RT_avg x 0.1  (lower is better)\n")

    # ── Baseline table ──────────────────────────────────────────────────
    lines.append("## Baseline Performance\n")
    lines.append("| Preset | ATE RMSE (m) | Drift/100m (m) | RPE (m) | Avg RT (ms) |")
    lines.append("|---|---:|---:|---:|---:|")
    for p in ALL_PRESETS:
        if p in baseline:
            b = baseline[p]
            lines.append(f"| {p} | {b['ate_rmse']:.2f} | {b['drift_100m']:.2f} | {b['rpe_trans']:.2f} | {b['rt_avg']:.2f} |")
    lines.append("")

    # ── Per-parameter sensitivity tables ────────────────────────────────
    lines.append("---\n")
    lines.append("## Parameter Sensitivity Tables\n")

    # Track best values per platform
    best_drone = {}  # param → (value, score)
    best_car   = {}

    current_dim = None
    for param in grouped:
        dim = DIMENSIONS.get(param, "Other")
        if dim != current_dim:
            lines.append(f"### Dimension: {dim}\n")
            current_dim = dim

        lines.append(f"#### `{param}`\n")

        values = grouped[param]
        # Sort by numeric value
        sorted_vals = sorted(values.keys(), key=lambda x: float(x))

        # Table header: one column per preset + composite scores
        lines.append(f"| Value | " + " | ".join(f"ATE ({p})" for p in ALL_PRESETS) +
                     " | Drone Score | Car Score | Overall |")
        lines.append("|---:" + "|---:" * (len(ALL_PRESETS) + 3) + "|")

        best_overall = (None, 1e9)
        best_d = (None, 1e9)
        best_c = (None, 1e9)

        for val in sorted_vals:
            point_rows = values[val]
            drone_score = compute_platform_score(point_rows, DRONE_PRESETS)
            car_score   = compute_platform_score(point_rows, CAR_PRESETS)
            overall     = compute_composite_score(point_rows)

            if overall < best_overall[1]:
                best_overall = (val, overall)
            if drone_score < best_d[1]:
                best_d = (val, drone_score)
            if car_score < best_c[1]:
                best_c = (val, car_score)

            # ATE per preset
            ate_cells = []
            for p in ALL_PRESETS:
                matching = [r for r in point_rows if r["preset"] == p]
                if matching:
                    ate = matching[0]["ate_rmse"]
                    bate = baseline.get(p, {}).get("ate_rmse", ate)
                    delta_pct = ((ate - bate) / max(bate, 1e-9)) * 100 if bate else 0
                    sign = "+" if delta_pct > 0 else ""
                    g = grade(ate, ATE_GOOD, ATE_WARN)
                    marker = {"good": " **ok**", "warn": " *warn*", "fail": ""}[g]
                    ate_cells.append(f"{ate:.2f} ({sign}{delta_pct:.0f}%){marker}")
                else:
                    ate_cells.append("—")

            is_baseline_val = (float(val) == float(
                {"gyro_noise": 1e-3, "accel_noise": 1e-2, "gyro_bias_rw": 1e-5,
                 "accel_bias_rw": 1e-4, "voxel_resolution": 0.3,
                 "convergence_eps": 1e-3, "min_correspondences": 20,
                 "max_iterations": 5, "imu_integration_substeps": 1,
                 "plane_noise_sigma": 0.01}.get(param, 0)))
            marker = " **baseline**" if is_baseline_val else ""

            lines.append(f"| {fmt_sci(val)}{marker} | " +
                        " | ".join(ate_cells) +
                        f" | {drone_score:.2f} | {car_score:.2f} | {overall:.2f} |")

        best_drone[param] = best_d
        best_car[param]   = best_c

        lines.append(f"\n**Best for drone:** `{param}` = {fmt_sci(best_d[0])} (score {best_d[1]:.2f})")
        lines.append(f"**Best for car:** `{param}` = {fmt_sci(best_c[0])} (score {best_c[1]:.2f})\n")

    # ── Summary: optimal configs ────────────────────────────────────────
    lines.append("---\n")
    lines.append("## Recommended Optimal Configurations\n")

    lines.append("### Drone Profile (EuRoC-type scenarios)\n")
    lines.append("| Parameter | Baseline | Recommended | Delta |")
    lines.append("|---|---:|---:|---|")
    defaults_map = {"gyro_noise": 1e-3, "accel_noise": 1e-2, "gyro_bias_rw": 1e-5,
                    "accel_bias_rw": 1e-4, "voxel_resolution": 0.3,
                    "convergence_eps": 1e-3, "min_correspondences": 20,
                    "max_iterations": 5, "imu_integration_substeps": 1,
                    "plane_noise_sigma": 0.01}
    drone_cfg = {}
    for param in grouped:
        bval = defaults_map.get(param, 0)
        rval = float(best_drone[param][0])
        drone_cfg[param] = rval
        if rval != bval:
            lines.append(f"| `{param}` | {fmt_sci(bval)} | **{fmt_sci(rval)}** | changed |")
        else:
            lines.append(f"| `{param}` | {fmt_sci(bval)} | {fmt_sci(rval)} | — |")

    lines.append("\n### Car Profile (KITTI-type scenarios)\n")
    lines.append("| Parameter | Baseline | Recommended | Delta |")
    lines.append("|---|---:|---:|---|")
    car_cfg = {}
    for param in grouped:
        bval = defaults_map.get(param, 0)
        rval = float(best_car[param][0])
        car_cfg[param] = rval
        if rval != bval:
            lines.append(f"| `{param}` | {fmt_sci(bval)} | **{fmt_sci(rval)}** | changed |")
        else:
            lines.append(f"| `{param}` | {fmt_sci(bval)} | {fmt_sci(rval)} | — |")

    # ── Cross-validation warning ────────────────────────────────────────
    lines.append("\n### Cross-validation note\n")
    lines.append("> These recommendations are derived from one-at-a-time sweeps on")
    lines.append("> synthetic stress-test data. Parameter interactions are NOT captured.")
    lines.append("> Validate on real KITTI/EuRoC sequences before deploying.")
    lines.append("> Avoid overfitting: if a parameter only improves one preset while")
    lines.append("> degrading others, keep the baseline value.\n")

    # ── Write report ────────────────────────────────────────────────────
    report_path = os.path.join(output_dir, "SENSITIVITY_REPORT.md")
    with open(report_path, "w") as f:
        f.write("\n".join(lines))
    print(f"Wrote report: {report_path}")

    # ── Write recommended YAML configs ──────────────────────────────────
    for profile, cfg, platform_name in [
        ("drone_tuned", drone_cfg, "Drone"),
        ("car_tuned", car_cfg, "Car"),
    ]:
        yaml_path = os.path.join(output_dir, f"{profile}.yaml")
        with open(yaml_path, "w") as f:
            f.write(f"# Thunderbird SDK — {platform_name} Tuned Config\n")
            f.write(f"# Generated by sensitivity_sweep analysis\n")
            f.write(f"# Optimised for {'EuRoC-type' if profile.startswith('drone') else 'KITTI-type'} scenarios\n\n")
            for k, v in cfg.items():
                if isinstance(v, float) and v == int(v) and abs(v) < 1e6:
                    f.write(f"{k}: {int(v)}\n")
                else:
                    f.write(f"{k}: {v}\n")
        print(f"Wrote config: {yaml_path}")


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else r"C:\data\github\thunderbird\build\sensitivity\sensitivity_results.csv"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(csv_path)
    analyse(csv_path, output_dir)

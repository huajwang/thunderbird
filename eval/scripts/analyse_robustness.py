#!/usr/bin/env python3
"""
Thunderbird SLAM — Robustness Sweep Analyser
Reads robustness_results.csv and produces:
  1. Per-fault-mode degradation tables (markdown)
  2. Failure threshold identification
  3. Combined-fault resilience grading
  4. ROBUSTNESS_REPORT.md
"""
import csv
import sys
import os
import math
from collections import defaultdict

# ── Thresholds ──────────────────────────────────────────────────────────────
# Drift increase relative to baseline (percentage points)
DRIFT_WARN_DELTA = 2.0    # warn if drift increases >2 pp
DRIFT_FAIL_DELTA = 10.0   # fail if drift increases >10 pp
ATE_WARN_MULT  = 1.5      # warn if ATE increases >50%
ATE_FAIL_MULT  = 3.0      # fail if ATE increases >200%
FRAME_LOSS_WARN = 0.10    # warn if >10% poses lost
FRAME_LOSS_FAIL = 0.50    # fail if >50% poses lost

# Presets grouped by platform
DRONE_PRESETS = {"aggressive_drone", "spinning_top"}
CAR_PRESETS   = {"fast_car", "degenerate_corridor"}
ALL_PRESETS   = sorted(DRONE_PRESETS | CAR_PRESETS)

# Human-friendly fault mode names
FAULT_NAMES = {
    "lidar_drop": "LiDAR Frame Drop",
    "imu_noise":  "IMU Additive Noise",
    "ts_jitter":  "Timestamp Jitter",
    "rate_div":   "LiDAR Rate Reduction",
}

JITTER_LABELS = {
    "0":        "0",
    "1000":     "1 µs",
    "10000":    "10 µs",
    "100000":   "100 µs",
    "500000":   "500 µs",
    "1000000":  "1 ms",
    "5000000":  "5 ms",
    "10000000": "10 ms",
}

def fmt_jitter(val_str):
    return JITTER_LABELS.get(val_str, f"{float(val_str)/1e6:.1f} ms")

def fmt_pct(v):
    return f"{v:.2f}%"

def fmt_m(v):
    return f"{v:.3f} m"

def fmt_ms(v):
    return f"{v:.2f} ms"

def grade_ate(ate, baseline_ate):
    if baseline_ate == 0:
        return "pass"
    ratio = ate / baseline_ate
    if ratio <= ATE_WARN_MULT:
        return "pass"
    elif ratio <= ATE_FAIL_MULT:
        return "warn"
    return "FAIL"

def grade_drift(drift, baseline_drift):
    delta = drift - baseline_drift
    if delta <= DRIFT_WARN_DELTA:
        return "pass"
    elif delta <= DRIFT_FAIL_DELTA:
        return "warn"
    return "FAIL"

def grade_poses(poses, baseline_poses):
    if baseline_poses == 0:
        return "pass"
    loss = 1.0 - poses / baseline_poses
    if loss <= FRAME_LOSS_WARN:
        return "pass"
    elif loss <= FRAME_LOSS_FAIL:
        return "warn"
    return "FAIL"

def overall_grade(*grades):
    if "FAIL" in grades:
        return "FAIL"
    if "warn" in grades:
        return "warn"
    return "pass"

GRADE_EMOJI = {"pass": "✅", "warn": "⚠️", "FAIL": "❌"}


def load_results(csv_path):
    rows = []
    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for r in reader:
            for k in ["ate_rmse", "ate_max", "rpe_trans", "rpe_rot",
                       "drift_pct", "drift_100m", "rt_avg", "rt_p95", "rt_max",
                       "num_poses", "distance", "duration",
                       "fault_drop", "fault_noise", "fault_jitter", "fault_div"]:
                try:
                    r[k] = float(r.get(k, 0) or 0)
                except (ValueError, TypeError):
                    r[k] = float('nan')
            rows.append(r)
    return rows


def extract_baselines(rows):
    """Extract baseline (no-fault) results per preset."""
    baselines = {}
    for r in rows:
        mode = r["sweep_mode"]
        # Baseline rows: zero-fault values
        is_baseline = False
        if mode == "lidar_drop" and r["fault_drop"] == 0.0:
            is_baseline = True
        elif mode == "imu_noise" and r["fault_noise"] == 0.0:
            is_baseline = True
        elif mode == "ts_jitter" and r["fault_jitter"] == 0.0:
            is_baseline = True
        elif mode == "rate_div" and r["fault_div"] == 1.0:
            is_baseline = True

        if is_baseline:
            preset = r["preset"]
            if preset not in baselines:
                baselines[preset] = r
    return baselines


def find_threshold(rows, mode, preset, baseline, metric, limit_mult=None, limit_delta=None):
    """Find the lowest fault value where a metric exceeds a threshold."""
    mode_rows = [r for r in rows if r["sweep_mode"] == mode and r["preset"] == preset]
    base_val = baseline.get(metric, 0)

    for r in sorted(mode_rows, key=lambda x: x["fault_drop"] if mode == "lidar_drop"
                    else x["fault_noise"] if mode == "imu_noise"
                    else x["fault_jitter"] if mode == "ts_jitter"
                    else x["fault_div"]):
        val = r.get(metric, 0)
        if math.isnan(val):
            return r["sweep_value"]
        if limit_mult and base_val > 0 and val / base_val > limit_mult:
            return r["sweep_value"]
        if limit_delta and val - base_val > limit_delta:
            return r["sweep_value"]
    return None


def generate_report(rows, baselines, output_dir):
    """Generate ROBUSTNESS_REPORT.md."""
    lines = []
    L = lines.append

    L("# Thunderbird SLAM — Robustness Evaluation Report\n")
    L("## Overview\n")
    L("This report evaluates the SLAM system's resilience to simulated hardware")
    L("faults. Four fault modes are tested independently, plus five combined-fault")
    L("scenarios of increasing severity.\n")
    L("**Fault Modes:**")
    L("| Mode | Description | Sweep Range |")
    L("|------|-------------|-------------|")
    L("| LiDAR Frame Drop | Random frame loss | 0–50% |")
    L("| IMU Additive Noise | Scaled Gaussian noise | 0–20× baseline σ |")
    L("| Timestamp Jitter | Uniform ± offset | 0–10 ms |")
    L("| LiDAR Rate Reduction | Keep every Nth frame | 1× – 8× decimation |")
    L("")

    # ── Per-mode tables ─────────────────────────────────────────────────
    L("---\n## Single-Fault Degradation\n")

    for mode in ["lidar_drop", "imu_noise", "ts_jitter", "rate_div"]:
        mode_name = FAULT_NAMES[mode]
        L(f"### {mode_name}\n")

        # Determine value column label and sort key
        if mode == "lidar_drop":
            val_label = "Drop Rate"
            sort_key = "fault_drop"
            fmt_val = lambda r: f"{r['fault_drop']:.0%}"
        elif mode == "imu_noise":
            val_label = "Noise Scale"
            sort_key = "fault_noise"
            fmt_val = lambda r: f"{r['fault_noise']:.1f}×"
        elif mode == "ts_jitter":
            val_label = "Jitter"
            sort_key = "fault_jitter"
            fmt_val = lambda r: fmt_jitter(r["sweep_value"])
        else:
            val_label = "Rate Divisor"
            sort_key = "fault_div"
            fmt_val = lambda r: f"1/{int(r['fault_div'])}"

        L(f"| {val_label} | Preset | ATE RMSE | Δ ATE | Drift | Δ Drift | Poses | Avg RT | Grade |")
        L("|" + "---|" * 9)

        mode_rows = [r for r in rows if r["sweep_mode"] == mode]
        mode_rows.sort(key=lambda r: (r[sort_key], r["preset"]))

        for r in mode_rows:
            preset = r["preset"]
            bl = baselines.get(preset, {})
            bl_ate = bl.get("ate_rmse", 0) or 0
            bl_drift = bl.get("drift_pct", 0) or 0
            bl_poses = bl.get("num_poses", 0) or 0

            ate = r.get("ate_rmse", 0)
            drift = r.get("drift_pct", 0)
            poses = r.get("num_poses", 0)
            rt = r.get("rt_avg", 0)

            if math.isnan(ate):
                L(f"| {fmt_val(r)} | {preset} | FAIL | — | — | — | — | — | ❌ |")
                continue

            d_ate = ate - bl_ate if bl_ate else 0
            d_drift = drift - bl_drift if not math.isnan(bl_drift) else 0

            g_ate = grade_ate(ate, bl_ate)
            g_drift = grade_drift(drift, bl_drift)
            g_poses = grade_poses(poses, bl_poses)
            g = overall_grade(g_ate, g_drift, g_poses)

            L(f"| {fmt_val(r)} | {preset} | {fmt_m(ate)} | {'+' if d_ate >= 0 else ''}{fmt_m(d_ate)} | {fmt_pct(drift)} | {'+' if d_drift >= 0 else ''}{fmt_pct(d_drift)} | {int(poses)} | {fmt_ms(rt)} | {GRADE_EMOJI[g]} |")

        L("")

    # ── Combined faults ─────────────────────────────────────────────────
    L("---\n## Combined-Fault Scenarios\n")
    L("| Scenario | Preset | Drop | Noise | Jitter | Div | ATE RMSE | Drift | Poses | Grade |")
    L("|" + "---|" * 10)

    combined_rows = [r for r in rows if r["sweep_mode"].startswith("combined_")]
    # Sort by scenario severity order
    severity_order = {"mild": 0, "moderate": 1, "harsh": 2, "extreme": 3, "nightmare": 4}
    combined_rows.sort(key=lambda r: (severity_order.get(r["sweep_value"], 99), r["preset"]))

    for r in combined_rows:
        preset = r["preset"]
        bl = baselines.get(preset, {})
        bl_ate = bl.get("ate_rmse", 0) or 0
        bl_drift = bl.get("drift_pct", 0) or 0
        bl_poses = bl.get("num_poses", 0) or 0

        ate = r.get("ate_rmse", 0)
        drift = r.get("drift_pct", 0)
        poses = r.get("num_poses", 0)

        if math.isnan(ate):
            g = "FAIL"
        else:
            g_ate = grade_ate(ate, bl_ate)
            g_drift = grade_drift(drift, bl_drift)
            g_poses = grade_poses(poses, bl_poses)
            g = overall_grade(g_ate, g_drift, g_poses)

        label = r["sweep_value"]
        drop_s = f"{r['fault_drop']:.0%}"
        noise_s = f"{r['fault_noise']:.0f}×"
        jitter_s = fmt_jitter(str(int(r['fault_jitter'])))
        div_s = f"1/{int(r['fault_div'])}"

        ate_s = fmt_m(ate) if not math.isnan(ate) else "FAIL"
        drift_s = fmt_pct(drift) if not math.isnan(drift) else "—"

        L(f"| {label} | {preset} | {drop_s} | {noise_s} | {jitter_s} | {div_s} | {ate_s} | {drift_s} | {int(poses)} | {GRADE_EMOJI[g]} |")

    L("")

    # ── Failure thresholds ──────────────────────────────────────────────
    L("---\n## Failure Thresholds\n")
    L("Failure is defined as ATE increasing > 3× baseline **or** drift increasing\n"
      "> 10 percentage points **or** > 50% pose loss.\n")
    L("| Fault Mode | Preset | ATE 3× Threshold | Drift +10pp Threshold |")
    L("|" + "---|" * 4)

    for mode in ["lidar_drop", "imu_noise", "ts_jitter", "rate_div"]:
        mode_name = FAULT_NAMES[mode]
        for preset in ALL_PRESETS:
            bl = baselines.get(preset, {})
            t_ate = find_threshold(rows, mode, preset, bl, "ate_rmse", limit_mult=ATE_FAIL_MULT)
            t_drift = find_threshold(rows, mode, preset, bl, "drift_pct", limit_delta=DRIFT_FAIL_DELTA)
            t_ate_s = str(t_ate) if t_ate else "> max tested"
            t_drift_s = str(t_drift) if t_drift else "> max tested"
            L(f"| {mode_name} | {preset} | {t_ate_s} | {t_drift_s} |")

    L("")

    # ── Summary statistics ──────────────────────────────────────────────
    L("---\n## Summary Statistics\n")

    total = len(rows)
    non_combined = [r for r in rows if not r["sweep_mode"].startswith("combined_")]
    combined = [r for r in rows if r["sweep_mode"].startswith("combined_")]

    # Count grades
    grade_counts = {"pass": 0, "warn": 0, "FAIL": 0}
    for r in rows:
        preset = r["preset"]
        bl = baselines.get(preset, {})
        bl_ate = bl.get("ate_rmse", 0) or 0
        bl_drift = bl.get("drift_pct", 0) or 0
        bl_poses = bl.get("num_poses", 0) or 0

        ate = r.get("ate_rmse", 0)
        drift = r.get("drift_pct", 0)
        poses = r.get("num_poses", 0)

        if math.isnan(ate):
            grade_counts["FAIL"] += 1
            continue

        g_ate = grade_ate(ate, bl_ate)
        g_drift = grade_drift(drift, bl_drift)
        g_poses = grade_poses(poses, bl_poses)
        g = overall_grade(g_ate, g_drift, g_poses)
        grade_counts[g] += 1

    L(f"- **Total runs:** {total}")
    L(f"  - Single-fault: {len(non_combined)}")
    L(f"  - Combined-fault: {len(combined)}")
    L(f"- **Pass:** {grade_counts['pass']}  ({100*grade_counts['pass']/total:.0f}%)")
    L(f"- **Warn:** {grade_counts['warn']}  ({100*grade_counts['warn']/total:.0f}%)")
    L(f"- **Fail:** {grade_counts['FAIL']}  ({100*grade_counts['FAIL']/total:.0f}%)")
    L("")

    # ── Per-platform resilience summary ─────────────────────────────────
    L("### Per-Platform Resilience\n")

    for platform, presets in [("Drone", DRONE_PRESETS), ("Car", CAR_PRESETS)]:
        plat_rows = [r for r in rows if r["preset"] in presets]
        plat_pass = 0
        plat_total = len(plat_rows)
        for r in plat_rows:
            preset = r["preset"]
            bl = baselines.get(preset, {})
            bl_ate = bl.get("ate_rmse", 0) or 0
            bl_drift = bl.get("drift_pct", 0) or 0
            bl_poses = bl.get("num_poses", 0) or 0
            ate = r.get("ate_rmse", 0)
            drift = r.get("drift_pct", 0)
            poses = r.get("num_poses", 0)
            if not math.isnan(ate):
                g_ate = grade_ate(ate, bl_ate)
                g_drift = grade_drift(drift, bl_drift)
                g_poses = grade_poses(poses, bl_poses)
                g = overall_grade(g_ate, g_drift, g_poses)
                if g == "pass":
                    plat_pass += 1

        pct = 100 * plat_pass / plat_total if plat_total else 0
        L(f"- **{platform}:** {plat_pass}/{plat_total} pass ({pct:.0f}%)")

    L("")

    # ── Recommendations ─────────────────────────────────────────────────
    L("---\n## Recommendations\n")
    L("Based on the robustness evaluation:\n")

    # Analyse which modes cause the most degradation
    max_ate_increase = defaultdict(lambda: 0)
    for r in non_combined:
        preset = r["preset"]
        bl = baselines.get(preset, {})
        bl_ate = bl.get("ate_rmse", 0) or 0
        if bl_ate > 0:
            ate = r.get("ate_rmse", 0)
            if not math.isnan(ate):
                increase = (ate - bl_ate) / bl_ate
                mode = r["sweep_mode"]
                if increase > max_ate_increase[mode]:
                    max_ate_increase[mode] = increase

    sorted_modes = sorted(max_ate_increase.items(), key=lambda x: -x[1])
    for mode, inc in sorted_modes:
        mode_name = FAULT_NAMES.get(mode, mode)
        if inc > 2.0:
            L(f"- 🔴 **{mode_name}**: Max ATE increase {inc:.0%} — critical vulnerability")
        elif inc > 0.5:
            L(f"- 🟡 **{mode_name}**: Max ATE increase {inc:.0%} — moderate sensitivity")
        else:
            L(f"- 🟢 **{mode_name}**: Max ATE increase {inc:.0%} — resilient")

    L("")
    L("---\n*Generated by analyse_robustness.py*\n")

    # ── Write report ────────────────────────────────────────────────────
    report_path = os.path.join(output_dir, "ROBUSTNESS_REPORT.md")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"Wrote report: {report_path}")
    return report_path


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <robustness_results.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    output_dir = os.path.dirname(csv_path) or "."

    rows = load_results(csv_path)
    print(f"Loaded {len(rows)} results from {csv_path}")

    baselines = extract_baselines(rows)
    print(f"Baselines for {len(baselines)} presets: {', '.join(sorted(baselines))}")

    report = generate_report(rows, baselines, output_dir)


if __name__ == "__main__":
    main()

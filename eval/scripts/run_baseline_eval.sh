#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Thunderbird SDK — Systematic SLAM Baseline Evaluation
# ─────────────────────────────────────────────────────────────────────────────
#
# Runs AcmeSlamEngine through KITTI and EuRoC sequences with both
# drone.yaml and car.yaml configurations to establish baseline metrics.
#
# Usage:
#   ./run_baseline_eval.sh [--kitti-dir /path/to/kitti] [--euroc-dir /path/to/euroc]
#
# Prerequisites:
#   - slam_eval binary built and on PATH (or in ../build/eval/)
#   - KITTI odometry dataset: sequences/00, 05, 07 + poses/
#   - EuRoC MAV dataset: MH_04_difficult, V2_03_difficult
#   - python3 with matplotlib + numpy (for plots)
#
# Output:
#   eval_baseline/
#     kitti_00_drone/   metrics.csv, summary.json, trajectory_overlay.png, ...
#     kitti_00_car/
#     ...
#     euroc_MH04_drone/
#     ...
#     baseline_summary.csv
#     baseline_report.txt
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

# ── Defaults ─────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
EVAL_BIN="${BUILD_DIR}/eval/slam_eval"
DRONE_CONFIG="${REPO_ROOT}/slamd/config/drone.yaml"
CAR_CONFIG="${REPO_ROOT}/slamd/config/car.yaml"
OUTPUT_ROOT="${REPO_ROOT}/eval_baseline"

KITTI_DIR=""
EUROC_DIR=""

# KITTI sequences to evaluate.
KITTI_SEQS=("00" "05" "07")

# EuRoC sequences to evaluate.
EUROC_SEQS=("MH_04_difficult" "V2_03_difficult")

# Config profiles.
CONFIGS=("drone" "car")
CONFIG_FILES=("${DRONE_CONFIG}" "${CAR_CONFIG}")

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --kitti-dir)  KITTI_DIR="$2";  shift 2 ;;
        --euroc-dir)  EUROC_DIR="$2";  shift 2 ;;
        --output)     OUTPUT_ROOT="$2"; shift 2 ;;
        --eval-bin)   EVAL_BIN="$2";   shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--kitti-dir DIR] [--euroc-dir DIR] [--output DIR] [--eval-bin PATH]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Validate ─────────────────────────────────────────────────────────────────
if [[ ! -x "$EVAL_BIN" ]]; then
    # Try common build locations.
    for candidate in \
        "${BUILD_DIR}/eval/slam_eval" \
        "${REPO_ROOT}/build_ver_test/eval/slam_eval" \
        "$(command -v slam_eval 2>/dev/null || true)"; do
        if [[ -x "$candidate" ]]; then
            EVAL_BIN="$candidate"
            break
        fi
    done
fi

if [[ ! -x "$EVAL_BIN" ]]; then
    echo "ERROR: slam_eval binary not found. Build with: cd build && cmake --build . --target slam_eval_cli"
    exit 1
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  Thunderbird SLAM — Baseline Evaluation Campaign"
echo "═══════════════════════════════════════════════════════════════"
echo "  Binary:  ${EVAL_BIN}"
echo "  Configs: ${CONFIGS[*]}"
echo "  Output:  ${OUTPUT_ROOT}"
echo "═══════════════════════════════════════════════════════════════"

mkdir -p "${OUTPUT_ROOT}"

# ── Summary CSV header ───────────────────────────────────────────────────────
SUMMARY_CSV="${OUTPUT_ROOT}/baseline_summary.csv"
echo "dataset,config,ate_rmse_m,ate_mean_m,ate_max_m,rpe_trans_rmse_m,rpe_rot_rmse_deg,drift_pct,drift_per_100m,runtime_avg_ms,runtime_p95_ms,cpu_avg_pct,peak_rss_mb,num_poses,distance_m,duration_s,status" \
    > "${SUMMARY_CSV}"

TOTAL=0
PASS=0
FAIL=0

# ── Helper: extract metric from CSV ─────────────────────────────────────────
extract_metric() {
    local csv="$1"
    local key="$2"
    if [[ -f "$csv" ]]; then
        grep "^${key}," "$csv" 2>/dev/null | head -1 | cut -d',' -f2 || echo "N/A"
    else
        echo "N/A"
    fi
}

# ── Helper: run one evaluation ───────────────────────────────────────────────
run_eval() {
    local dataset_path="$1"
    local format="$2"
    local config_file="$3"
    local config_name="$4"
    local dataset_name="$5"
    local output_dir="$6"

    TOTAL=$((TOTAL + 1))

    echo ""
    echo "──────────────────────────────────────────────────────────────"
    echo "  [${TOTAL}] ${dataset_name} + ${config_name}"
    echo "──────────────────────────────────────────────────────────────"

    local status="OK"
    local extra_flags=""

    # EuRoC has native IMU — don't use synth.
    # KITTI is LiDAR-only — use synth IMU from GT.
    if [[ "$format" == "kitti" ]]; then
        extra_flags="--synth-imu --imu-rate 200"
    fi

    mkdir -p "${output_dir}"

    # Run slam_eval.
    set +e
    "${EVAL_BIN}" \
        -f "${format}" \
        -c "${config_file}" \
        -o "${output_dir}" \
        --outlier-reject \
        ${extra_flags} \
        -v \
        "${dataset_path}" \
        2>"${output_dir}/eval_stderr.log"
    local rc=$?
    set -e

    if [[ $rc -ne 0 ]]; then
        echo "  ⚠ FAILED (exit code ${rc})"
        status="FAIL:rc${rc}"
        FAIL=$((FAIL + 1))
    else
        PASS=$((PASS + 1))
    fi

    # Extract metrics from CSV.
    local csv="${output_dir}/metrics.csv"
    local ate_rmse=$(extract_metric "$csv" "ate_rmse")
    local ate_mean=$(extract_metric "$csv" "ate_mean")
    local ate_max=$(extract_metric "$csv" "ate_max")
    local rpe_trans=$(extract_metric "$csv" "rpe_trans_rmse")
    local rpe_rot=$(extract_metric "$csv" "rpe_rot_rmse")
    local drift_pct=$(extract_metric "$csv" "drift_pct")
    local drift_100m=$(extract_metric "$csv" "drift_per_100m")
    local rt_avg=$(extract_metric "$csv" "runtime_avg")
    local rt_p95=$(extract_metric "$csv" "runtime_p95")
    local cpu_avg=$(extract_metric "$csv" "cpu_avg")
    local peak_rss=$(extract_metric "$csv" "peak_rss")
    local num_poses=$(extract_metric "$csv" "num_poses")
    local distance=$(extract_metric "$csv" "distance")
    local duration=$(extract_metric "$csv" "duration")

    # Convert peak RSS from bytes to MB.
    if [[ "$peak_rss" != "N/A" && "$peak_rss" != "0" ]]; then
        peak_rss_mb=$(echo "scale=1; ${peak_rss} / 1048576" | bc 2>/dev/null || echo "$peak_rss")
    else
        peak_rss_mb="$peak_rss"
    fi

    # Append to summary CSV.
    echo "${dataset_name},${config_name},${ate_rmse},${ate_mean},${ate_max},${rpe_trans},${rpe_rot},${drift_pct},${drift_100m},${rt_avg},${rt_p95},${cpu_avg},${peak_rss_mb},${num_poses},${distance},${duration},${status}" \
        >> "${SUMMARY_CSV}"

    echo "  ATE RMSE: ${ate_rmse} m | RPE: ${rpe_trans} m | Drift: ${drift_100m} m/100m | Avg: ${rt_avg} ms"
}

# ═════════════════════════════════════════════════════════════════════════════
#  KITTI sequences
# ═════════════════════════════════════════════════════════════════════════════

if [[ -n "$KITTI_DIR" && -d "$KITTI_DIR" ]]; then
    echo ""
    echo "▶ KITTI Odometry Benchmark"
    for seq in "${KITTI_SEQS[@]}"; do
        seq_path="${KITTI_DIR}/sequences/${seq}"
        if [[ ! -d "$seq_path" ]]; then
            echo "  SKIP: ${seq_path} not found"
            continue
        fi
        for i in "${!CONFIGS[@]}"; do
            run_eval \
                "$seq_path" \
                "kitti" \
                "${CONFIG_FILES[$i]}" \
                "${CONFIGS[$i]}" \
                "kitti_${seq}" \
                "${OUTPUT_ROOT}/kitti_${seq}_${CONFIGS[$i]}"
        done
    done
else
    echo ""
    echo "⚠ KITTI directory not specified or not found. Use --kitti-dir <path>"
    echo "  Expected structure: <kitti-dir>/sequences/00/velodyne/ + poses/00.txt"
fi

# ═════════════════════════════════════════════════════════════════════════════
#  EuRoC sequences
# ═════════════════════════════════════════════════════════════════════════════

if [[ -n "$EUROC_DIR" && -d "$EUROC_DIR" ]]; then
    echo ""
    echo "▶ EuRoC MAV Benchmark"
    for seq in "${EUROC_SEQS[@]}"; do
        seq_path="${EUROC_DIR}/${seq}"
        if [[ ! -d "$seq_path" ]]; then
            echo "  SKIP: ${seq_path} not found"
            continue
        fi
        for i in "${!CONFIGS[@]}"; do
            run_eval \
                "$seq_path" \
                "euroc" \
                "${CONFIG_FILES[$i]}" \
                "${CONFIGS[$i]}" \
                "euroc_${seq}" \
                "${OUTPUT_ROOT}/euroc_${seq}_${CONFIGS[$i]}"
        done
    done
else
    echo ""
    echo "⚠ EuRoC directory not specified or not found. Use --euroc-dir <path>"
    echo "  Expected structure: <euroc-dir>/MH_04_difficult/mav0/imu0/data.csv"
fi

# ═════════════════════════════════════════════════════════════════════════════
#  Generate consolidated report
# ═════════════════════════════════════════════════════════════════════════════

REPORT="${OUTPUT_ROOT}/baseline_report.txt"

cat > "${REPORT}" << 'HEADER'
═══════════════════════════════════════════════════════════════════════════════
  Thunderbird SLAM — Baseline Performance Report
═══════════════════════════════════════════════════════════════════════════════
HEADER

echo "" >> "${REPORT}"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M:%S UTC')" >> "${REPORT}"
echo "Runs: ${TOTAL} total, ${PASS} passed, ${FAIL} failed" >> "${REPORT}"
echo "" >> "${REPORT}"

# Pretty-print the CSV as a table.
if command -v column &>/dev/null && [[ -f "${SUMMARY_CSV}" ]]; then
    echo "── Summary Table ─────────────────────────────────────────────────" >> "${REPORT}"
    echo "" >> "${REPORT}"
    column -t -s',' < "${SUMMARY_CSV}" >> "${REPORT}"
    echo "" >> "${REPORT}"
else
    cat "${SUMMARY_CSV}" >> "${REPORT}"
fi

# ── Analysis: identify weak spots ────────────────────────────────────────────
echo "── Weak Spot Analysis ────────────────────────────────────────────────" >> "${REPORT}"
echo "" >> "${REPORT}"

python3 - "${SUMMARY_CSV}" >> "${REPORT}" 2>/dev/null << 'PYEOF' || echo "(python3 analysis unavailable)" >> "${REPORT}"
import csv, sys

rows = []
with open(sys.argv[1]) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

if not rows:
    print("  No results to analyse.")
    sys.exit(0)

# Thresholds for concern flags.
ATE_WARN   = 2.0    # m
DRIFT_WARN = 2.0    # m/100m
RT_WARN    = 50.0   # ms/frame
RPE_WARN   = 0.5    # m

print("  Flags: ⚠ = borderline,  ✗ = poor,  ✓ = acceptable\n")

for r in rows:
    ds = r['dataset']
    cfg = r['config']
    status = r['status']

    if status != 'OK':
        print(f"  {ds:20s} [{cfg:5s}]  ✗  EVALUATION FAILED ({status})")
        continue

    flags = []
    try:
        ate = float(r['ate_rmse_m'])
        if ate > ATE_WARN * 2:   flags.append(f"✗ ATE RMSE={ate:.3f}m (very high)")
        elif ate > ATE_WARN:     flags.append(f"⚠ ATE RMSE={ate:.3f}m")
    except: pass

    try:
        drift = float(r['drift_per_100m'])
        if drift > DRIFT_WARN * 2: flags.append(f"✗ Drift={drift:.3f}m/100m (unstable)")
        elif drift > DRIFT_WARN:   flags.append(f"⚠ Drift={drift:.3f}m/100m")
    except: pass

    try:
        rpe = float(r['rpe_trans_rmse_m'])
        if rpe > RPE_WARN * 2: flags.append(f"✗ RPE={rpe:.3f}m")
        elif rpe > RPE_WARN:   flags.append(f"⚠ RPE={rpe:.3f}m")
    except: pass

    try:
        rt = float(r['runtime_avg_ms'])
        if rt > RT_WARN * 2:  flags.append(f"✗ Runtime={rt:.1f}ms (real-time violation)")
        elif rt > RT_WARN:    flags.append(f"⚠ Runtime={rt:.1f}ms (approaching limit)")
    except: pass

    if not flags:
        print(f"  {ds:20s} [{cfg:5s}]  ✓  All metrics within tolerance")
    else:
        for i, flag in enumerate(flags):
            prefix = f"  {ds:20s} [{cfg:5s}]" if i == 0 else " " * 30
            print(f"{prefix}  {flag}")

# ── Config comparison ────────────────────────────────────────────────────
print("\n── Drone vs Car Config Comparison ──────────────────────────────────\n")

# Group by dataset.
datasets = {}
for r in rows:
    ds = r['dataset']
    cfg = r['config']
    if ds not in datasets:
        datasets[ds] = {}
    datasets[ds][cfg] = r

for ds in sorted(datasets):
    if 'drone' in datasets[ds] and 'car' in datasets[ds]:
        d = datasets[ds]['drone']
        c = datasets[ds]['car']

        try:
            d_ate = float(d['ate_rmse_m'])
            c_ate = float(c['ate_rmse_m'])
            d_drift = float(d['drift_per_100m'])
            c_drift = float(c['drift_per_100m'])
            d_rt = float(d['runtime_avg_ms'])
            c_rt = float(c['runtime_avg_ms'])

            ate_winner = "drone" if d_ate < c_ate else "car"
            drift_winner = "drone" if d_drift < c_drift else "car"
            rt_winner = "drone" if d_rt < c_rt else "car"

            print(f"  {ds}:")
            print(f"    ATE RMSE:  drone={d_ate:.4f}m  car={c_ate:.4f}m  → {ate_winner} wins")
            print(f"    Drift:     drone={d_drift:.4f}  car={c_drift:.4f}  → {drift_winner} wins")
            print(f"    Runtime:   drone={d_rt:.1f}ms   car={c_rt:.1f}ms   → {rt_winner} wins")
            print()
        except:
            print(f"  {ds}: comparison unavailable (missing metrics)")
            print()

PYEOF

echo "" >> "${REPORT}"
echo "═══════════════════════════════════════════════════════════════════════════════" >> "${REPORT}"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  Evaluation complete: ${PASS}/${TOTAL} passed"
echo "  Summary CSV:  ${SUMMARY_CSV}"
echo "  Report:       ${REPORT}"
echo "═══════════════════════════════════════════════════════════════"

exit ${FAIL}

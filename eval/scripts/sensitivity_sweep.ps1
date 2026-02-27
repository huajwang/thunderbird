<#
.SYNOPSIS
    Thunderbird SLAM — Parameter Sensitivity Sweep
.DESCRIPTION
    Runs one-at-a-time parameter sweeps across all stress-test presets.
    Produces a consolidated CSV for analysis.
.NOTES
    Reproducible: deterministic stress presets (fixed seeds), config-driven.
#>
param(
    [string]$EvalBin   = "C:\data\github\thunderbird\build\eval\slam_eval.exe",
    [string]$OutputDir = "C:\data\github\thunderbird\build\sensitivity",
    [string]$SdkDll    = "C:\data\github\thunderbird\build\sdk"
)

# ── Ensure SDK DLL is in PATH ───────────────────────────────────────────────
if ($env:PATH -notlike "*$SdkDll*") {
    $env:PATH = "$SdkDll;$env:PATH"
}

# ── Stress presets (proxy for KITTI ≈ fast_car, EuRoC ≈ aggressive_drone) ───
$PRESETS = @("aggressive_drone", "fast_car", "degenerate_corridor", "spinning_top")

# ── Engine compiled defaults (centre point of each sweep) ───────────────────
$DEFAULTS = @{
    gyro_noise               = 1e-3
    accel_noise              = 1e-2
    gyro_bias_rw             = 1e-5
    accel_bias_rw            = 1e-4
    voxel_resolution         = 0.3
    convergence_eps          = 1e-3
    min_correspondences      = 20
    max_iterations           = 5
    imu_integration_substeps = 1
    plane_noise_sigma        = 0.01
    max_residual             = 0.5
    map_radius               = 100.0
    max_map_points           = 500000
}

# ── Sweep definitions: parameter → list of values to try ────────────────────
# Dimension 1: IMU noise covariance
# Dimension 2: Voxel filter resolution
# Dimension 3: Keyframe selection threshold (convergence_eps, min_correspondences)
# Dimension 4: Motion model constraints (max_iterations, substeps, plane_noise)
$SWEEPS = [ordered]@{
    # ── Dim 1: IMU noise ─────────────────────────────────────────────────
    gyro_noise               = @(2e-4, 5e-4, 1e-3, 2e-3, 5e-3, 1e-2)
    accel_noise              = @(1e-3, 5e-3, 1e-2, 2e-2, 5e-2, 1e-1)
    gyro_bias_rw             = @(1e-7, 1e-6, 1e-5, 1e-4, 1e-3)
    accel_bias_rw            = @(1e-6, 1e-5, 1e-4, 1e-3, 1e-2)
    # ── Dim 2: Voxel resolution ──────────────────────────────────────────
    voxel_resolution         = @(0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1.0)
    # ── Dim 3: Keyframe / convergence ────────────────────────────────────
    convergence_eps          = @(1e-5, 1e-4, 1e-3, 1e-2, 1e-1)
    min_correspondences      = @(5, 10, 20, 50, 100)
    # ── Dim 4: Motion model ──────────────────────────────────────────────
    max_iterations           = @(1, 2, 5, 10, 15, 20)
    imu_integration_substeps = @(1, 2, 4, 8)
    plane_noise_sigma        = @(0.001, 0.005, 0.01, 0.05, 0.1)
}

# ── Helper: write a flat YAML config file ────────────────────────────────────
function Write-SweepConfig {
    param([string]$Path, [hashtable]$Overrides)
    $lines = @("# Auto-generated sensitivity sweep config")
    foreach ($kv in $DEFAULTS.GetEnumerator()) {
        $key = $kv.Key
        $val = $kv.Value
        if ($Overrides.ContainsKey($key)) { $val = $Overrides[$key] }
        $lines += "${key}: $val"
    }
    $lines | Out-File -FilePath $Path -Encoding utf8
}

# ── Results accumulator ─────────────────────────────────────────────────────
$csvHeader = "sweep_param,sweep_value,preset,ate_rmse,ate_max,rpe_trans,drift_pct,drift_100m,rt_avg,rt_p95,rt_max,num_poses,distance"
$allRows   = [System.Collections.Generic.List[string]]::new()

# ── Create output directory ─────────────────────────────────────────────────
New-Item -ItemType Directory -Force -Path $OutputDir          | Out-Null
New-Item -ItemType Directory -Force -Path "$OutputDir\configs" | Out-Null
New-Item -ItemType Directory -Force -Path "$OutputDir\runs"    | Out-Null

# ── 1. Baseline runs (engine defaults, no config override) ──────────────────
Write-Host "════════════════════════════════════════════════════════════════"
Write-Host "  Phase 1: Baseline (engine defaults)"
Write-Host "════════════════════════════════════════════════════════════════"

$baseYaml = "$OutputDir\configs\baseline.yaml"
Write-SweepConfig -Path $baseYaml -Overrides @{}

foreach ($preset in $PRESETS) {
    $runDir = "$OutputDir\runs\baseline_$preset"
    New-Item -ItemType Directory -Force -Path $runDir | Out-Null
    & $EvalBin --stress $preset -c $baseYaml -o $runDir --no-plot --csv-only 2>$null
    $csvPath = "$runDir\metrics.csv"
    if (Test-Path $csvPath) {
        $m = Import-Csv $csvPath
        $row = "baseline,0,$preset"
        foreach ($metric in @("ate_rmse","ate_max","rpe_trans_rmse","drift_pct","drift_per_100m","runtime_avg","runtime_p95","runtime_max","num_poses","distance")) {
            $v = ($m | Where-Object { $_.metric -eq $metric }).value
            if (-not $v) { $v = "0" }
            $row += ",$v"
        }
        $allRows.Add($row)
        $ate = ($m | Where-Object { $_.metric -eq "ate_rmse" }).value
        Write-Host "  baseline / $preset : ATE=$ate"
    }
}

# ── 2. Parameter sweeps ─────────────────────────────────────────────────────
$totalSweeps = 0
foreach ($s in $SWEEPS.GetEnumerator()) { $totalSweeps += $s.Value.Count }
$totalRuns = $totalSweeps * $PRESETS.Count
$runIdx = 0

Write-Host "════════════════════════════════════════════════════════════════"
Write-Host "  Phase 2: Parameter sweeps ($totalSweeps points × $($PRESETS.Count) presets = $totalRuns runs)"
Write-Host "════════════════════════════════════════════════════════════════"

$sw = [System.Diagnostics.Stopwatch]::StartNew()

foreach ($sweep in $SWEEPS.GetEnumerator()) {
    $param = $sweep.Key
    $values = $sweep.Value

    Write-Host "`n── Sweeping: $param ──────────────────────────────────"

    foreach ($val in $values) {
        # Tag for filenames (safe for paths)
        $valTag = "$val" -replace '[^a-zA-Z0-9._\-]', ''

        # Write config with this one parameter overridden
        $cfgPath = "$OutputDir\configs\${param}_${valTag}.yaml"
        Write-SweepConfig -Path $cfgPath -Overrides @{ $param = $val }

        foreach ($preset in $PRESETS) {
            $runIdx++
            $runDir = "$OutputDir\runs\${param}_${valTag}_$preset"
            New-Item -ItemType Directory -Force -Path $runDir | Out-Null

            & $EvalBin --stress $preset -c $cfgPath -o $runDir --no-plot --csv-only 2>$null

            $csvPath = "$runDir\metrics.csv"
            if (Test-Path $csvPath) {
                $m = Import-Csv $csvPath
                $row = "$param,$val,$preset"
                foreach ($metric in @("ate_rmse","ate_max","rpe_trans_rmse","drift_pct","drift_per_100m","runtime_avg","runtime_p95","runtime_max","num_poses","distance")) {
                    $v = ($m | Where-Object { $_.metric -eq $metric }).value
                    if (-not $v) { $v = "0" }
                    $row += ",$v"
                }
                $allRows.Add($row)
            }

            $pct = [math]::Round(100 * $runIdx / $totalRuns, 1)
            $elapsed = $sw.Elapsed
            $eta = if ($runIdx -gt 0) {
                [TimeSpan]::FromTicks($elapsed.Ticks / $runIdx * ($totalRuns - $runIdx))
            } else { [TimeSpan]::Zero }
            Write-Host ("  [{0,5:F1}%] {1}={2} / {3}  ETA {4:mm\:ss}" -f $pct, $param, $val, $preset, $eta)
        }
    }
}

$sw.Stop()

# ── 3. Write consolidated CSV ───────────────────────────────────────────────
$csvOut = "$OutputDir\sensitivity_results.csv"
$csvHeader | Out-File -FilePath $csvOut -Encoding utf8
$allRows | Out-File -FilePath $csvOut -Encoding utf8 -Append

Write-Host "`n════════════════════════════════════════════════════════════════"
Write-Host "  Done: $($allRows.Count) results in $($sw.Elapsed.ToString('mm\:ss'))"
Write-Host "  CSV:  $csvOut"
Write-Host "════════════════════════════════════════════════════════════════"

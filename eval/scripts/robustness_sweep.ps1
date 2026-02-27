<#
.SYNOPSIS
    Thunderbird SLAM — Robustness / Fault Injection Sweep
.DESCRIPTION
    Systematically tests SLAM system resilience by sweeping individual fault
    modes (LiDAR drop, IMU noise, timestamp jitter, LiDAR rate reduction)
    and combined faults across all stress-test presets.
    Produces robustness_results.csv for analysis.
.NOTES
    Requires fault injection support in slam_eval (--fault-* flags).
    Deterministic: fixed RNG seed (42) for all fault injection.
#>
param(
    [string]$EvalBin   = "C:\data\github\thunderbird\build\eval\slam_eval.exe",
    [string]$OutputDir = "C:\data\github\thunderbird\build\robustness",
    [string]$SdkDll    = "C:\data\github\thunderbird\build\sdk"
)

# ── Ensure SDK DLL is in PATH ───────────────────────────────────────────────
if ($env:PATH -notlike "*$SdkDll*") {
    $env:PATH = "$SdkDll;$env:PATH"
}

# ── Stress presets ──────────────────────────────────────────────────────────
$PRESETS = @("aggressive_drone", "fast_car", "degenerate_corridor", "spinning_top")

# ── Fault sweep definitions ─────────────────────────────────────────────────
# Each fault mode is swept independently (others set to 0 / off).
# Then a combined-faults section tests realistic multi-fault scenarios.
$FAULT_SWEEPS = [ordered]@{
    # Fault mode 1: LiDAR frame drops (random)
    lidar_drop = @(0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50)
    # Fault mode 2: Additive IMU noise (multiplier of baseline sigma)
    imu_noise  = @(0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0)
    # Fault mode 3: Timestamp jitter (nanoseconds, ±uniform)
    ts_jitter  = @(0, 1000, 10000, 100000, 500000, 1000000, 5000000, 10000000)
    # Fault mode 4: LiDAR rate decimation (divisor: 1=off, 2=50%, 4=25%)
    rate_div   = @(1, 2, 3, 4, 6, 8)
}

# Combined fault scenarios (realistic degraded hardware)
$COMBINED_FAULTS = @(
    @{ label="mild";      drop=0.05; noise=1.0;  jitter=10000;   div=1 }
    @{ label="moderate";  drop=0.10; noise=2.0;  jitter=100000;  div=2 }
    @{ label="harsh";     drop=0.20; noise=5.0;  jitter=1000000; div=2 }
    @{ label="extreme";   drop=0.30; noise=10.0; jitter=5000000; div=4 }
    @{ label="nightmare"; drop=0.50; noise=20.0; jitter=10000000; div=8 }
)

# ── Helper: run one eval and collect results ─────────────────────────────────
function Run-FaultEval {
    param(
        [string]$Preset,
        [string]$RunDir,
        [double]$Drop    = 0.0,
        [double]$Noise   = 0.0,
        [long]$Jitter    = 0,
        [int]$RateDiv    = 1
    )

    New-Item -ItemType Directory -Force -Path $RunDir | Out-Null

    $evalArgs = @("--stress", $Preset, "-o", $RunDir, "--no-plot", "--csv-only")
    if ($Drop -gt 0)    { $evalArgs += @("--fault-drop",      "$Drop") }
    if ($Noise -gt 0)   { $evalArgs += @("--fault-imu-noise", "$Noise") }
    if ($Jitter -gt 0)  { $evalArgs += @("--fault-jitter",    "$Jitter") }
    if ($RateDiv -gt 1) { $evalArgs += @("--fault-rate-div",  "$RateDiv") }

    & $EvalBin @evalArgs 2>$null

    $csvPath = "$RunDir\metrics.csv"
    if (Test-Path $csvPath) {
        $m = Import-Csv $csvPath
        $result = @{}
        foreach ($row in $m) {
            $result[$row.metric] = $row.value
        }
        return $result
    }
    return $null
}

# ── Results CSV header ───────────────────────────────────────────────────────
$csvHeader = "sweep_mode,sweep_value,preset,fault_drop,fault_noise,fault_jitter,fault_div,ate_rmse,ate_max,rpe_trans,rpe_rot,drift_pct,drift_100m,rt_avg,rt_p95,rt_max,num_poses,distance,duration"
$allRows = [System.Collections.Generic.List[string]]::new()

# ── Create output directory ──────────────────────────────────────────────────
New-Item -ItemType Directory -Force -Path $OutputDir       | Out-Null
New-Item -ItemType Directory -Force -Path "$OutputDir\runs" | Out-Null

# ── Count total runs ────────────────────────────────────────────────────────
$totalSingle = 0
foreach ($s in $FAULT_SWEEPS.GetEnumerator()) { $totalSingle += $s.Value.Count }
$totalRuns = ($totalSingle + $COMBINED_FAULTS.Count) * $PRESETS.Count
$runIdx = 0

$sw = [System.Diagnostics.Stopwatch]::StartNew()

# ═══════════════════════════════════════════════════════════════════════════
#  Phase 1: Single-fault sweeps
# ═══════════════════════════════════════════════════════════════════════════
Write-Host "════════════════════════════════════════════════════════════════"
Write-Host "  Phase 1: Single-fault sweeps ($totalSingle values × $($PRESETS.Count) presets)"
Write-Host "════════════════════════════════════════════════════════════════"

foreach ($sweep in $FAULT_SWEEPS.GetEnumerator()) {
    $mode = $sweep.Key
    $values = $sweep.Value

    Write-Host "`n── Sweeping: $mode ──────────────────────────────────"

    foreach ($val in $values) {
        foreach ($preset in $PRESETS) {
            $runIdx++
            $valTag = "$val" -replace '[^a-zA-Z0-9._\-]', ''
            $runDir = "$OutputDir\runs\${mode}_${valTag}_${preset}"

            # Map sweep mode to fault parameters
            $drop = 0.0; $noise = 0.0; $jitter = [long]0; $div = 1
            switch ($mode) {
                "lidar_drop" { $drop    = $val }
                "imu_noise"  { $noise   = $val }
                "ts_jitter"  { $jitter  = [long]$val }
                "rate_div"   { $div     = [int]$val }
            }

            $m = Run-FaultEval -Preset $preset -RunDir $runDir `
                               -Drop $drop -Noise $noise -Jitter $jitter -RateDiv $div

            if ($m) {
                $row = "$mode,$val,$preset,$drop,$noise,$jitter,$div"
                foreach ($metric in @("ate_rmse","ate_max","rpe_trans_rmse","rpe_rot_rmse",
                                      "drift_pct","drift_per_100m",
                                      "runtime_avg","runtime_p95","runtime_max",
                                      "num_poses","distance","duration")) {
                    $v = $m[$metric]
                    if (-not $v) { $v = "0" }
                    $row += ",$v"
                }
                $allRows.Add($row)
                $ate = $m["ate_rmse"]
                Write-Host ("  [{0,5:F1}%] {1}={2} / {3} : ATE={4}" -f `
                    ([math]::Round(100 * $runIdx / $totalRuns, 1)), $mode, $val, $preset, $ate)
            } else {
                # Failed run — record NaN
                $row = "$mode,$val,$preset,$drop,$noise,$jitter,$div"
                $row += ",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,0,0,0"
                $allRows.Add($row)
                Write-Host ("  [{0,5:F1}%] {1}={2} / {3} : FAILED" -f `
                    ([math]::Round(100 * $runIdx / $totalRuns, 1)), $mode, $val, $preset)
            }

            # ETA
            $elapsed = $sw.Elapsed
            if ($runIdx -gt 0) {
                $eta = [TimeSpan]::FromTicks($elapsed.Ticks / $runIdx * ($totalRuns - $runIdx))
                Write-Host ("           elapsed {0:mm\:ss}  ETA {1:mm\:ss}" -f $elapsed, $eta)
            }
        }
    }
}

# ═══════════════════════════════════════════════════════════════════════════
#  Phase 2: Combined-fault scenarios
# ═══════════════════════════════════════════════════════════════════════════
Write-Host "`n════════════════════════════════════════════════════════════════"
Write-Host "  Phase 2: Combined-fault scenarios ($($COMBINED_FAULTS.Count) × $($PRESETS.Count) presets)"
Write-Host "════════════════════════════════════════════════════════════════"

foreach ($combo in $COMBINED_FAULTS) {
    $label = $combo.label
    Write-Host "`n── Combined: $label ──────────────────────────────────"

    foreach ($preset in $PRESETS) {
        $runIdx++
        $runDir = "$OutputDir\runs\combined_${label}_${preset}"

        $m = Run-FaultEval -Preset $preset -RunDir $runDir `
                           -Drop $combo.drop -Noise $combo.noise `
                           -Jitter $combo.jitter -RateDiv $combo.div

        if ($m) {
            $row = "combined_$label,$label,$preset,$($combo.drop),$($combo.noise),$($combo.jitter),$($combo.div)"
            foreach ($metric in @("ate_rmse","ate_max","rpe_trans_rmse","rpe_rot_rmse",
                                  "drift_pct","drift_per_100m",
                                  "runtime_avg","runtime_p95","runtime_max",
                                  "num_poses","distance","duration")) {
                $v = $m[$metric]
                if (-not $v) { $v = "0" }
                $row += ",$v"
            }
            $allRows.Add($row)
            $ate = $m["ate_rmse"]
            Write-Host ("  [{0,5:F1}%] combined_{1} / {2} : ATE={3}" -f `
                ([math]::Round(100 * $runIdx / $totalRuns, 1)), $label, $preset, $ate)
        } else {
            $row = "combined_$label,$label,$preset,$($combo.drop),$($combo.noise),$($combo.jitter),$($combo.div)"
            $row += ",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,0,0,0"
            $allRows.Add($row)
            Write-Host ("  [{0,5:F1}%] combined_{1} / {2} : FAILED" -f `
                ([math]::Round(100 * $runIdx / $totalRuns, 1)), $label, $preset)
        }
    }
}

$sw.Stop()

# ── 3. Write consolidated CSV ───────────────────────────────────────────────
$csvOut = "$OutputDir\robustness_results.csv"
$csvHeader | Out-File -FilePath $csvOut -Encoding utf8
$allRows | Out-File -FilePath $csvOut -Encoding utf8 -Append

Write-Host "`n════════════════════════════════════════════════════════════════"
Write-Host "  Done: $($allRows.Count) results in $($sw.Elapsed.ToString('mm\:ss'))"
Write-Host "  CSV:  $csvOut"
Write-Host "════════════════════════════════════════════════════════════════"

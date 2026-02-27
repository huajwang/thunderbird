# Thunderbird SLAM — Robustness Evaluation Report

## Overview

This report evaluates the SLAM system's resilience to simulated hardware
faults. Four fault modes are tested independently, plus five combined-fault
scenarios of increasing severity.

**Fault Modes:**
| Mode | Description | Sweep Range |
|------|-------------|-------------|
| LiDAR Frame Drop | Random frame loss | 0–50% |
| IMU Additive Noise | Scaled Gaussian noise | 0–20× baseline σ |
| Timestamp Jitter | Uniform ± offset | 0–10 ms |
| LiDAR Rate Reduction | Keep every Nth frame | 1× – 8× decimation |

---
## Single-Fault Degradation

### LiDAR Frame Drop

| Drop Rate | Preset | ATE RMSE | Δ ATE | Drift | Δ Drift | Poses | Avg RT | Grade |
|---|---|---|---|---|---|---|---|---|
| 0% | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 8.60 ms | ✅ |
| 0% | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 10.18 ms | ✅ |
| 0% | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.08 ms | ✅ |
| 0% | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.01 ms | ✅ |
| 5% | aggressive_drone | 35.876 m | +0.024 m | 7.36% | -0.02% | 608 | 9.69 ms | ✅ |
| 5% | degenerate_corridor | 1.845 m | +0.005 m | 27.25% | -0.08% | 457 | 15.04 ms | ✅ |
| 5% | fast_car | 204.895 m | -0.369 m | 16.80% | +0.19% | 1176 | 7.72 ms | ✅ |
| 5% | spinning_top | 0.228 m | -0.004 m | 9.45% | -0.34% | 220 | 11.31 ms | ✅ |
| 10% | aggressive_drone | 35.928 m | +0.077 m | 7.37% | -0.01% | 576 | 9.90 ms | ✅ |
| 10% | degenerate_corridor | 1.849 m | +0.010 m | 26.47% | -0.85% | 432 | 12.48 ms | ✅ |
| 10% | fast_car | 204.852 m | -0.412 m | 16.67% | +0.07% | 1121 | 7.93 ms | ✅ |
| 10% | spinning_top | 0.225 m | -0.007 m | 9.33% | -0.46% | 206 | 11.08 ms | ✅ |
| 15% | aggressive_drone | 35.798 m | -0.054 m | 7.34% | -0.04% | 542 | 10.54 ms | ⚠️ |
| 15% | degenerate_corridor | 1.860 m | +0.021 m | 26.23% | -1.09% | 407 | 12.06 ms | ⚠️ |
| 15% | fast_car | 204.843 m | -0.421 m | 16.65% | +0.04% | 1057 | 8.16 ms | ⚠️ |
| 15% | spinning_top | 0.221 m | -0.010 m | 8.97% | -0.83% | 197 | 9.96 ms | ⚠️ |
| 20% | aggressive_drone | 36.114 m | +0.262 m | 7.22% | -0.16% | 501 | 9.98 ms | ⚠️ |
| 20% | degenerate_corridor | 1.880 m | +0.040 m | 25.10% | -2.23% | 372 | 9.47 ms | ⚠️ |
| 20% | fast_car | 204.755 m | -0.509 m | 16.40% | -0.21% | 981 | 7.92 ms | ⚠️ |
| 20% | spinning_top | 0.223 m | -0.009 m | 8.95% | -0.84% | 191 | 10.01 ms | ⚠️ |
| 30% | aggressive_drone | 36.742 m | +0.891 m | 7.23% | -0.15% | 456 | 10.03 ms | ⚠️ |
| 30% | degenerate_corridor | 1.886 m | +0.046 m | 22.86% | -4.47% | 340 | 9.55 ms | ⚠️ |
| 30% | fast_car | 204.597 m | -0.667 m | 16.41% | -0.20% | 875 | 7.96 ms | ⚠️ |
| 30% | spinning_top | 0.224 m | -0.008 m | 8.86% | -0.94% | 175 | 9.40 ms | ⚠️ |
| 40% | aggressive_drone | 36.704 m | +0.853 m | 7.16% | -0.22% | 391 | 9.12 ms | ⚠️ |
| 40% | degenerate_corridor | 1.886 m | +0.047 m | 21.54% | -5.78% | 293 | 9.30 ms | ⚠️ |
| 40% | fast_car | 204.858 m | -0.406 m | 16.15% | -0.45% | 753 | 8.06 ms | ⚠️ |
| 40% | spinning_top | 0.225 m | -0.007 m | 8.74% | -1.05% | 151 | 9.54 ms | ⚠️ |
| 50% | aggressive_drone | 36.852 m | +1.000 m | 6.85% | -0.53% | 312 | 9.33 ms | ❌ |
| 50% | degenerate_corridor | 1.903 m | +0.063 m | 20.85% | -6.47% | 234 | 8.98 ms | ❌ |
| 50% | fast_car | 204.631 m | -0.633 m | 15.61% | -1.00% | 610 | 7.84 ms | ⚠️ |
| 50% | spinning_top | 0.224 m | -0.008 m | 8.06% | -1.74% | 123 | 10.03 ms | ⚠️ |

### IMU Additive Noise

| Noise Scale | Preset | ATE RMSE | Δ ATE | Drift | Δ Drift | Poses | Avg RT | Grade |
|---|---|---|---|---|---|---|---|---|
| 0.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.09 ms | ✅ |
| 0.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.65 ms | ✅ |
| 0.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.26 ms | ✅ |
| 0.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.98 ms | ✅ |
| 0.5× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 9.71 ms | ✅ |
| 0.5× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.41 ms | ✅ |
| 0.5× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.51 ms | ✅ |
| 0.5× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.20 ms | ✅ |
| 1.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.65 ms | ✅ |
| 1.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 14.50 ms | ✅ |
| 1.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.38 ms | ✅ |
| 1.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.66 ms | ✅ |
| 2.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 12.09 ms | ✅ |
| 2.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.73 ms | ✅ |
| 2.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 7.78 ms | ✅ |
| 2.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.47 ms | ✅ |
| 5.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.19 ms | ✅ |
| 5.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 12.51 ms | ✅ |
| 5.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.53 ms | ✅ |
| 5.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.11 ms | ✅ |
| 10.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.48 ms | ✅ |
| 10.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 13.91 ms | ✅ |
| 10.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 7.24 ms | ✅ |
| 10.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.84 ms | ✅ |
| 20.0× | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.92 ms | ✅ |
| 20.0× | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 14.78 ms | ✅ |
| 20.0× | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.42 ms | ✅ |
| 20.0× | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.11 ms | ✅ |

### Timestamp Jitter

| Jitter | Preset | ATE RMSE | Δ ATE | Drift | Δ Drift | Poses | Avg RT | Grade |
|---|---|---|---|---|---|---|---|---|
| 0 | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.49 ms | ✅ |
| 0 | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 14.36 ms | ✅ |
| 0 | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 7.88 ms | ✅ |
| 0 | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.02 ms | ✅ |
| 1 µs | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.38 ms | ✅ |
| 1 µs | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 14.38 ms | ✅ |
| 1 µs | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 7.87 ms | ✅ |
| 1 µs | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.36 ms | ✅ |
| 10 µs | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.48 ms | ✅ |
| 10 µs | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.50 ms | ✅ |
| 10 µs | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 7.83 ms | ✅ |
| 10 µs | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 9.98 ms | ✅ |
| 100 µs | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.23 ms | ✅ |
| 100 µs | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.78 ms | ✅ |
| 100 µs | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.00 ms | ✅ |
| 100 µs | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.15 ms | ✅ |
| 500 µs | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 9.96 ms | ✅ |
| 500 µs | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 12.89 ms | ✅ |
| 500 µs | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.61 ms | ✅ |
| 500 µs | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.67 ms | ✅ |
| 1 ms | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 11.28 ms | ✅ |
| 1 ms | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 10.43 ms | ✅ |
| 1 ms | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.74 ms | ✅ |
| 1 ms | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 11.98 ms | ✅ |
| 5 ms | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.06 ms | ✅ |
| 5 ms | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.77 ms | ✅ |
| 5 ms | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.35 ms | ✅ |
| 5 ms | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 9.97 ms | ✅ |
| 10 ms | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 10.58 ms | ✅ |
| 10 ms | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.71 ms | ✅ |
| 10 ms | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.31 ms | ✅ |
| 10 ms | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.09 ms | ✅ |

### LiDAR Rate Reduction

| Rate Divisor | Preset | ATE RMSE | Δ ATE | Drift | Δ Drift | Poses | Avg RT | Grade |
|---|---|---|---|---|---|---|---|---|
| 1/1 | aggressive_drone | 35.851 m | +0.000 m | 7.38% | +0.00% | 625 | 9.63 ms | ✅ |
| 1/1 | degenerate_corridor | 1.840 m | +0.000 m | 27.32% | +0.00% | 470 | 9.57 ms | ✅ |
| 1/1 | fast_car | 205.264 m | +0.000 m | 16.61% | +0.00% | 1220 | 8.24 ms | ✅ |
| 1/1 | spinning_top | 0.232 m | +0.000 m | 9.80% | +0.00% | 226 | 10.50 ms | ✅ |
| 1/2 | aggressive_drone | 35.769 m | -0.082 m | 6.86% | -0.52% | 328 | 10.90 ms | ⚠️ |
| 1/2 | degenerate_corridor | 1.898 m | +0.059 m | 21.19% | -6.14% | 251 | 10.49 ms | ⚠️ |
| 1/2 | fast_car | 207.870 m | +2.606 m | 15.81% | -0.79% | 626 | 7.48 ms | ⚠️ |
| 1/2 | spinning_top | 0.228 m | -0.003 m | 7.77% | -2.02% | 129 | 9.45 ms | ⚠️ |
| 1/3 | aggressive_drone | 35.375 m | -0.477 m | 6.47% | -0.90% | 229 | 7.39 ms | ❌ |
| 1/3 | degenerate_corridor | 1.919 m | +0.080 m | 20.00% | -7.32% | 177 | 12.74 ms | ❌ |
| 1/3 | fast_car | 210.526 m | +5.262 m | 14.89% | -1.72% | 427 | 8.34 ms | ❌ |
| 1/3 | spinning_top | 0.219 m | -0.013 m | 6.85% | -2.95% | 96 | 6.35 ms | ❌ |
| 1/4 | aggressive_drone | 35.203 m | -0.648 m | 6.30% | -1.08% | 179 | 5.81 ms | ❌ |
| 1/4 | degenerate_corridor | 1.907 m | +0.067 m | 18.38% | -8.94% | 141 | 13.24 ms | ❌ |
| 1/4 | fast_car | 213.936 m | +8.672 m | 13.93% | -2.68% | 328 | 8.25 ms | ❌ |
| 1/4 | spinning_top | 0.213 m | -0.019 m | 5.92% | -3.88% | 80 | 6.76 ms | ❌ |
| 1/6 | aggressive_drone | 34.384 m | -1.468 m | 6.15% | -1.23% | 130 | 9.29 ms | ❌ |
| 1/6 | degenerate_corridor | 1.818 m | -0.022 m | 16.18% | -11.14% | 104 | 12.93 ms | ❌ |
| 1/6 | fast_car | 216.499 m | +11.235 m | 12.84% | -3.76% | 229 | 8.16 ms | ❌ |
| 1/6 | spinning_top | 0.228 m | -0.004 m | 5.79% | -4.01% | 64 | 11.15 ms | ❌ |
| 1/8 | aggressive_drone | 35.246 m | -0.606 m | 6.11% | -1.27% | 105 | 8.81 ms | ❌ |
| 1/8 | degenerate_corridor | 1.796 m | -0.043 m | 15.07% | -12.25% | 86 | 12.78 ms | ❌ |
| 1/8 | fast_car | 214.961 m | +9.697 m | 12.42% | -4.19% | 179 | 15.01 ms | ❌ |
| 1/8 | spinning_top | 0.237 m | +0.006 m | 6.03% | -3.77% | 48 | 6.89 ms | ❌ |

---
## Combined-Fault Scenarios

| Scenario | Preset | Drop | Noise | Jitter | Div | ATE RMSE | Drift | Poses | Grade |
|---|---|---|---|---|---|---|---|---|---|
| mild | aggressive_drone | 5% | 1× | 10 µs | 1/1 | 35.583 m | 7.39% | 591 | ✅ |
| mild | degenerate_corridor | 5% | 1× | 10 µs | 1/1 | 1.843 m | 26.19% | 447 | ✅ |
| mild | fast_car | 5% | 1× | 10 µs | 1/1 | 204.422 m | 16.63% | 1163 | ✅ |
| mild | spinning_top | 5% | 1× | 10 µs | 1/1 | 0.232 m | 9.69% | 220 | ✅ |
| moderate | aggressive_drone | 10% | 2× | 100 µs | 1/2 | 35.914 m | 6.53% | 289 | ❌ |
| moderate | degenerate_corridor | 10% | 2× | 100 µs | 1/2 | 1.904 m | 21.20% | 231 | ❌ |
| moderate | fast_car | 10% | 2× | 100 µs | 1/2 | 210.272 m | 15.77% | 563 | ❌ |
| moderate | spinning_top | 10% | 2× | 100 µs | 1/2 | 0.225 m | 7.47% | 124 | ⚠️ |
| harsh | aggressive_drone | 20% | 5× | 1 ms | 1/2 | 35.626 m | 6.76% | 272 | ❌ |
| harsh | degenerate_corridor | 20% | 5× | 1 ms | 1/2 | 1.912 m | 20.63% | 205 | ❌ |
| harsh | fast_car | 20% | 5× | 1 ms | 1/2 | 209.401 m | 15.29% | 500 | ❌ |
| harsh | spinning_top | 20% | 5× | 1 ms | 1/2 | 0.227 m | 7.25% | 113 | ⚠️ |
| extreme | aggressive_drone | 30% | 10× | 5 ms | 1/4 | 34.660 m | 6.64% | 141 | ❌ |
| extreme | degenerate_corridor | 30% | 10× | 5 ms | 1/4 | 1.848 m | 16.31% | 106 | ❌ |
| extreme | fast_car | 30% | 10× | 5 ms | 1/4 | 218.629 m | 13.19% | 234 | ❌ |
| extreme | spinning_top | 30% | 10× | 5 ms | 1/4 | 0.246 m | 5.85% | 63 | ❌ |
| nightmare | aggressive_drone | 50% | 20× | 10 ms | 1/8 | 34.755 m | 6.03% | 69 | ❌ |
| nightmare | degenerate_corridor | 50% | 20× | 10 ms | 1/8 | 1.811 m | 15.44% | 56 | ❌ |
| nightmare | fast_car | 50% | 20× | 10 ms | 1/8 | 190.938 m | 11.46% | 102 | ❌ |
| nightmare | spinning_top | 50% | 20× | 10 ms | 1/8 | 0.254 m | 7.50% | 28 | ❌ |

---
## Failure Thresholds

Failure is defined as ATE increasing > 3× baseline **or** drift increasing
> 10 percentage points **or** > 50% pose loss.

| Fault Mode | Preset | ATE 3× Threshold | Drift +10pp Threshold |
|---|---|---|---|
| LiDAR Frame Drop | aggressive_drone | > max tested | > max tested |
| LiDAR Frame Drop | degenerate_corridor | > max tested | > max tested |
| LiDAR Frame Drop | fast_car | > max tested | > max tested |
| LiDAR Frame Drop | spinning_top | > max tested | > max tested |
| IMU Additive Noise | aggressive_drone | > max tested | > max tested |
| IMU Additive Noise | degenerate_corridor | > max tested | > max tested |
| IMU Additive Noise | fast_car | > max tested | > max tested |
| IMU Additive Noise | spinning_top | > max tested | > max tested |
| Timestamp Jitter | aggressive_drone | > max tested | > max tested |
| Timestamp Jitter | degenerate_corridor | > max tested | > max tested |
| Timestamp Jitter | fast_car | > max tested | > max tested |
| Timestamp Jitter | spinning_top | > max tested | > max tested |
| LiDAR Rate Reduction | aggressive_drone | > max tested | > max tested |
| LiDAR Rate Reduction | degenerate_corridor | > max tested | > max tested |
| LiDAR Rate Reduction | fast_car | > max tested | > max tested |
| LiDAR Rate Reduction | spinning_top | > max tested | > max tested |

---
## Summary Statistics

- **Total runs:** 136
  - Single-fault: 116
  - Combined-fault: 20
- **Pass:** 80  (59%)
- **Warn:** 24  (18%)
- **Fail:** 32  (24%)

### Per-Platform Resilience

- **Drone:** 40/68 pass (59%)
- **Car:** 40/68 pass (59%)

---
## Recommendations

Based on the robustness evaluation:

- 🟢 **LiDAR Rate Reduction**: Max ATE increase 5% — resilient
- 🟢 **LiDAR Frame Drop**: Max ATE increase 3% — resilient
- 🟢 **IMU Additive Noise**: Max ATE increase 0% — resilient
- 🟢 **Timestamp Jitter**: Max ATE increase 0% — resilient

---
*Generated by analyse_robustness.py*

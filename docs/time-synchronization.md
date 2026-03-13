# Time Synchronization — How It Works

```
Time ──────────────────────────────────────────────────────▶

LiDAR  ─── L1 ─────────────── L2 ─────────────── L3 ───
            10 Hz

IMU    ─ I1 I2 I3 I4 I5 I6 I7 I8 I9 I10 I11 I12 I13 ──
            200 Hz

Camera ──── C1 ─────── C2 ─────── C3 ─────── C4 ───────
            30 Hz

SyncEngine takes each LiDAR frame as reference and finds the
nearest IMU sample and Camera frame within ±tolerance_ns.

Bundle 1:  L1 + I_nearest + C_nearest
Bundle 2:  L2 + I_nearest + C_nearest
...
```

## Algorithm (per sync cycle)

1. Pop the oldest unconsumed LiDAR frame → `ref_ts`
2. Search IMU queue for sample closest to `ref_ts` within tolerance
3. Search Camera queue for frame closest to `ref_ts` within tolerance
4. If Camera is available, emit `SyncBundle`; evict consumed data
5. If not, wait until next poll cycle

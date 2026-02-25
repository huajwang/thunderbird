#!/usr/bin/env python3
"""
Thunderbird Spatial SDK — Python example.

Demonstrates:
  1. Connecting to the (simulated) device
  2. Pulling LiDAR / IMU / Camera frames via the Pull API
  3. Registering a callback for synchronised frames
  4. Recording a short session to disk
  5. Replaying the session with the Player (Pull + Callback)
  6. Printing frame info to verify correctness

Build the SDK with:
    cmake -S . -B build -DTHUNDERBIRD_BUILD_PYTHON=ON \\
          -DTHUNDERBIRD_USE_SIMULATED=ON -DCMAKE_BUILD_TYPE=Release
    cmake --build build --target _spatial_sdk_core

Run with PYTHONPATH pointing to the build output:
    PYTHONPATH=build/sdk/python python examples/python_example.py
"""

from __future__ import annotations

import sys
import time

import spatial_sdk as sdk

RECORDING_FILE = "demo_session.tbrec"


def section(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


# ─────────────────────────────────────────────────────────────────────────────
#  1. Connect to device and pull a few frames
# ─────────────────────────────────────────────────────────────────────────────
def demo_pull_api() -> None:
    section("1 · Pull API — connect, start, pull frames")

    dev = sdk.DeviceManager()
    status = dev.connect()
    print(f"  connect()   → {status}")

    status = dev.start()
    print(f"  start()     → {status}")

    # Let the simulated sensors produce some data.
    time.sleep(0.3)

    # Pull a few frames from each sensor.
    for _ in range(3):
        frame = dev.get_next_lidar_frame()
        if frame is not None:
            pts = frame.points   # NumPy structured array
            print(f"  LiDAR  ts={frame.timestamp_ns:>15}  seq={frame.sequence:>4}"
                  f"  pts={frame.num_points:>5}  x[0]={pts['x'][0]:.3f}")

    for _ in range(5):
        frame = dev.get_next_imu_frame()
        if frame is not None:
            a = frame.accel
            print(f"  IMU    ts={frame.timestamp_ns:>15}"
                  f"  accel=[{a[0]:.3f}, {a[1]:.3f}, {a[2]:.3f}]")

    for _ in range(2):
        frame = dev.get_next_image_frame()
        if frame is not None:
            px = frame.pixels
            print(f"  Camera ts={frame.timestamp_ns:>15}  {frame.width}×{frame.height}"
                  f"  seq={frame.sequence}  shape={px.shape}")

    dev.stop()
    dev.disconnect()
    print("  (stopped & disconnected)")


# ─────────────────────────────────────────────────────────────────────────────
#  2. Callback API — synchronised frames
# ─────────────────────────────────────────────────────────────────────────────
def demo_callback_api() -> None:
    section("2 · Callback API — synchronised frames")

    synced_frames: list[sdk.SyncedFrame] = []

    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()

    def on_synced(sf: sdk.SyncedFrame) -> None:
        synced_frames.append(sf)

    dev.register_synced_frame_callback(on_synced)

    time.sleep(0.5)  # collect ~5 synced frames at 10 Hz LiDAR
    dev.stop()
    dev.disconnect()

    print(f"  Received {len(synced_frames)} synchronised frames:")
    for sf in synced_frames[:3]:
        cam_info = "None"
        if sf.camera is not None:
            cam_info = f"{sf.camera.width}×{sf.camera.height} ts={sf.camera.timestamp_ns}"
        print(f"    seq={sf.sequence}  quality={sf.sync_quality:.2f}"
              f"  imu_count={len(sf.imu_block):>3}  camera={cam_info}")

    stats = dev.sync_stats()
    print(f"  Sync stats: {stats}")


# ─────────────────────────────────────────────────────────────────────────────
#  3. Record a session
# ─────────────────────────────────────────────────────────────────────────────
def demo_recorder() -> None:
    section("3 · Recorder — save session to disk")

    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()
    time.sleep(0.2)

    info = sdk.RecorderDeviceInfo("PY-SN-001", "1.0.0", "DemoSensor")
    rec = sdk.Recorder(RECORDING_FILE, info)
    rec.start()
    print(f"  Recording to '{RECORDING_FILE}' ...")

    # Pull frames and record them for ~0.5 seconds.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        lf = dev.get_next_lidar_frame()
        if lf is not None:
            rec.record_lidar_frame(lf)

        for _ in range(20):
            imf = dev.get_next_imu_frame()
            if imf is not None:
                rec.record_imu_frame(imf)

        cf = dev.get_next_image_frame()
        if cf is not None:
            rec.record_image_frame(cf)

        time.sleep(0.02)

    rec.stop()
    dev.stop()
    dev.disconnect()

    s = rec.stats()
    print(f"  Recorded: {s}")


# ─────────────────────────────────────────────────────────────────────────────
#  4. Replay session — Pull API
# ─────────────────────────────────────────────────────────────────────────────
def demo_player_pull() -> None:
    section("4 · Player Pull API — replay from disk")

    player = sdk.Player(RECORDING_FILE, sdk.PlayerConfig(playback_speed=0.0))
    if not player.start():
        print("  ERROR: could not open recording")
        return

    info = player.device_info()
    print(f"  Device info: {info}")

    # Wait for playback to finish (as-fast-as-possible)
    while not player.finished():
        time.sleep(0.005)

    # Drain all frames
    lidar_count = imu_count = cam_count = 0
    while True:
        f = player.get_next_lidar_frame()
        if f is None:
            break
        lidar_count += 1
        if lidar_count <= 2:
            print(f"  LiDAR  ts={f.timestamp_ns}  seq={f.sequence}"
                  f"  pts={f.num_points}")

    while player.get_next_imu_frame() is not None:
        imu_count += 1

    while player.get_next_image_frame() is not None:
        cam_count += 1

    player.stop()
    print(f"  Played back:  lidar={lidar_count}  imu={imu_count}  cam={cam_count}")
    print(f"  Player stats: {player.stats()}")


# ─────────────────────────────────────────────────────────────────────────────
#  5. Replay session — Callback API
# ─────────────────────────────────────────────────────────────────────────────
def demo_player_callback() -> None:
    section("5 · Player Callback API — replay from disk at 2× speed")

    counts = {"lidar": 0, "imu": 0, "camera": 0}

    player = sdk.Player(RECORDING_FILE, sdk.PlayerConfig(playback_speed=2.0))

    player.on_lidar_frame(lambda f: counts.__setitem__("lidar", counts["lidar"] + 1))
    player.on_imu_frame(lambda f: counts.__setitem__("imu", counts["imu"] + 1))
    player.on_image_frame(lambda f: counts.__setitem__("camera", counts["camera"] + 1))

    player.start()

    while not player.finished():
        time.sleep(0.01)
    player.stop()

    print(f"  Callback counts: lidar={counts['lidar']}"
          f"  imu={counts['imu']}  camera={counts['camera']}")
    print(f"  Player stats:    {player.stats()}")


# ─────────────────────────────────────────────────────────────────────────────
#  6. Context-manager usage
# ─────────────────────────────────────────────────────────────────────────────
def demo_context_manager() -> None:
    section("6 · Context-manager (with statement)")

    with sdk.DeviceManager() as dev:
        dev.start()
        time.sleep(0.1)
        f = dev.get_next_lidar_frame()
        if f:
            print(f"  Got LiDAR frame: {f}")
        dev.stop()
    print("  (DeviceManager auto-disconnected)")


# ─────────────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  Thunderbird Spatial SDK — Python Example")
    print("=" * 60)

    demo_pull_api()
    demo_callback_api()
    demo_recorder()
    demo_player_pull()
    demo_player_callback()
    demo_context_manager()

    print(f"\n{'=' * 60}")
    print("  All demos completed successfully!")
    print(f"{'=' * 60}")

    # Clean up recording file
    import os
    try:
        os.remove(RECORDING_FILE)
    except OSError:
        pass

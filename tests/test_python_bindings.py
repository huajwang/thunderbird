#!/usr/bin/env python3
"""
Thunderbird Spatial SDK — Python binding smoke-tests.

Run after building:
    PYTHONPATH=build/sdk/python python tests/test_python_bindings.py

These tests validate that:
  • All expected classes and enums are importable
  • DeviceManager connects / starts / stops (simulated mode)
  • Pull API returns frames with correct NumPy types
  • Callback API delivers frames
  • Recorder round-trips through Player
  • Context managers work
"""

from __future__ import annotations

import os
import sys
import time
import traceback

# ── helpers ──────────────────────────────────────────────────────────────────

_passed = 0
_failed = 0


def run_test(name: str, fn) -> None:
    global _passed, _failed
    try:
        fn()
        print(f"  [PASS] {name}")
        _passed += 1
    except Exception:
        print(f"  [FAIL] {name}")
        traceback.print_exc()
        _failed += 1


# ── tests ────────────────────────────────────────────────────────────────────

def test_imports():
    import spatial_sdk as sdk
    # Check that key symbols exist
    for attr in ("Status", "PixelFormat", "DeviceInfo", "LidarFrame",
                 "ImuFrame", "ImageFrame", "SyncedFrame", "SyncStats",
                 "RecorderDeviceInfo", "RecorderStats", "PlayerConfig",
                 "PlayerStats", "DeviceConfig", "DeviceManager",
                 "Recorder", "Player"):
        assert hasattr(sdk, attr), f"Missing: {attr}"


def test_status_enum():
    import spatial_sdk as sdk
    assert sdk.Status.OK is not None
    assert str(sdk.Status.OK) == "Status.OK"
    assert sdk.Status.NotConnected is not None


def test_pixel_format_enum():
    import spatial_sdk as sdk
    assert sdk.PixelFormat.RGB8 is not None
    assert sdk.PixelFormat.Mono8 is not None


def test_device_connect_start_stop():
    import spatial_sdk as sdk
    dev = sdk.DeviceManager()
    s = dev.connect()
    assert s == sdk.Status.OK, f"connect returned {s}"
    assert dev.is_connected()

    s = dev.start()
    assert s == sdk.Status.OK, f"start returned {s}"
    assert dev.is_streaming()

    time.sleep(0.15)

    s = dev.stop()
    assert s == sdk.Status.OK, f"stop returned {s}"

    s = dev.disconnect()
    assert s == sdk.Status.OK, f"disconnect returned {s}"
    assert not dev.is_connected()


def test_pull_api():
    import numpy as np
    import spatial_sdk as sdk

    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()
    time.sleep(0.2)

    # LiDAR
    lf = dev.get_next_lidar_frame()
    assert lf is not None, "Expected a LiDAR frame"
    assert lf.timestamp_ns > 0
    pts = lf.points
    assert hasattr(pts, 'dtype'), "points should be a NumPy array"
    assert 'x' in pts.dtype.names
    assert 'intensity' in pts.dtype.names

    # IMU
    imf = dev.get_next_imu_frame()
    assert imf is not None, "Expected an IMU frame"
    assert imf.timestamp_ns > 0
    a = imf.accel
    assert len(a) == 3
    assert isinstance(a[0], (float, np.floating))

    # Camera
    cf = dev.get_next_image_frame()
    assert cf is not None, "Expected a camera frame"
    assert cf.width > 0 and cf.height > 0
    px = cf.pixels
    assert hasattr(px, 'shape')
    assert px.ndim >= 2

    dev.stop()
    dev.disconnect()


def test_callback_api():
    import spatial_sdk as sdk

    counts = {"lidar": 0, "imu": 0, "camera": 0}

    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()

    dev.register_lidar_callback(lambda f: counts.__setitem__("lidar", counts["lidar"] + 1))
    dev.register_imu_callback(lambda f: counts.__setitem__("imu", counts["imu"] + 1))
    dev.register_image_callback(lambda f: counts.__setitem__("camera", counts["camera"] + 1))

    time.sleep(0.3)
    dev.stop()
    dev.disconnect()

    assert counts["lidar"] > 0, f"Expected LiDAR callbacks, got {counts['lidar']}"
    assert counts["imu"] > 0,   f"Expected IMU callbacks, got {counts['imu']}"
    assert counts["camera"] > 0, f"Expected camera callbacks, got {counts['camera']}"


def test_synced_frame():
    import spatial_sdk as sdk

    frames: list = []
    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()
    dev.register_synced_frame_callback(lambda sf: frames.append(sf))
    time.sleep(0.5)
    dev.stop()
    dev.disconnect()

    assert len(frames) > 0, "Expected at least one synced frame"
    sf = frames[0]
    assert sf.lidar is not None
    assert isinstance(sf.imu_block, list)
    assert sf.sequence >= 0


def test_recorder_player_roundtrip():
    import spatial_sdk as sdk

    test_file = "_test_py_roundtrip.tbrec"

    # Record some frames
    dev = sdk.DeviceManager()
    dev.connect()
    dev.start()
    time.sleep(0.15)

    info = sdk.RecorderDeviceInfo("PY-TEST", "0.0.1", "TestModel")
    rec = sdk.Recorder(test_file, info)
    assert rec.start()

    t0 = time.monotonic()
    recorded = 0
    while time.monotonic() - t0 < 0.3:
        lf = dev.get_next_lidar_frame()
        if lf is not None:
            rec.record_lidar_frame(lf)
            recorded += 1
        imf = dev.get_next_imu_frame()
        if imf is not None:
            rec.record_imu_frame(imf)
            recorded += 1
        time.sleep(0.01)

    rec.stop()
    dev.stop()
    dev.disconnect()

    assert recorded > 0, "Should have recorded some frames"
    s = rec.stats()
    assert s.lidar_frames > 0 or s.imu_frames > 0

    # Play back
    player = sdk.Player(test_file, sdk.PlayerConfig(playback_speed=0.0))
    assert player.start()

    di = player.device_info()
    assert di.serial_number == "PY-TEST"
    assert di.model_name == "TestModel"

    while not player.finished():
        time.sleep(0.005)

    lidar_n = 0
    while player.get_next_lidar_frame() is not None:
        lidar_n += 1

    imu_n = 0
    while player.get_next_imu_frame() is not None:
        imu_n += 1

    player.stop()

    ps = player.stats()
    assert ps.total_records > 0
    assert lidar_n == ps.lidar_frames
    assert imu_n == ps.imu_frames

    os.remove(test_file)


def test_recorder_context_manager():
    import spatial_sdk as sdk

    test_file = "_test_py_ctx.tbrec"
    with sdk.Recorder(test_file) as rec:
        assert rec.recording()
    assert not rec.recording()
    os.remove(test_file)


def test_device_context_manager():
    import spatial_sdk as sdk

    with sdk.DeviceManager() as dev:
        dev.start()
        time.sleep(0.05)
        assert dev.is_streaming()
        dev.stop()
    assert not dev.is_connected()


def test_device_info():
    import spatial_sdk as sdk

    dev = sdk.DeviceManager()
    dev.connect()
    info = dev.device_info()
    assert isinstance(info, sdk.DeviceInfo)
    assert isinstance(info.serial_number, str)
    dev.disconnect()


def test_player_invalid_file():
    import spatial_sdk as sdk
    p = sdk.Player("nonexistent_file.tbrec")
    assert not p.start()


# ── main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Thunderbird Spatial SDK — Python Binding Tests")
    print("=" * 50)

    run_test("test_imports",              test_imports)
    run_test("test_status_enum",          test_status_enum)
    run_test("test_pixel_format_enum",    test_pixel_format_enum)
    run_test("test_device_connect",       test_device_connect_start_stop)
    run_test("test_pull_api",             test_pull_api)
    run_test("test_callback_api",         test_callback_api)
    run_test("test_synced_frame",         test_synced_frame)
    run_test("test_recorder_player",      test_recorder_player_roundtrip)
    run_test("test_recorder_ctx_mgr",     test_recorder_context_manager)
    run_test("test_device_ctx_mgr",       test_device_context_manager)
    run_test("test_device_info",          test_device_info)
    run_test("test_player_invalid_file",  test_player_invalid_file)

    print(f"\n{_passed + _failed} tests run: {_passed} passed, {_failed} failed.")
    sys.exit(0 if _failed == 0 else 1)

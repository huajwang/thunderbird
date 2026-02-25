"""
Thunderbird SDK — Python example (requires thunderbird_py built with pybind11)

Usage:
    python example.py
"""
import time

# Assumes thunderbird_py.so / thunderbird_py.pyd is on PYTHONPATH
import thunderbird_py as tb


def main():
    lidar_count = 0
    imu_count = 0
    camera_count = 0
    sync_count = 0

    def on_lidar(frame):
        nonlocal lidar_count
        lidar_count += 1

    def on_imu(sample):
        nonlocal imu_count
        imu_count += 1

    def on_camera(frame):
        nonlocal camera_count
        camera_count += 1

    def on_sync(bundle):
        nonlocal sync_count
        sync_count += 1
        if bundle.lidar:
            pts = bundle.lidar.num_points()
            print(f"  [sync #{sync_count}] lidar pts={pts}", end="")
        if bundle.camera:
            img = bundle.camera.to_numpy()
            print(f"  cam shape={img.shape}", end="")
        print()

    cfg = tb.DeviceConfig()
    cfg.lidar_hz = 10.0
    cfg.imu_hz = 100.0
    cfg.camera_fps = 15.0

    dev = tb.DeviceManager(cfg)

    dev.on_lidar(on_lidar)
    dev.on_imu(on_imu)
    dev.on_camera(on_camera)
    dev.on_sync(on_sync)

    status = dev.connect()
    if status != tb.Status.OK:
        print(f"connect failed: {status}")
        return

    info = dev.device_info()
    print(f"Device: {info.model_name}  S/N: {info.serial_number}")

    dev.start()
    print("Streaming for 3 seconds ...")
    time.sleep(3)
    dev.stop()
    dev.disconnect()

    print(f"\nTotals — LiDAR: {lidar_count}  IMU: {imu_count}  Camera: {camera_count}  Sync: {sync_count}")


if __name__ == "__main__":
    main()

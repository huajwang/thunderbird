# Hardware Setup Guide

Connect the Thunderbird SDK to physical LiDAR, IMU, and Camera devices.

---

## Supported Hardware

### LiDAR

| Device | Interface | Protocol | Status |
|--------|-----------|----------|--------|
| Velodyne VLP-16 | Ethernet (UDP) | Velodyne data packet v1 | ✅ Supported |
| Ouster OS1-64 | Ethernet (TCP/UDP) | Ouster sensor protocol | 🔲 Planned |
| Livox Mid-360 | Ethernet (UDP) | Livox SDK protocol | 🔲 Planned |

### IMU

| Device | Interface | Protocol | Status |
|--------|-----------|----------|--------|
| Integrated (on-device) | Shared transport | Thunderbird wire protocol | ✅ Supported |
| External (serial) | USB/UART | — | 🔲 Planned |

### Camera

| Device | Interface | Protocol | Status |
|--------|-----------|----------|--------|
| Integrated (on-device) | Shared transport | Thunderbird wire protocol | ✅ Supported |
| USB cameras (V4L2) | USB | UVC | 🔲 Planned |

---

## Network Configuration

### Ethernet Connection (LiDAR / Fused Sensor)

```
┌──────────┐      Ethernet       ┌──────────────┐
│   Host   │◄───────────────────►│  Sensor Unit │
│ 192.168. │    1 Gbps            │  192.168.    │
│ 1.50     │    direct or switch  │  1.100:7500  │
└──────────┘                      └──────────────┘
```

**Step 1: Configure the host network interface**

```bash
# Linux — static IP on the sensor-facing NIC
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity
ping 192.168.1.100
```

**Step 2: Increase socket buffer size** (recommended for high-rate LiDAR)

```bash
# Temporary (until reboot)
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# Permanent
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_default=26214400" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

**Step 3: Connect with the SDK**

```cpp
thunderbird::DeviceConfig config;
config.uri = "eth://192.168.1.100:7500";

thunderbird::DeviceManager device(config);
auto status = device.connect();    // blocks until handshake
if (status != thunderbird::Status::OK) {
    // See error-codes.md for Status values
}
```

### USB Connection

```cpp
thunderbird::DeviceConfig config;
config.uri = "usb://0";           // first USB device

thunderbird::DeviceManager device(config);
device.connect();
```

**Linux USB permissions:**

```bash
# Option A: Run as root (not recommended for production)
sudo ./my_app

# Option B: udev rule (recommended)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="XXXX", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/99-thunderbird.rules
sudo udevadm control --reload-rules
```

---

## Firmware Requirements

| Sensor | Minimum Firmware | Recommended |
|--------|-----------------|-------------|
| Thunderbird Fused Sensor | 1.0.0 | Latest |
| Velodyne VLP-16 | 3.0.26 | 3.0.41+ |

Check firmware version after connection:

```cpp
auto info = device.device_info();
std::printf("FW: %s  Serial: %s  Model: %s\n",
            info.firmware_version.c_str(),
            info.serial_number.c_str(),
            info.model_name.c_str());
```

---

## Physical Mounting

### Coordinate Frame Convention

```
        Z (up)
        │
        │
        │
        └───── X (forward)
       /
      /
     Y (left)
```

- **Origin:** sensor optical center
- **X:** forward (direction of travel)
- **Y:** left (following right-hand rule)
- **Z:** up

### Recommended Mounting

| Platform | Mount Position | Notes |
|----------|---------------|-------|
| Car (roof) | Center-top, ≥ 1.8 m height | Minimize occlusion from body panels |
| Drone | Bottom-center or forward-tilt | Ensure ground visibility for SLAM |
| Robot (indoor) | Front, ~0.5 m height | Angle slightly downward for obstacle detection |

---

## Verifying the Hardware Connection

```bash
# Build and run the hardware verification example
./build/examples/basic_streaming

# With real hardware, set the URI:
# (modify basic_streaming.cpp or use your own code with config.uri)
```

After connecting, check device health:

```cpp
device.connect();
device.start();

// Wait for health data to accumulate
std::this_thread::sleep_for(std::chrono::seconds(2));

auto health = device.health_monitor()->snapshot();
std::printf("Health score: %.2f\n", health.health_score);
std::printf("LiDAR: %.1f Hz  IMU: %.1f Hz  Camera: %.1f FPS\n",
            health.lidar_hz, health.imu_hz, health.camera_fps);
```

If any rate is 0.0 Hz, see [Troubleshooting](../troubleshooting/troubleshooting.md).

---

## Next Steps

| What | Where |
|------|-------|
| Time synchronization | [Time Sync Guide](../guides/time-synchronization.md) |
| Health monitoring | [examples/health_monitor_demo.cpp](../../examples/health_monitor_demo.cpp) |
| Connection state machine | [examples/comm_layer_demo.cpp](../../examples/comm_layer_demo.cpp) |
| Advanced configuration | [Advanced Config Guide](../guides/advanced-configuration.md) |

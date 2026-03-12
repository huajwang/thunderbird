#!/usr/bin/env python3
"""
Thunderbird .tbrec → ROS 2 bag converter.

Reads a .tbrec recording using a built-in binary reader (no Thunderbird
Python bindings required) and writes a ROS 2 bag (SQLite3 storage) with
standard message types:

  /thunderbird/lidar/points     → sensor_msgs/msg/PointCloud2
  /thunderbird/imu/data         → sensor_msgs/msg/Imu
  /thunderbird/camera/image     → sensor_msgs/msg/Image

Usage:
    python tbrec_to_rosbag.py recording.tbrec [-o output_bag]

Requires:
    pip install rosbag2-py  (from ros2 packages or standalone)
"""

from __future__ import annotations

import argparse
import os
import struct
import sys

# ---------------------------------------------------------------------------
#  .tbrec binary format constants (from recording_format.h)
# ---------------------------------------------------------------------------
FILE_MAGIC = b"TBREC001"
FORMAT_VERSION = 1

SENSOR_LIDAR = 0
SENSOR_IMU = 1
SENSOR_CAMERA = 2

FILE_HEADER_FMT = "<8sII32s32s32sqqQ"
FILE_HEADER_SIZE = struct.calcsize(FILE_HEADER_FMT)

RECORD_HEADER_FMT = "<B3xqI"
RECORD_HEADER_SIZE = struct.calcsize(RECORD_HEADER_FMT)  # 16

LIDAR_REC_HEADER_FMT = "<II"
LIDAR_REC_HEADER_SIZE = struct.calcsize(LIDAR_REC_HEADER_FMT)

POINT_FMT = "<ffffi"
POINT_SIZE = struct.calcsize(POINT_FMT)  # 20

IMU_PAYLOAD_FMT = "<ffffff"
IMU_PAYLOAD_SIZE = struct.calcsize(IMU_PAYLOAD_FMT)  # 24

CAMERA_REC_HEADER_FMT = "<IIIB3xII"
CAMERA_REC_HEADER_SIZE = struct.calcsize(CAMERA_REC_HEADER_FMT)

PIXEL_FORMAT_NAMES = {0: "mono8", 1: "rgb8", 2: "bgr8", 3: "yuyv", 4: "nv12"}
PIXEL_FORMAT_BPP = {0: 1, 1: 3, 2: 3, 3: 2, 4: 1}

# ---------------------------------------------------------------------------
#  CDR serialisation helpers (ROS 2 wire format)
# ---------------------------------------------------------------------------

def _make_time(ns: int) -> bytes:
    sec = int(ns // 1_000_000_000)
    nsec = int(ns % 1_000_000_000)
    return struct.pack("<iI", sec, nsec)


def _make_header(ns: int, frame_id: str, seq: int = 0) -> bytes:
    frame_bytes = frame_id.encode("utf-8") + b"\x00"
    # Header: stamp (8) + frame_id (4+len+1 padded to 4)
    stamp = _make_time(ns)
    frame_len = len(frame_bytes)
    padding = (4 - (frame_len % 4)) % 4
    return stamp + struct.pack("<I", frame_len) + frame_bytes + (b"\x00" * padding)


def serialise_pointcloud2(
    timestamp_ns: int,
    frame_id: str,
    points_raw: bytes,
    point_count: int,
) -> bytes:
    """Build sensor_msgs/msg/PointCloud2 CDR payload."""
    # PointField descriptors: x, y, z, intensity
    #   name(str) offset(u32) datatype(u8) count(u32)
    FLOAT32 = 7
    fields = [
        (b"x\x00", 0, FLOAT32, 1),
        (b"y\x00", 4, FLOAT32, 1),
        (b"z\x00", 8, FLOAT32, 1),
        (b"intensity\x00", 12, FLOAT32, 1),
    ]

    point_step = 16  # 4 floats (skip timestamp_offset_ns for ROS compat)
    row_step = point_step * point_count
    height = 1
    width = point_count

    hdr = _make_header(timestamp_ns, frame_id)

    # Fields array
    fields_data = struct.pack("<I", len(fields))
    for name_b, off, dtype, cnt in fields:
        fields_data += struct.pack("<I", len(name_b)) + name_b
        # Pad name to 4-byte boundary
        pad = (4 - len(name_b) % 4) % 4
        fields_data += b"\x00" * pad
        # PointField: uint32 offset; uint8 datatype; 3 bytes padding; uint32 count
        fields_data += struct.pack("<IB3xI", off, dtype, cnt)

    # Build point data (strip timestamp_offset_ns from each 20-byte record)
    if point_count > 0:
        point_data = bytearray(point_step * point_count)
        for i in range(point_count):
            src_off = i * POINT_SIZE
            dst_off = i * point_step
            point_data[dst_off : dst_off + point_step] = points_raw[
                src_off : src_off + point_step
            ]
    else:
        point_data = b""

    body = struct.pack("<II", height, width)
    body += fields_data
    body += struct.pack("<?", False)  # is_bigendian
    body += b"\x00" * 3  # padding
    body += struct.pack("<II", point_step, row_step)
    body += struct.pack("<I", len(point_data)) + bytes(point_data)
    body += struct.pack("<?", True)  # is_dense
    body += b"\x00" * 3

    # CDR encapsulation header
    cdr = b"\x00\x01\x00\x00" + hdr + body
    return cdr


def serialise_imu(
    timestamp_ns: int,
    frame_id: str,
    accel: tuple[float, float, float],
    gyro: tuple[float, float, float],
) -> bytes:
    """Build sensor_msgs/msg/Imu CDR payload."""
    hdr = _make_header(timestamp_ns, frame_id)

    # Orientation: unknown → set covariance[0] = -1
    orientation = struct.pack("<dddd", 0.0, 0.0, 0.0, 1.0)
    orient_cov = struct.pack("<9d", -1.0, 0, 0, 0, 0, 0, 0, 0, 0)

    angular_vel = struct.pack("<ddd", float(gyro[0]), float(gyro[1]), float(gyro[2]))
    angular_cov = struct.pack("<9d", 0, 0, 0, 0, 0, 0, 0, 0, 0)

    linear_acc = struct.pack("<ddd", float(accel[0]), float(accel[1]), float(accel[2]))
    linear_cov = struct.pack("<9d", 0, 0, 0, 0, 0, 0, 0, 0, 0)

    cdr = b"\x00\x01\x00\x00" + hdr
    cdr += orientation + orient_cov
    cdr += angular_vel + angular_cov
    cdr += linear_acc + linear_cov
    return cdr


def serialise_image(
    timestamp_ns: int,
    frame_id: str,
    width: int,
    height: int,
    pixel_format: int,
    pixel_data: bytes,
) -> bytes:
    """Build sensor_msgs/msg/Image CDR payload."""
    hdr = _make_header(timestamp_ns, frame_id)

    encoding = PIXEL_FORMAT_NAMES.get(pixel_format, "mono8")
    enc_bytes = encoding.encode("utf-8") + b"\x00"
    bpp = PIXEL_FORMAT_BPP.get(pixel_format, 1)
    step = width * bpp

    body = struct.pack("<II", height, width)
    body += struct.pack("<I", len(enc_bytes)) + enc_bytes
    pad = (4 - len(enc_bytes) % 4) % 4
    body += b"\x00" * pad
    body += struct.pack("<?", False)  # is_bigendian
    body += b"\x00" * 3
    body += struct.pack("<I", step)
    body += struct.pack("<I", len(pixel_data)) + pixel_data

    cdr = b"\x00\x01\x00\x00" + hdr + body
    return cdr


# ---------------------------------------------------------------------------
#  Pure-Python .tbrec reader (no compiled bindings needed)
# ---------------------------------------------------------------------------

class TbrecReader:
    """Minimal .tbrec reader using struct-based parsing."""

    def __init__(self, path: str):
        self._f = open(path, "rb")
        try:
            raw = self._f.read(FILE_HEADER_SIZE)
            if len(raw) < FILE_HEADER_SIZE:
                raise ValueError("File too small for .tbrec header")

            parts = struct.unpack(FILE_HEADER_FMT, raw)
            magic = parts[0]
            if magic != FILE_MAGIC:
                raise ValueError(f"Bad magic: {magic!r}")

            self.version = parts[1]
            self.serial_number = parts[3].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
            self.firmware_version = parts[4].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
            self.model_name = parts[5].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
            self.start_ns = parts[6]
            self.stop_ns = parts[7]
            self.total_records = parts[8]
        except Exception:
            self._f.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._f.close()

    def records(self):
        """Yield (sensor_tag, timestamp_ns, payload_bytes) tuples."""
        while True:
            hdr_raw = self._f.read(RECORD_HEADER_SIZE)
            if len(hdr_raw) < RECORD_HEADER_SIZE:
                return
            sensor, ts, payload_sz = struct.unpack(RECORD_HEADER_FMT, hdr_raw)
            payload = self._f.read(payload_sz)
            if len(payload) < payload_sz:
                return
            yield sensor, ts, payload


# ---------------------------------------------------------------------------
#  Main converter
# ---------------------------------------------------------------------------

def convert(input_path: str, output_path: str) -> None:
    try:
        import rosbag2_py
    except ImportError:
        rosbag2_py = None

    if rosbag2_py is None:
        print(
            "rosbag2_py not available — writing raw CDR files as fallback.\n"
            "Install ros2 packages for native bag output.",
            file=sys.stderr,
        )
        _convert_raw(input_path, output_path)
        return

    _convert_rosbag(input_path, output_path)


def _convert_rosbag(input_path: str, output_path: str) -> None:
    """Convert using rosbag2_py for native ROS 2 bag output."""
    import rosbag2_py

    storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    # Create topics
    lidar_topic = rosbag2_py.TopicMetadata(
        name="/thunderbird/lidar/points",
        type="sensor_msgs/msg/PointCloud2",
        serialization_format="cdr",
    )
    imu_topic = rosbag2_py.TopicMetadata(
        name="/thunderbird/imu/data",
        type="sensor_msgs/msg/Imu",
        serialization_format="cdr",
    )
    camera_topic = rosbag2_py.TopicMetadata(
        name="/thunderbird/camera/image",
        type="sensor_msgs/msg/Image",
        serialization_format="cdr",
    )
    writer.create_topic(lidar_topic)
    writer.create_topic(imu_topic)
    writer.create_topic(camera_topic)

    counts = {"lidar": 0, "imu": 0, "camera": 0}

    with TbrecReader(input_path) as reader:
        print(f"Device: {reader.model_name}  S/N: {reader.serial_number}")
        print(f"Records: {reader.total_records}")

        for sensor, ts, payload in reader.records():
            if sensor == SENSOR_LIDAR:
                seq, n_pts = struct.unpack(LIDAR_REC_HEADER_FMT, payload[:LIDAR_REC_HEADER_SIZE])
                pts_raw = payload[LIDAR_REC_HEADER_SIZE:]
                msg = serialise_pointcloud2(ts, "thunderbird_lidar", pts_raw, n_pts)
                writer.write("/thunderbird/lidar/points", msg, ts)
                counts["lidar"] += 1

            elif sensor == SENSOR_IMU:
                vals = struct.unpack(IMU_PAYLOAD_FMT, payload[:IMU_PAYLOAD_SIZE])
                accel = vals[0:3]
                gyro = vals[3:6]
                msg = serialise_imu(ts, "thunderbird_imu", accel, gyro)
                writer.write("/thunderbird/imu/data", msg, ts)
                counts["imu"] += 1

            elif sensor == SENSOR_CAMERA:
                parts = struct.unpack(
                    CAMERA_REC_HEADER_FMT, payload[:CAMERA_REC_HEADER_SIZE]
                )
                w, h, stride, pf, seq, pix_sz = parts
                pix_data = payload[CAMERA_REC_HEADER_SIZE : CAMERA_REC_HEADER_SIZE + pix_sz]

                # Packed image buffer.  Bytes-per-pixel must come from the
                # pixel format, not from the stride (which may include padding).
                if w > 0 and h > 0 and stride > 0:
                    bpp = PIXEL_FORMAT_BPP.get(pf, 1)
                    expected_row_bytes = w * bpp
                    frame_bytes = stride * h

                    # Only strip padding when the declared payload size matches
                    # stride * height and stride is larger than the actual row
                    # size implied by the pixel format.
                    if (
                        bpp > 0
                        and expected_row_bytes > 0
                        and pix_sz == frame_bytes
                        and stride > expected_row_bytes
                    ):
                        pix_data = b"".join(
                            pix_data[r * stride : r * stride + expected_row_bytes]
                            for r in range(h)
                        )

                msg = serialise_image(ts, "thunderbird_camera", w, h, pf, pix_data)
                writer.write("/thunderbird/camera/image", msg, ts)
                counts["camera"] += 1

    print(f"\nWrote {output_path}")
    print(f"  LiDAR frames:  {counts['lidar']}")
    print(f"  IMU frames:    {counts['imu']}")
    print(f"  Camera frames: {counts['camera']}")


def _convert_raw(input_path: str, output_path: str) -> None:
    """Fallback: write individual CDR files when rosbag2 isn't available."""
    os.makedirs(output_path, exist_ok=True)
    counts = {"lidar": 0, "imu": 0, "camera": 0}

    with TbrecReader(input_path) as reader:
        print(f"Device: {reader.model_name}  S/N: {reader.serial_number}")
        print(f"Records: {reader.total_records}")

        for sensor, ts, payload in reader.records():
            if sensor == SENSOR_LIDAR:
                seq, n_pts = struct.unpack(LIDAR_REC_HEADER_FMT, payload[:LIDAR_REC_HEADER_SIZE])
                pts_raw = payload[LIDAR_REC_HEADER_SIZE:]
                msg = serialise_pointcloud2(ts, "thunderbird_lidar", pts_raw, n_pts)
                out = os.path.join(output_path, f"lidar_{counts['lidar']:06d}.cdr")
                with open(out, "wb") as f:
                    f.write(msg)
                counts["lidar"] += 1

            elif sensor == SENSOR_IMU:
                vals = struct.unpack(IMU_PAYLOAD_FMT, payload[:IMU_PAYLOAD_SIZE])
                msg = serialise_imu(ts, "thunderbird_imu", vals[0:3], vals[3:6])
                out = os.path.join(output_path, f"imu_{counts['imu']:06d}.cdr")
                with open(out, "wb") as f:
                    f.write(msg)
                counts["imu"] += 1

            elif sensor == SENSOR_CAMERA:
                parts = struct.unpack(
                    CAMERA_REC_HEADER_FMT, payload[:CAMERA_REC_HEADER_SIZE]
                )
                w, h, stride, pf, seq, pix_sz = parts
                pix_data = payload[CAMERA_REC_HEADER_SIZE : CAMERA_REC_HEADER_SIZE + pix_sz]

                # Re-pack rows to remove any padding (see rosbag path above).
                if w > 0 and h > 0 and stride > 0:
                    bpp = PIXEL_FORMAT_BPP.get(pf, 1)
                    expected_row_bytes = w * bpp
                    frame_bytes = stride * h

                    if (
                        bpp > 0
                        and expected_row_bytes > 0
                        and pix_sz == frame_bytes
                        and stride > expected_row_bytes
                    ):
                        pix_data = b"".join(
                            pix_data[r * stride : r * stride + expected_row_bytes]
                            for r in range(h)
                        )

                msg = serialise_image(ts, "thunderbird_camera", w, h, pf, pix_data)
                out = os.path.join(output_path, f"camera_{counts['camera']:06d}.cdr")
                with open(out, "wb") as f:
                    f.write(msg)
                counts["camera"] += 1

    print(f"\nWrote {len(os.listdir(output_path))} files to {output_path}/")
    print(f"  LiDAR frames:  {counts['lidar']}")
    print(f"  IMU frames:    {counts['imu']}")
    print(f"  Camera frames: {counts['camera']}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Thunderbird .tbrec recording to ROS 2 bag"
    )
    parser.add_argument("input", help="Path to .tbrec file")
    parser.add_argument(
        "-o", "--output", default=None, help="Output bag path (default: <input>_bag)"
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: {args.input} not found", file=sys.stderr)
        sys.exit(1)

    output = args.output or (os.path.splitext(args.input)[0] + "_bag")
    convert(args.input, output)


if __name__ == "__main__":
    main()

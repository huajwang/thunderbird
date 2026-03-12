#!/usr/bin/env python3
"""
Thunderbird .tbrec → PCD + PNG extractor.

Reads a .tbrec recording and extracts:
  - LiDAR frames → individual ASCII .pcd files
  - Camera frames → individual .png files
  - timestamps.csv index file (for OpenCalib and other tools)

Usage:
    python tbrec_to_pcd.py recording.tbrec [-o output_dir]

Requires:
    Only Python stdlib (+ optional PIL/Pillow for PNG compression).
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys

# ---------------------------------------------------------------------------
#  .tbrec binary format constants (from recording_format.h)
# ---------------------------------------------------------------------------
FILE_MAGIC = b"TBREC001"

SENSOR_LIDAR = 0
SENSOR_IMU = 1
SENSOR_CAMERA = 2

FILE_HEADER_FMT = "<8sII32s32s32sqqQ"
FILE_HEADER_SIZE = struct.calcsize(FILE_HEADER_FMT)

RECORD_HEADER_FMT = "<B3xqI"
RECORD_HEADER_SIZE = struct.calcsize(RECORD_HEADER_FMT)

LIDAR_REC_HEADER_FMT = "<II"
LIDAR_REC_HEADER_SIZE = struct.calcsize(LIDAR_REC_HEADER_FMT)

POINT_FMT = "<ffffi"
POINT_SIZE = struct.calcsize(POINT_FMT)

IMU_PAYLOAD_FMT = "<ffffff"
IMU_PAYLOAD_SIZE = struct.calcsize(IMU_PAYLOAD_FMT)

CAMERA_REC_HEADER_FMT = "<IIIB3xII"
CAMERA_REC_HEADER_SIZE = struct.calcsize(CAMERA_REC_HEADER_FMT)

PIXEL_FORMAT_BPP = {0: 1, 1: 3, 2: 3, 3: 2, 4: 1}


# ---------------------------------------------------------------------------
#  .tbrec reader (same as tbrec_to_rosbag.py — self-contained for no deps)
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
            if parts[0] != FILE_MAGIC:
                raise ValueError(f"Bad magic: {parts[0]!r}")

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
#  PCD writer (ASCII format, compatible with PCL / Open3D / CloudCompare)
# ---------------------------------------------------------------------------

def write_pcd(path: str, points_raw: bytes, point_count: int) -> None:
    """Write an ASCII PCD file with fields x y z intensity."""
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {point_count}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {point_count}\n")
        f.write("DATA ascii\n")
        for i in range(point_count):
            off = i * POINT_SIZE
            x, y, z, intensity, _ = struct.unpack(
                POINT_FMT, points_raw[off : off + POINT_SIZE]
            )
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.3f}\n")


def write_pcd_binary(path: str, points_raw: bytes, point_count: int) -> None:
    """Write a binary PCD file (much faster for large clouds)."""
    with open(path, "wb") as f:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {point_count}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {point_count}\n"
            "DATA binary\n"
        )
        f.write(header.encode("ascii"))
        # Write point data (strip timestamp_offset_ns — keep only x,y,z,intensity)
        for i in range(point_count):
            off = i * POINT_SIZE
            f.write(points_raw[off : off + 16])  # 4 floats = 16 bytes


# ---------------------------------------------------------------------------
#  Image writer
# ---------------------------------------------------------------------------

def write_image(path: str, width: int, height: int, pixel_format: int, data: bytes) -> None:
    """Write camera frame as PNG (via Pillow) or raw PPM/PGM fallback."""
    bpp = PIXEL_FORMAT_BPP.get(pixel_format, 1)

    try:
        from PIL import Image  # type: ignore

        if pixel_format == 0:  # Mono8
            img = Image.frombytes("L", (width, height), data)
        elif pixel_format == 1:  # RGB8
            img = Image.frombytes("RGB", (width, height), data)
        elif pixel_format == 2:  # BGR8
            img = Image.frombytes("RGB", (width, height), data)
            # Swap R and B channels
            r, g, b = img.split()
            img = Image.merge("RGB", (b, g, r))
        else:
            # YUYV/NV12 and other non-RGB formats — save as raw bytes.
            _write_raw_dump(path, data)
            return

        img.save(path)
        return
    except ImportError:
        pass

    # Fallback: write PPM/PGM for RGB/mono formats; raw dump otherwise.
    if pixel_format in (0, 1, 2):
        _write_raw_image(path, width, height, bpp, data)
    else:
        _write_raw_dump(path, data)


def _write_raw_image(path: str, width: int, height: int, bpp: int, data: bytes) -> None:
    """Write image as PPM (color) or PGM (grayscale) — no dependencies."""
    ext = ".pgm" if bpp == 1 else ".ppm"
    out_path = os.path.splitext(path)[0] + ext
    with open(out_path, "wb") as f:
        if bpp == 1:
            f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        else:
            f.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        f.write(data[: width * height * bpp])


def _write_raw_dump(path: str, data: bytes) -> None:
    """Write bytes to a raw dump for unsupported pixel formats."""
    out_path = os.path.splitext(path)[0] + ".bin"
    with open(out_path, "wb") as f:
        f.write(data)


# ---------------------------------------------------------------------------
#  Main converter
# ---------------------------------------------------------------------------

def extract(input_path: str, output_dir: str, binary_pcd: bool = False) -> None:
    lidar_dir = os.path.join(output_dir, "lidar")
    camera_dir = os.path.join(output_dir, "camera")
    imu_dir = os.path.join(output_dir, "imu")
    os.makedirs(lidar_dir, exist_ok=True)
    os.makedirs(camera_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)

    ts_path = os.path.join(output_dir, "timestamps.csv")
    counts = {"lidar": 0, "imu": 0, "camera": 0}

    with TbrecReader(input_path) as reader, open(ts_path, "w", newline="") as ts_file:
        ts_writer = csv.writer(ts_file)
        ts_writer.writerow(["sensor", "index", "timestamp_ns", "filename"])

        print(f"Device: {reader.model_name}  S/N: {reader.serial_number}")
        print(f"Records: {reader.total_records}")

        for sensor, ts, payload in reader.records():
            if sensor == SENSOR_LIDAR:
                idx = counts["lidar"]
                seq, n_pts = struct.unpack(
                    LIDAR_REC_HEADER_FMT, payload[:LIDAR_REC_HEADER_SIZE]
                )
                pts_raw = payload[LIDAR_REC_HEADER_SIZE:]
                fname = f"{idx:06d}.pcd"
                pcd_path = os.path.join(lidar_dir, fname)

                if binary_pcd:
                    write_pcd_binary(pcd_path, pts_raw, n_pts)
                else:
                    write_pcd(pcd_path, pts_raw, n_pts)

                ts_writer.writerow(["lidar", idx, ts, f"lidar/{fname}"])
                counts["lidar"] += 1

            elif sensor == SENSOR_IMU:
                idx = counts["imu"]
                vals = struct.unpack(IMU_PAYLOAD_FMT, payload[:IMU_PAYLOAD_SIZE])
                fname = f"{idx:06d}.csv"
                imu_path = os.path.join(imu_dir, fname)
                with open(imu_path, "w") as f:
                    f.write("accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
                    f.write(",".join(f"{v:.8f}" for v in vals) + "\n")
                ts_writer.writerow(["imu", idx, ts, f"imu/{fname}"])
                counts["imu"] += 1

            elif sensor == SENSOR_CAMERA:
                idx = counts["camera"]
                parts = struct.unpack(
                    CAMERA_REC_HEADER_FMT, payload[:CAMERA_REC_HEADER_SIZE]
                )
                w, h, stride, pf, seq, pix_sz = parts
                pix_data = payload[
                    CAMERA_REC_HEADER_SIZE : CAMERA_REC_HEADER_SIZE + pix_sz
                ]

                # Re-pack rows to remove any row padding so write_image()
                # (and the PPM/PGM fallback) see a tightly packed buffer.
                if w > 0 and h > 0 and stride > 0:
                    bpp = PIXEL_FORMAT_BPP.get(pf, 1)
                    expected_row_bytes = w * bpp
                    frame_bytes = stride * h

                    # Only strip padding when the declared payload size
                    # matches stride * height (confirming stride-based
                    # layout) and stride exceeds the actual row size.
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

                fname = f"{idx:06d}.png"
                img_path = os.path.join(camera_dir, fname)
                write_image(img_path, w, h, pf, pix_data)
                # Check if fallback format was used
                actual_name = fname
                alt = os.path.splitext(fname)[0] + ".pgm"
                alt2 = os.path.splitext(fname)[0] + ".ppm"
                alt3 = os.path.splitext(fname)[0] + ".bin"
                if os.path.exists(os.path.join(camera_dir, alt)):
                    actual_name = alt
                elif os.path.exists(os.path.join(camera_dir, alt2)):
                    actual_name = alt2
                elif os.path.exists(os.path.join(camera_dir, alt3)):
                    actual_name = alt3
                ts_writer.writerow(["camera", idx, ts, f"camera/{actual_name}"])
                counts["camera"] += 1

    print(f"\nOutput: {output_dir}/")
    print(f"  LiDAR .pcd files:  {counts['lidar']}  (in lidar/)")
    print(f"  Camera images:     {counts['camera']}  (in camera/)")
    print(f"  IMU samples:       {counts['imu']}  (in imu/)")
    print(f"  Index: timestamps.csv")


def main():
    parser = argparse.ArgumentParser(
        description="Extract Thunderbird .tbrec recording to PCD + PNG files"
    )
    parser.add_argument("input", help="Path to .tbrec file")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output directory (default: <input>_extracted)",
    )
    parser.add_argument(
        "--binary-pcd",
        action="store_true",
        help="Write binary PCD files instead of ASCII (faster, smaller)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: {args.input} not found", file=sys.stderr)
        sys.exit(1)

    output = args.output or (os.path.splitext(args.input)[0] + "_extracted")
    extract(args.input, output, binary_pcd=args.binary_pcd)


if __name__ == "__main__":
    main()

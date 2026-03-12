#!/usr/bin/env python3
"""
Tests for .tbrec converter tools (Phase 7).

Creates a synthetic .tbrec file and verifies both converters parse it correctly.
No compiled bindings or ROS 2 required — uses the pure-Python readers only.
"""
import os
import struct
import sys
import tempfile

# Add tools directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python", "tools"))

import tbrec_to_rosbag as rosbag_conv
import tbrec_to_pcd as pcd_conv


# ── .tbrec synthetic writer ─────────────────────────────────────────────────

FILE_MAGIC = b"TBREC001"
FILE_HEADER_FMT = "<8sII32s32s32sqqQ"
RECORD_HEADER_FMT = "<B3xqI"
LIDAR_REC_HEADER_FMT = "<II"
POINT_FMT = "<ffffi"
IMU_PAYLOAD_FMT = "<ffffff"
CAMERA_REC_HEADER_FMT = "<IIIB3xII"


def write_test_tbrec(path, n_lidar=3, n_imu=10, n_camera=2):
    """Write a synthetic .tbrec file with known content."""
    records = []

    # LiDAR records
    for i in range(n_lidar):
        ts = 1000000 * (i + 1)  # 1ms, 2ms, 3ms
        n_pts = 5
        lidar_hdr = struct.pack(LIDAR_REC_HEADER_FMT, i, n_pts)
        pts = b""
        for p in range(n_pts):
            pts += struct.pack(POINT_FMT,
                               float(p), float(i), 0.0, 1.0, p * 100)
        payload = lidar_hdr + pts
        rec_hdr = struct.pack(RECORD_HEADER_FMT, 0, ts, len(payload))
        records.append((ts, rec_hdr + payload))

    # IMU records
    for i in range(n_imu):
        ts = 500000 + 200000 * i  # 0.5ms + 0.2ms*i
        payload = struct.pack(IMU_PAYLOAD_FMT,
                              0.0, 0.0, 9.81, 0.01, 0.02, 0.03)
        rec_hdr = struct.pack(RECORD_HEADER_FMT, 1, ts, len(payload))
        records.append((ts, rec_hdr + payload))

    # Camera records
    for i in range(n_camera):
        ts = 1500000 * (i + 1)
        w, h = 4, 3
        pix_format = 0  # Mono8
        pix_data = bytes(range(w * h))
        cam_hdr = struct.pack(CAMERA_REC_HEADER_FMT,
                              w, h, w, pix_format, i, len(pix_data))
        payload = cam_hdr + pix_data
        rec_hdr = struct.pack(RECORD_HEADER_FMT, 2, ts, len(payload))
        records.append((ts, rec_hdr + payload))

    # Sort by timestamp
    records.sort(key=lambda r: r[0])

    total_records = len(records)
    start_ts = records[0][0]
    stop_ts = records[-1][0]

    file_hdr = struct.pack(FILE_HEADER_FMT,
                           FILE_MAGIC, 1, 0,
                           b"SN001\x00" + b"\x00" * 26,
                           b"1.0.0\x00" + b"\x00" * 26,
                           b"TestModel\x00" + b"\x00" * 22,
                           start_ts, stop_ts, total_records)

    with open(path, "wb") as f:
        f.write(file_hdr)
        for _, data in records:
            f.write(data)


# ── Tests ────────────────────────────────────────────────────────────────────

def test_tbrec_reader():
    """TbrecReader correctly parses header and records."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        path = tmp.name
    try:
        write_test_tbrec(path, n_lidar=3, n_imu=10, n_camera=2)

        with rosbag_conv.TbrecReader(path) as reader:
            assert reader.model_name == "TestModel", f"Got: {reader.model_name}"
            assert reader.serial_number == "SN001"
            assert reader.total_records == 15  # 3 + 10 + 2

            counts = {0: 0, 1: 0, 2: 0}
            for sensor, ts, payload in reader.records():
                assert sensor in (0, 1, 2), f"Bad sensor tag: {sensor}"
                assert ts > 0
                assert len(payload) > 0
                counts[sensor] += 1

            assert counts[0] == 3, f"LiDAR: {counts[0]}"
            assert counts[1] == 10, f"IMU: {counts[1]}"
            assert counts[2] == 2, f"Camera: {counts[2]}"

        print("  TbrecReader              OK")
    finally:
        os.unlink(path)


def test_tbrec_reader_bad_magic():
    """TbrecReader rejects files with bad magic."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tmp.write(b"BADMAGIC" + b"\x00" * 144)
        path = tmp.name
    try:
        raised = False
        try:
            rosbag_conv.TbrecReader(path)
        except ValueError:
            raised = True
        assert raised, "Should have raised ValueError"
        print("  Bad magic rejection      OK")
    finally:
        os.unlink(path)


def test_pointcloud2_serialisation():
    """PointCloud2 CDR payload has correct structure."""
    pts = struct.pack(POINT_FMT, 1.0, 2.0, 3.0, 0.5, 100) * 2
    cdr = rosbag_conv.serialise_pointcloud2(1000000, "lidar", pts, 2)
    assert len(cdr) > 0
    # CDR encapsulation: first 4 bytes = 00 01 00 00
    assert cdr[:4] == b"\x00\x01\x00\x00"
    print("  PointCloud2 CDR          OK")


def test_imu_serialisation():
    """IMU CDR payload has correct structure."""
    cdr = rosbag_conv.serialise_imu(1000000, "imu", (0, 0, 9.81), (0.1, 0.2, 0.3))
    assert len(cdr) > 0
    assert cdr[:4] == b"\x00\x01\x00\x00"
    print("  Imu CDR                  OK")


def test_image_serialisation():
    """Image CDR payload has correct structure."""
    pix = bytes([128] * 12)  # 4x3 mono8
    cdr = rosbag_conv.serialise_image(1000000, "cam", 4, 3, 0, pix)
    assert len(cdr) > 0
    assert cdr[:4] == b"\x00\x01\x00\x00"
    print("  Image CDR                OK")


def test_pcd_extractor():
    """PCD extractor creates correct directory structure."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tbrec_path = tmp.name
    out_dir = tbrec_path + "_extracted"
    try:
        write_test_tbrec(tbrec_path, n_lidar=2, n_imu=5, n_camera=1)
        pcd_conv.extract(tbrec_path, out_dir, binary_pcd=False)

        # Check directory structure
        assert os.path.isdir(os.path.join(out_dir, "lidar"))
        assert os.path.isdir(os.path.join(out_dir, "camera"))
        assert os.path.isdir(os.path.join(out_dir, "imu"))
        assert os.path.isfile(os.path.join(out_dir, "timestamps.csv"))

        # Check PCD files
        pcd_files = os.listdir(os.path.join(out_dir, "lidar"))
        assert len(pcd_files) == 2, f"Expected 2, got {len(pcd_files)}"
        assert "000000.pcd" in pcd_files

        # Verify PCD content
        with open(os.path.join(out_dir, "lidar", "000000.pcd")) as f:
            content = f.read()
            assert "VERSION 0.7" in content
            assert "FIELDS x y z intensity" in content
            assert "POINTS 5" in content

        # Check IMU files
        imu_files = os.listdir(os.path.join(out_dir, "imu"))
        assert len(imu_files) == 5

        # Check timestamp index
        with open(os.path.join(out_dir, "timestamps.csv")) as f:
            lines = f.readlines()
            assert lines[0].strip() == "sensor,index,timestamp_ns,filename"
            data_lines = [l for l in lines[1:] if l.strip()]
            assert len(data_lines) == 8  # 2 lidar + 5 imu + 1 camera

        print("  PCD extractor            OK")
    finally:
        os.unlink(tbrec_path)
        import shutil
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)


def test_pcd_binary_mode():
    """Binary PCD writer produces valid binary PCD."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tbrec_path = tmp.name
    out_dir = tbrec_path + "_bin"
    try:
        write_test_tbrec(tbrec_path, n_lidar=1, n_imu=0, n_camera=0)
        pcd_conv.extract(tbrec_path, out_dir, binary_pcd=True)

        pcd_path = os.path.join(out_dir, "lidar", "000000.pcd")
        with open(pcd_path, "rb") as f:
            header = b""
            while True:
                line = f.readline()
                header += line
                if line.startswith(b"DATA binary"):
                    break
            # After header, remaining bytes should be 5 points * 16 bytes
            data = f.read()
            assert len(data) == 5 * 16, f"Expected 80 bytes, got {len(data)}"

        print("  PCD binary mode          OK")
    finally:
        os.unlink(tbrec_path)
        import shutil
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)


def test_raw_fallback_converter():
    """Raw CDR fallback writes individual files."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tbrec_path = tmp.name
    out_dir = tbrec_path + "_raw"
    try:
        write_test_tbrec(tbrec_path, n_lidar=1, n_imu=2, n_camera=1)
        rosbag_conv._convert_raw(tbrec_path, out_dir)

        files = os.listdir(out_dir)
        lidar_files = [f for f in files if f.startswith("lidar_")]
        imu_files = [f for f in files if f.startswith("imu_")]
        camera_files = [f for f in files if f.startswith("camera_")]

        assert len(lidar_files) == 1
        assert len(imu_files) == 2
        assert len(camera_files) == 1

        print("  Raw CDR fallback         OK")
    finally:
        os.unlink(tbrec_path)
        import shutil
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)


def test_lidar_point_data_integrity():
    """Verify extracted PCD point values match what was written."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tbrec_path = tmp.name
    out_dir = tbrec_path + "_verify"
    try:
        write_test_tbrec(tbrec_path, n_lidar=1, n_imu=0, n_camera=0)
        pcd_conv.extract(tbrec_path, out_dir, binary_pcd=False)

        pcd_path = os.path.join(out_dir, "lidar", "000000.pcd")
        with open(pcd_path) as f:
            lines = f.readlines()
        # Find DATA line, points follow
        data_idx = next(i for i, l in enumerate(lines) if l.startswith("DATA"))
        point_lines = [l.strip() for l in lines[data_idx+1:] if l.strip()]
        assert len(point_lines) == 5

        # First point: x=0, y=0, z=0, intensity=1.0
        vals = point_lines[0].split()
        assert abs(float(vals[0]) - 0.0) < 1e-4  # x
        assert abs(float(vals[1]) - 0.0) < 1e-4  # y (frame 0)
        assert abs(float(vals[3]) - 1.0) < 1e-4  # intensity

        print("  LiDAR data integrity     OK")
    finally:
        os.unlink(tbrec_path)
        import shutil
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)


def test_camera_stride_padding():
    """Camera frames with row padding (stride > width*bpp) are correctly re-packed."""
    with tempfile.NamedTemporaryFile(suffix=".tbrec", delete=False) as tmp:
        tbrec_path = tmp.name
    out_dir = tbrec_path + "_padded"
    try:
        # Build a .tbrec manually with a single padded camera frame.
        # Mono8: bpp=1, width=4, but stride=8 (4 bytes of padding per row).
        w, h = 4, 3
        stride = 8  # 4 bytes of content + 4 bytes padding per row
        pix_format = 0  # Mono8
        # Pixel data with known values + padding garbage per row
        rows = []
        for r in range(h):
            content = bytes([10 * (r + 1) + c for c in range(w)])  # 10,11,12,13 / 20,21,22,23 / 30,31,32,33
            padding = bytes([0xFF] * (stride - w))  # 4 garbage bytes
            rows.append(content + padding)
        pix_data = b"".join(rows)  # stride * h = 24 bytes total

        cam_hdr = struct.pack(CAMERA_REC_HEADER_FMT,
                              w, h, stride, pix_format, 0, len(pix_data))
        payload = cam_hdr + pix_data
        ts = 1000000
        rec_hdr = struct.pack(RECORD_HEADER_FMT, 2, ts, len(payload))

        file_hdr = struct.pack(FILE_HEADER_FMT,
                               FILE_MAGIC, 1, 0,
                               b"SN001\x00" + b"\x00" * 26,
                               b"1.0.0\x00" + b"\x00" * 26,
                               b"TestModel\x00" + b"\x00" * 22,
                               ts, ts, 1)

        with open(tbrec_path, "wb") as f:
            f.write(file_hdr)
            f.write(rec_hdr + payload)

        # Extract using PCD extractor
        pcd_conv.extract(tbrec_path, out_dir, binary_pcd=False)

        # Find the extracted camera image
        camera_dir = os.path.join(out_dir, "camera")
        assert os.path.isdir(camera_dir), "camera dir not created"
        files = os.listdir(camera_dir)
        assert len(files) == 1, f"Expected 1 camera file, got {len(files)}"

        # Read the image and verify it has the right size (no padding bytes)
        img_path = os.path.join(camera_dir, files[0])
        with open(img_path, "rb") as f:
            data = f.read()

        # PGM/PNG output should contain the content pixels but not padding.
        # For PGM ASCII: header + w*h pixel values.  For PNG: decompressed
        # payload should be w*h bytes.  We just verify the 0xFF padding
        # garbage doesn't appear in the image portion.
        #
        # For Mono8 with these small values (10-33), the 0xFF padding bytes
        # should not appear if re-packing was correct.
        # Check file is non-empty and reasonable size
        assert len(data) > 0, "image file is empty"
        # For PGM files, verify the raw pixel area doesn't contain 0xFF
        if files[0].endswith(".pgm"):
            # PGM P5: header lines, then raw pixel data
            lines = data.split(b"\n", 3)  # P5 / W H / maxval / data
            raw_pixels = lines[-1] if len(lines) >= 4 else data
            # The tightly packed image should be w*h = 12 bytes
            assert len(raw_pixels) == w * h, f"Expected {w*h} pixel bytes, got {len(raw_pixels)}"
            assert 0xFF not in raw_pixels, "padding byte 0xFF found in output pixels"

        print("  Camera stride padding    OK")
    finally:
        os.unlink(tbrec_path)
        import shutil
        if os.path.isdir(out_dir):
            shutil.rmtree(out_dir)


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("TbrecConverters:")
    test_tbrec_reader()
    test_tbrec_reader_bad_magic()
    test_pointcloud2_serialisation()
    test_imu_serialisation()
    test_image_serialisation()
    test_pcd_extractor()
    test_pcd_binary_mode()
    test_raw_fallback_converter()
    test_lidar_point_data_integrity()
    test_camera_stride_padding()
    print("TbrecConverters: ALL TESTS PASSED")

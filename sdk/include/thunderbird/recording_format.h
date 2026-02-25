// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Recording File Format (Step 5)
// ─────────────────────────────────────────────────────────────────────────────
//
// Binary file layout for recording and replaying multi-sensor sessions.
//
// ┌──────────────────────────────────────────────────────────────────────┐
// │                          File Header                                │
// │  magic[8] + version(u32) + flags(u32) + DeviceInfo + timestamps     │
// ├──────────────────────────────────────────────────────────────────────┤
// │  Frame Record 0:  RecordHeader + payload bytes                      │
// ├──────────────────────────────────────────────────────────────────────┤
// │  Frame Record 1:  RecordHeader + payload bytes                      │
// ├──────────────────────────────────────────────────────────────────────┤
// │  ...                                                                │
// ├──────────────────────────────────────────────────────────────────────┤
// │  Frame Record N:  RecordHeader + payload bytes                      │
// └──────────────────────────────────────────────────────────────────────┘
//
// Design rationale:
//
//   • **Sequential append-only**: frames are written strictly in arrival
//     order.  This makes recording zero-allocation on the write path
//     (just fwrite) and supports streaming to disk without seeking.
//
//   • **Self-describing**: each record carries its sensor type, timestamp,
//     and payload length, so the reader can skip record types it doesn't
//     care about and detect corruption (unexpected EOF, wrong magic).
//
//   • **Fixed-size headers**: both the file header and per-record headers
//     are POD structs with known sizes, making them trivial to read/write
//     with a single fread/fwrite.  Variable-length data (pixel buffers,
//     point arrays) follow immediately.
//
//   • **Payload layout**: each sensor type defines a small fixed "sub-header"
//     followed by variable-length data:
//     - LiDAR:  LidarRecordHeader + n × PointXYZIT
//     - IMU:    ImuRecordPayload  (fixed 28 bytes, no variable part)
//     - Camera: CameraRecordHeader + pixel bytes
//
//   • **Extensibility**: the version field (currently 1) and the `flags`
//     bitmap allow future additions (compression, encryption, SyncedFrame
//     records, index tables) without breaking old readers.
//
//   • **No alignment padding**: structs are `#pragma pack(push,1)` to
//     guarantee on-disk layout matches in-memory layout regardless of
//     platform ABI.
//
//   • **Endianness**: little-endian assumed (x86/ARM).  A future version
//     could add a byte-order flag if portability is required.
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>

namespace thunderbird::recording {

// ─── Magic & version ────────────────────────────────────────────────────────

/// 8-byte magic at the start of every recording file.
inline constexpr char kFileMagic[8] = {'T','B','R','E','C','0','0','1'};

/// Current format version.  Bump when the on-disk layout changes.
inline constexpr uint32_t kFormatVersion = 1;

// ─── Sensor tag ─────────────────────────────────────────────────────────────

enum class SensorTag : uint8_t {
    LiDAR  = 0,
    IMU    = 1,
    Camera = 2,
};

// ─── File header ────────────────────────────────────────────────────────────

#pragma pack(push, 1)

struct FileHeader {
    char     magic[8];               ///< must equal kFileMagic
    uint32_t version;                ///< kFormatVersion
    uint32_t flags;                  ///< reserved (0)

    // Device metadata (fixed-size for simplicity)
    char     serial_number[32];      ///< null-terminated
    char     firmware_version[32];   ///< null-terminated
    char     model_name[32];         ///< null-terminated

    int64_t  start_timestamp_ns;     ///< first frame timestamp (ns)
    int64_t  stop_timestamp_ns;      ///< last frame timestamp  (ns, filled on close)
    uint64_t total_records;          ///< total frame count (filled on close)
};

static_assert(sizeof(FileHeader) == 8 + 4 + 4 + 32*3 + 8 + 8 + 8,
              "FileHeader must be tightly packed");

// ─── Per-record header ──────────────────────────────────────────────────────

struct RecordHeader {
    SensorTag sensor;                ///< which sensor produced this record
    uint8_t   reserved[3];          ///< alignment padding (zero)
    int64_t   timestamp_ns;          ///< hardware timestamp of the frame
    uint32_t  payload_size;          ///< bytes of payload following this header
};

static_assert(sizeof(RecordHeader) == 1 + 3 + 8 + 4,
              "RecordHeader must be 16 bytes");

// ─── LiDAR payload ─────────────────────────────────────────────────────────

/// Fixed sub-header before the point array.
struct LidarRecordHeader {
    uint32_t sequence;               ///< LidarFrame::sequence
    uint32_t point_count;            ///< number of PointXYZIT structs following
};

/// On-disk point layout (matches data::PointXYZIT exactly).
struct PointRecord {
    float   x;
    float   y;
    float   z;
    float   intensity;
    int32_t timestamp_offset_ns;
};

static_assert(sizeof(PointRecord) == 20, "PointRecord must be 20 bytes");

// ─── IMU payload ────────────────────────────────────────────────────────────

/// Complete IMU record (no variable part).
struct ImuRecordPayload {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
};

static_assert(sizeof(ImuRecordPayload) == 24, "ImuRecordPayload must be 24 bytes");

// ─── Camera payload ─────────────────────────────────────────────────────────

struct CameraRecordHeader {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint8_t  pixel_format;           ///< data::PixelFormat cast to uint8_t
    uint8_t  reserved[3];
    uint32_t sequence;
    uint32_t pixel_data_size;        ///< bytes of pixel data following
};

static_assert(sizeof(CameraRecordHeader) == 4+4+4+1+3+4+4,
              "CameraRecordHeader must be 24 bytes");

#pragma pack(pop)

} // namespace thunderbird::recording

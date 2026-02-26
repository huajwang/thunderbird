// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Python Bindings (pybind11)
// ─────────────────────────────────────────────────────────────────────────────
//
// Exposes the core SDK surface to Python as the `_spatial_sdk_core` extension
// module.  The pure-Python wrapper `spatial_sdk/__init__.py` re-exports
// everything with docstrings and convenience helpers.
//
// Key design decisions:
//   • LiDAR points are exposed as NumPy structured arrays (zero-copy view
//     while the C++ frame is alive, copied into an owned array on return
//     so the user never holds a dangling reference).
//   • Camera pixel buffers are exposed as NumPy uint8 arrays with proper
//     shape (H×W×C or H×W for mono).
//   • IMU accel/gyro are exposed as length-3 NumPy float32 arrays.
//   • Callbacks from C++ threads acquire the GIL before invoking the
//     Python callable.
//   • All blocking C++ operations (connect, start, pull, wait) release
//     the GIL so other Python threads can run.
// ─────────────────────────────────────────────────────────────────────────────

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include "thunderbird/thunderbird.h"
#include "thunderbird/version.h"

#include <optional>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

namespace py = pybind11;
namespace td = thunderbird::data;

// ─── Helpers ────────────────────────────────────────────────────────────────

/// Build a NumPy structured array from a vector of PointXYZIT (with copy).
static py::array make_points_array(const std::vector<td::PointXYZIT>& pts) {
    // Define the dtype once.
    // NumPy requires a *list* of (name, format) tuples, not a tuple of tuples.
    py::list fields;
    fields.append(py::make_tuple("x",         py::format_descriptor<float>::format()));
    fields.append(py::make_tuple("y",         py::format_descriptor<float>::format()));
    fields.append(py::make_tuple("z",         py::format_descriptor<float>::format()));
    fields.append(py::make_tuple("intensity", py::format_descriptor<float>::format()));
    fields.append(py::make_tuple("timestamp_offset_ns",
                       py::format_descriptor<int32_t>::format()));
    py::dtype dt = py::dtype::from_args(fields);

    const auto n = static_cast<py::ssize_t>(pts.size());
    py::array arr(dt, {n}, {});

    if (n > 0) {
        std::memcpy(arr.mutable_data(), pts.data(),
                    pts.size() * sizeof(td::PointXYZIT));
    }
    return arr;
}

/// Build a NumPy uint8 array from an ImageFrame's pixel buffer (with copy
/// so the Python object owns its data and lifetime is independent).
static py::array_t<uint8_t> make_image_array(const td::ImageFrame& f) {
    if (!f.data || f.data->empty()) {
        return py::array_t<uint8_t>({0});
    }

    const uint32_t bpp = td::bytes_per_pixel(f.format);
    const auto h = static_cast<py::ssize_t>(f.height);
    const auto w = static_cast<py::ssize_t>(f.width);
    const auto c = static_cast<py::ssize_t>(bpp);

    py::array_t<uint8_t> arr;
    if (bpp == 1) {
        arr = py::array_t<uint8_t>({h, w});
    } else {
        arr = py::array_t<uint8_t>({h, w, c});
    }

    const size_t total = f.data->size();
    const size_t arr_size = static_cast<size_t>(h * w * c);
    const size_t copy_len = std::min(total, arr_size);
    std::memcpy(arr.mutable_data(), f.data->data(), copy_len);
    return arr;
}

/// Build a NumPy float32[3] from a std::array<float,3>.
static py::array_t<float> make_vec3(const std::array<float, 3>& v) {
    auto arr = py::array_t<float>(3);
    auto buf = arr.mutable_unchecked<1>();
    buf(0) = v[0];
    buf(1) = v[1];
    buf(2) = v[2];
    return arr;
}

// ─── Module definition ─────────────────────────────────────────────────────

PYBIND11_MODULE(_spatial_sdk_core, m) {
    m.doc() = "Thunderbird Spatial SDK — Python bindings";

    // ── Version constant (baked in at compile time from CMake) ──────────
    m.attr("THUNDERBIRD_VERSION") = THUNDERBIRD_VERSION_STRING;
    m.attr("THUNDERBIRD_VERSION_MAJOR") = THUNDERBIRD_VERSION_MAJOR;
    m.attr("THUNDERBIRD_VERSION_MINOR") = THUNDERBIRD_VERSION_MINOR;
    m.attr("THUNDERBIRD_VERSION_PATCH") = THUNDERBIRD_VERSION_PATCH;

    // =====================================================================
    //  Status enum
    // =====================================================================
    py::enum_<thunderbird::Status>(m, "Status")
        .value("OK",               thunderbird::Status::OK)
        .value("NotConnected",     thunderbird::Status::NotConnected)
        .value("Timeout",          thunderbird::Status::Timeout)
        .value("TransportError",   thunderbird::Status::TransportError)
        .value("InvalidParameter", thunderbird::Status::InvalidParameter)
        .value("NotSupported",     thunderbird::Status::NotSupported)
        .value("AlreadyStreaming", thunderbird::Status::AlreadyStreaming)
        .value("InternalError",    thunderbird::Status::InternalError)
        .def("__repr__", [](thunderbird::Status s) {
            return std::string("Status.") + thunderbird::status_string(s);
        });

    // =====================================================================
    //  PixelFormat enum
    // =====================================================================
    py::enum_<td::PixelFormat>(m, "PixelFormat")
        .value("Mono8", td::PixelFormat::Mono8)
        .value("RGB8",  td::PixelFormat::RGB8)
        .value("BGR8",  td::PixelFormat::BGR8)
        .value("YUYV",  td::PixelFormat::YUYV)
        .value("NV12",  td::PixelFormat::NV12);

    // =====================================================================
    //  DeviceInfo
    // =====================================================================
    py::class_<thunderbird::DeviceInfo>(m, "DeviceInfo")
        .def(py::init<>())
        .def_readwrite("serial_number",   &thunderbird::DeviceInfo::serial_number)
        .def_readwrite("firmware_version", &thunderbird::DeviceInfo::firmware_version)
        .def_readwrite("model_name",       &thunderbird::DeviceInfo::model_name)
        .def("__repr__", [](const thunderbird::DeviceInfo& d) {
            std::ostringstream os;
            os << "DeviceInfo(serial='" << d.serial_number
               << "', fw='" << d.firmware_version
               << "', model='" << d.model_name << "')";
            return os.str();
        });

    // =====================================================================
    //  LidarFrame
    // =====================================================================
    py::class_<td::LidarFrame>(m, "LidarFrame")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &td::LidarFrame::timestamp_ns)
        .def_readwrite("sequence",     &td::LidarFrame::sequence)
        .def_property("points",
            // Getter: return NumPy structured array
            [](const td::LidarFrame& f) { return make_points_array(f.points); },
            // Setter: not supported from Python (read-only for safety)
            nullptr)
        .def_property_readonly("num_points",
            [](const td::LidarFrame& f) {
                return static_cast<int>(f.points.size());
            })
        .def("__repr__", [](const td::LidarFrame& f) {
            std::ostringstream os;
            os << "LidarFrame(ts=" << f.timestamp_ns
               << ", seq=" << f.sequence
               << ", pts=" << f.points.size() << ")";
            return os.str();
        });

    // =====================================================================
    //  ImuFrame
    // =====================================================================
    py::class_<td::ImuFrame>(m, "ImuFrame")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &td::ImuFrame::timestamp_ns)
        .def_property_readonly("accel",
            [](const td::ImuFrame& f) { return make_vec3(f.accel); })
        .def_property_readonly("gyro",
            [](const td::ImuFrame& f) { return make_vec3(f.gyro); })
        .def("__repr__", [](const td::ImuFrame& f) {
            std::ostringstream os;
            os << "ImuFrame(ts=" << f.timestamp_ns
               << ", accel=[" << f.accel[0] << "," << f.accel[1] << "," << f.accel[2] << "]"
               << ", gyro=[" << f.gyro[0] << "," << f.gyro[1] << "," << f.gyro[2] << "])";
            return os.str();
        });

    // =====================================================================
    //  ImageFrame
    // =====================================================================
    py::class_<td::ImageFrame>(m, "ImageFrame")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &td::ImageFrame::timestamp_ns)
        .def_readwrite("width",        &td::ImageFrame::width)
        .def_readwrite("height",       &td::ImageFrame::height)
        .def_readwrite("stride",       &td::ImageFrame::stride)
        .def_readwrite("sequence",     &td::ImageFrame::sequence)
        .def_property_readonly("format",
            [](const td::ImageFrame& f) { return f.format; })
        .def_property_readonly("pixels",
            [](const td::ImageFrame& f) { return make_image_array(f); },
            "Pixel data as a NumPy array (H×W×C or H×W for mono)")
        .def("__repr__", [](const td::ImageFrame& f) {
            std::ostringstream os;
            os << "ImageFrame(ts=" << f.timestamp_ns
               << ", " << f.width << "x" << f.height
               << ", seq=" << f.sequence << ")";
            return os.str();
        });

    // =====================================================================
    //  SyncedFrame
    // =====================================================================
    py::class_<td::SyncedFrame>(m, "SyncedFrame")
        .def(py::init<>())
        .def_readonly("lidar",               &td::SyncedFrame::lidar)
        .def_property_readonly("camera",
            [](const td::SyncedFrame& sf) -> py::object {
                if (sf.camera) return py::cast(*sf.camera);
                return py::none();
            },
            "Camera frame (None if missing)")
        .def_property_readonly("imu_block",
            [](const td::SyncedFrame& sf) {
                py::list out;
                for (auto& imu : sf.imu_block)
                    out.append(py::cast(imu));
                return out;
            },
            "List of IMU frames between previous and current LiDAR epoch")
        .def_readonly("lidar_camera_offset_ns", &td::SyncedFrame::lidar_camera_offset_ns)
        .def_readonly("sync_quality",           &td::SyncedFrame::sync_quality)
        .def_readonly("sequence",               &td::SyncedFrame::sequence)
        .def("__repr__", [](const td::SyncedFrame& sf) {
            std::ostringstream os;
            os << "SyncedFrame(seq=" << sf.sequence
               << ", quality=" << sf.sync_quality
               << ", imu_count=" << sf.imu_block.size()
               << ", has_camera=" << sf.camera.has_value() << ")";
            return os.str();
        });

    // =====================================================================
    //  SyncStats
    // =====================================================================
    py::class_<td::SyncStats>(m, "SyncStats")
        .def(py::init<>())
        .def_readonly("frames_produced",   &td::SyncStats::frames_produced)
        .def_readonly("camera_misses",     &td::SyncStats::camera_misses)
        .def_readonly("imu_gaps",          &td::SyncStats::imu_gaps)
        .def_readonly("mean_offset_ns",    &td::SyncStats::mean_offset_ns)
        .def_readonly("drift_ns_per_sec",  &td::SyncStats::drift_ns_per_sec)
        .def("__repr__", [](const td::SyncStats& s) {
            std::ostringstream os;
            os << "SyncStats(produced=" << s.frames_produced
               << ", cam_miss=" << s.camera_misses
               << ", imu_gap=" << s.imu_gaps << ")";
            return os.str();
        });

    // =====================================================================
    //  RecorderDeviceInfo
    // =====================================================================
    py::class_<td::RecorderDeviceInfo>(m, "RecorderDeviceInfo")
        .def(py::init<>())
        .def(py::init([](const std::string& sn, const std::string& fw,
                         const std::string& model) {
            return td::RecorderDeviceInfo{sn, fw, model};
        }), py::arg("serial_number") = "",
            py::arg("firmware_version") = "",
            py::arg("model_name") = "")
        .def_readwrite("serial_number",    &td::RecorderDeviceInfo::serial_number)
        .def_readwrite("firmware_version", &td::RecorderDeviceInfo::firmware_version)
        .def_readwrite("model_name",       &td::RecorderDeviceInfo::model_name)
        .def("__repr__", [](const td::RecorderDeviceInfo& d) {
            std::ostringstream os;
            os << "RecorderDeviceInfo('" << d.serial_number
               << "', '" << d.firmware_version
               << "', '" << d.model_name << "')";
            return os.str();
        });

    // =====================================================================
    //  RecorderStats
    // =====================================================================
    py::class_<td::RecorderStats>(m, "RecorderStats")
        .def(py::init<>())
        .def_readonly("lidar_frames",  &td::RecorderStats::lidar_frames)
        .def_readonly("imu_frames",    &td::RecorderStats::imu_frames)
        .def_readonly("camera_frames", &td::RecorderStats::camera_frames)
        .def_readonly("total_bytes",   &td::RecorderStats::total_bytes)
        .def("__repr__", [](const td::RecorderStats& s) {
            std::ostringstream os;
            os << "RecorderStats(lidar=" << s.lidar_frames
               << ", imu=" << s.imu_frames
               << ", cam=" << s.camera_frames
               << ", bytes=" << s.total_bytes << ")";
            return os.str();
        });

    // =====================================================================
    //  PlayerConfig
    // =====================================================================
    py::class_<td::PlayerConfig>(m, "PlayerConfig")
        .def(py::init<>())
        .def(py::init([](double speed, size_t depth, bool loop) {
            return td::PlayerConfig{speed, depth, loop};
        }), py::arg("playback_speed") = 1.0,
            py::arg("queue_depth")    = 64,
            py::arg("loop")           = false)
        .def_readwrite("playback_speed", &td::PlayerConfig::playback_speed)
        .def_readwrite("queue_depth",    &td::PlayerConfig::queue_depth)
        .def_readwrite("loop",           &td::PlayerConfig::loop);

    // =====================================================================
    //  PlayerStats
    // =====================================================================
    py::class_<td::PlayerStats>(m, "PlayerStats")
        .def(py::init<>())
        .def_readonly("lidar_frames",  &td::PlayerStats::lidar_frames)
        .def_readonly("imu_frames",    &td::PlayerStats::imu_frames)
        .def_readonly("camera_frames", &td::PlayerStats::camera_frames)
        .def_readonly("total_records", &td::PlayerStats::total_records)
        .def("__repr__", [](const td::PlayerStats& s) {
            std::ostringstream os;
            os << "PlayerStats(lidar=" << s.lidar_frames
               << ", imu=" << s.imu_frames
               << ", cam=" << s.camera_frames
               << ", total=" << s.total_records << ")";
            return os.str();
        });

    // =====================================================================
    //  Recorder
    // =====================================================================
    py::class_<td::Recorder>(m, "Recorder")
        .def(py::init([](const std::string& path, td::RecorderDeviceInfo info) {
            return std::make_unique<td::Recorder>(path, std::move(info));
        }), py::arg("file_path"),
            py::arg("device_info") = td::RecorderDeviceInfo{})
        .def("start", &td::Recorder::start,
             "Open the file and begin recording. Returns True on success.")
        .def("stop", &td::Recorder::stop,
             "Flush and close the recording file.")
        .def("recording", &td::Recorder::recording,
             "True while recording is active.")
        .def("record_lidar_frame", &td::Recorder::recordLidarFrame,
             py::arg("frame"),
             "Record a LiDAR frame. Thread-safe.")
        .def("record_imu_frame", &td::Recorder::recordImuFrame,
             py::arg("frame"),
             "Record an IMU frame. Thread-safe.")
        .def("record_image_frame", &td::Recorder::recordImageFrame,
             py::arg("frame"),
             "Record a camera/image frame. Thread-safe.")
        .def("stats", &td::Recorder::stats,
             "Get recording statistics.")
        // Context manager support
        .def("__enter__", [](td::Recorder& r) -> td::Recorder& {
            r.start();
            return r;
        })
        .def("__exit__", [](td::Recorder& r, py::object, py::object, py::object) {
            r.stop();
        });

    // =====================================================================
    //  Player
    // =====================================================================
    py::class_<td::Player>(m, "Player")
        .def(py::init([](const std::string& path, td::PlayerConfig config) {
            return std::make_unique<td::Player>(path, config);
        }), py::arg("file_path"),
            py::arg("config") = td::PlayerConfig{})
        .def("start", [](td::Player& p) {
            bool ok;
            { py::gil_scoped_release release; ok = p.start(); }
            return ok;
        }, "Open the file, validate header, and start playback. Returns True on success.")
        .def("stop", [](td::Player& p) {
            py::gil_scoped_release release;
            p.stop();
        }, "Stop playback and join the playback thread.")
        .def("playing",  &td::Player::playing,  "True while playback is active.")
        .def("finished", &td::Player::finished, "True when all records have been dispatched.")
        // Pull API
        .def("get_next_lidar_frame", [](td::Player& p) -> py::object {
            auto f = p.getNextLidarFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next LiDAR frame or None.")
        .def("get_next_imu_frame", [](td::Player& p) -> py::object {
            auto f = p.getNextImuFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next IMU frame or None.")
        .def("get_next_image_frame", [](td::Player& p) -> py::object {
            auto f = p.getNextImageFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next camera frame or None.")
        // Callback API — must acquire GIL when called from C++ thread
        .def("on_lidar_frame", [](td::Player& p, py::function cb) {
            // prevent Python callback garbage collection
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            p.onLidarFrame([safe_cb](const td::LidarFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register a callback for LiDAR frames (called from playback thread).")
        .def("on_imu_frame", [](td::Player& p, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            p.onImuFrame([safe_cb](const td::ImuFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register a callback for IMU frames (called from playback thread).")
        .def("on_image_frame", [](td::Player& p, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            p.onImageFrame([safe_cb](const td::ImageFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register a callback for camera frames (called from playback thread).")
        // Metadata
        .def("device_info", &td::Player::deviceInfo,
             "Device info from the recording file header.")
        .def("stats", &td::Player::stats,
             "Playback statistics.")
        // Context manager
        .def("__enter__", [](td::Player& p) -> td::Player& {
            p.start();
            return p;
        })
        .def("__exit__", [](td::Player& p, py::object, py::object, py::object) {
            p.stop();
        });

    // =====================================================================
    //  DeviceConfig
    // =====================================================================
    py::class_<thunderbird::DeviceConfig>(m, "DeviceConfig")
        .def(py::init<>())
        .def_readwrite("uri",           &thunderbird::DeviceConfig::uri)
        .def_readwrite("lidar_hz",      &thunderbird::DeviceConfig::lidar_hz)
        .def_readwrite("imu_hz",        &thunderbird::DeviceConfig::imu_hz)
        .def_readwrite("camera_fps",    &thunderbird::DeviceConfig::camera_fps)
        .def_readwrite("camera_width",  &thunderbird::DeviceConfig::camera_width)
        .def_readwrite("camera_height", &thunderbird::DeviceConfig::camera_height);

    // =====================================================================
    //  DeviceManager (the main "Device" entry point)
    // =====================================================================
    py::class_<thunderbird::DeviceManager>(m, "DeviceManager")
        .def(py::init<thunderbird::DeviceConfig>(),
             py::arg("config") = thunderbird::DeviceConfig{})
        // Connection
        .def("connect", [](thunderbird::DeviceManager& d) {
            thunderbird::Status s;
            { py::gil_scoped_release release; s = d.connect(); }
            return s;
        }, "Connect to the device (or simulated backend). Returns Status.")
        .def("disconnect", [](thunderbird::DeviceManager& d) {
            thunderbird::Status s;
            { py::gil_scoped_release release; s = d.disconnect(); }
            return s;
        }, "Disconnect from the device. Returns Status.")
        .def("is_connected", &thunderbird::DeviceManager::is_connected)
        .def("device_info",  &thunderbird::DeviceManager::device_info)
        // Streaming
        .def("start", [](thunderbird::DeviceManager& d) {
            thunderbird::Status s;
            { py::gil_scoped_release release; s = d.start(); }
            return s;
        }, "Start all sensor streams.")
        .def("stop", [](thunderbird::DeviceManager& d) {
            thunderbird::Status s;
            { py::gil_scoped_release release; s = d.stop(); }
            return s;
        }, "Stop all sensor streams.")
        .def("is_streaming", &thunderbird::DeviceManager::is_streaming)
        // Raw callbacks (with GIL safety)
        .def("on_lidar", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.on_lidar([safe_cb](std::shared_ptr<const thunderbird::LidarFrame> f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"), "Register raw LiDAR callback.")
        .def("on_imu", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.on_imu([safe_cb](std::shared_ptr<const thunderbird::ImuSample> f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"), "Register raw IMU callback.")
        .def("on_camera", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.on_camera([safe_cb](std::shared_ptr<const thunderbird::CameraFrame> f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"), "Register raw camera callback.")
        // Data Layer — Pull API (public data types)
        .def("get_next_lidar_frame", [](thunderbird::DeviceManager& d) -> py::object {
            auto f = d.data_layer().getNextLidarFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next LiDAR frame from the data layer, or None.")
        .def("get_next_imu_frame", [](thunderbird::DeviceManager& d) -> py::object {
            auto f = d.data_layer().getNextImuFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next IMU frame from the data layer, or None.")
        .def("get_next_image_frame", [](thunderbird::DeviceManager& d) -> py::object {
            auto f = d.data_layer().getNextImageFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next image frame from the data layer, or None.")
        // Data Layer — Callback API (public data types, with GIL safety)
        .def("register_lidar_callback", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.data_layer().onLidarFrame([safe_cb](const td::LidarFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register callback for LiDAR frames (public data types).")
        .def("register_imu_callback", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.data_layer().onImuFrame([safe_cb](const td::ImuFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register callback for IMU frames (public data types).")
        .def("register_image_callback", [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.data_layer().onImageFrame([safe_cb](const td::ImageFrame& f) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(f); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register callback for image frames (public data types).")
        // Time Sync — Pull API
        .def("get_next_synced_frame", [](thunderbird::DeviceManager& d) -> py::object {
            auto f = d.time_sync().getNextSyncedFrame();
            if (f) return py::cast(*f);
            return py::none();
        }, "Pull next synchronised multi-sensor frame, or None.")
        // Time Sync — Callback API
        .def("register_synced_frame_callback",
             [](thunderbird::DeviceManager& d, py::function cb) {
            auto safe_cb = std::make_shared<py::function>(std::move(cb));
            d.on_synced_frame([safe_cb](const td::SyncedFrame& sf) {
                py::gil_scoped_acquire gil;
                try { (*safe_cb)(sf); }
                catch (py::error_already_set& e) { e.restore(); }
            });
        }, py::arg("callback"),
           "Register callback for synchronised frames.")
        // Time Sync stats
        .def("sync_stats", [](thunderbird::DeviceManager& d) {
            return d.time_sync().stats();
        }, "Get time-sync statistics.")
        // Context manager
        .def("__enter__", [](thunderbird::DeviceManager& d) -> thunderbird::DeviceManager& {
            d.connect();
            return d;
        })
        .def("__exit__", [](thunderbird::DeviceManager& d,
                            py::object, py::object, py::object) {
            d.stop();
            d.disconnect();
        })
        .def("__repr__", [](const thunderbird::DeviceManager& d) {
            std::ostringstream os;
            os << "DeviceManager(connected=" << d.is_connected()
               << ", streaming=" << d.is_streaming() << ")";
            return os.str();
        });
}

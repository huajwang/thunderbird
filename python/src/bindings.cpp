// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Python bindings (pybind11)
// ─────────────────────────────────────────────────────────────────────────────
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include "thunderbird/thunderbird.h"

namespace py = pybind11;
using namespace thunderbird;

// ── Helper: camera frame data → numpy array ─────────────────────────────────
static py::array_t<uint8_t> camera_frame_to_numpy(const CameraFrame& f) {
    size_t channels = (f.format == PixelFormat::Mono8) ? 1 : 3;
    return py::array_t<uint8_t>(
        {static_cast<py::ssize_t>(f.height),
         static_cast<py::ssize_t>(f.width),
         static_cast<py::ssize_t>(channels)},
        f.data.data()
    );
}

PYBIND11_MODULE(thunderbird_py, m) {
    m.doc() = "Thunderbird multi-sensor SDK — Python bindings";

    // ── Timestamp ───────────────────────────────────────────────────────────
    py::class_<Timestamp>(m, "Timestamp")
        .def(py::init<>())
        .def_readwrite("nanoseconds", &Timestamp::nanoseconds)
        .def("to_seconds",  &Timestamp::to_seconds)
        .def_static("now",  &Timestamp::now)
        .def("__repr__", [](const Timestamp& t) {
            return "Timestamp(ns=" + std::to_string(t.nanoseconds) + ")";
        });

    // ── Enums ───────────────────────────────────────────────────────────────
    py::enum_<SensorType>(m, "SensorType")
        .value("LiDAR",  SensorType::LiDAR)
        .value("IMU",    SensorType::IMU)
        .value("Camera", SensorType::Camera);

    py::enum_<Status>(m, "Status")
        .value("OK",               Status::OK)
        .value("NotConnected",     Status::NotConnected)
        .value("Timeout",          Status::Timeout)
        .value("TransportError",   Status::TransportError)
        .value("InvalidParameter", Status::InvalidParameter)
        .value("NotSupported",     Status::NotSupported)
        .value("AlreadyStreaming", Status::AlreadyStreaming)
        .value("InternalError",    Status::InternalError);

    py::enum_<PixelFormat>(m, "PixelFormat")
        .value("Mono8", PixelFormat::Mono8)
        .value("RGB8",  PixelFormat::RGB8)
        .value("BGR8",  PixelFormat::BGR8)
        .value("YUYV",  PixelFormat::YUYV)
        .value("NV12",  PixelFormat::NV12);

    // ── LidarPoint ──────────────────────────────────────────────────────────
    py::class_<LidarPoint>(m, "LidarPoint")
        .def(py::init<>())
        .def_readwrite("x", &LidarPoint::x)
        .def_readwrite("y", &LidarPoint::y)
        .def_readwrite("z", &LidarPoint::z)
        .def_readwrite("intensity", &LidarPoint::intensity)
        .def_readwrite("ring",      &LidarPoint::ring);

    // ── LidarFrame ──────────────────────────────────────────────────────────
    py::class_<LidarFrame, std::shared_ptr<LidarFrame>>(m, "LidarFrame")
        .def_readonly("timestamp",       &LidarFrame::timestamp)
        .def_readonly("host_timestamp",  &LidarFrame::host_timestamp)
        .def_readonly("sequence_number", &LidarFrame::sequence_number)
        .def_readonly("points",          &LidarFrame::points)
        .def("num_points", [](const LidarFrame& f) { return f.points.size(); });

    // ── ImuSample ───────────────────────────────────────────────────────────
    py::class_<ImuSample, std::shared_ptr<ImuSample>>(m, "ImuSample")
        .def_readonly("timestamp",      &ImuSample::timestamp)
        .def_readonly("host_timestamp", &ImuSample::host_timestamp)
        .def_readonly("accel",          &ImuSample::accel)
        .def_readonly("gyro",           &ImuSample::gyro)
        .def_readonly("temperature",    &ImuSample::temperature);

    // ── CameraFrame ─────────────────────────────────────────────────────────
    py::class_<CameraFrame, std::shared_ptr<CameraFrame>>(m, "CameraFrame")
        .def_readonly("timestamp",       &CameraFrame::timestamp)
        .def_readonly("host_timestamp",  &CameraFrame::host_timestamp)
        .def_readonly("sequence_number", &CameraFrame::sequence_number)
        .def_readonly("width",           &CameraFrame::width)
        .def_readonly("height",          &CameraFrame::height)
        .def_readonly("format",          &CameraFrame::format)
        .def("to_numpy", [](const CameraFrame& f) { return camera_frame_to_numpy(f); });

    // ── SyncBundle ──────────────────────────────────────────────────────────
    py::class_<SyncBundle, std::shared_ptr<SyncBundle>>(m, "SyncBundle")
        .def_readonly("reference_time", &SyncBundle::reference_time)
        .def_readonly("lidar",          &SyncBundle::lidar)
        .def_readonly("imu",            &SyncBundle::imu)
        .def_readonly("camera",         &SyncBundle::camera);

    // ── DeviceInfo ──────────────────────────────────────────────────────────
    py::class_<DeviceInfo>(m, "DeviceInfo")
        .def_readonly("serial_number",    &DeviceInfo::serial_number)
        .def_readonly("firmware_version", &DeviceInfo::firmware_version)
        .def_readonly("model_name",       &DeviceInfo::model_name);

    // ── DeviceConfig ────────────────────────────────────────────────────────
    py::class_<DeviceConfig>(m, "DeviceConfig")
        .def(py::init<>())
        .def_readwrite("uri",           &DeviceConfig::uri)
        .def_readwrite("lidar_hz",      &DeviceConfig::lidar_hz)
        .def_readwrite("imu_hz",        &DeviceConfig::imu_hz)
        .def_readwrite("camera_fps",    &DeviceConfig::camera_fps)
        .def_readwrite("camera_width",  &DeviceConfig::camera_width)
        .def_readwrite("camera_height", &DeviceConfig::camera_height);

    // ── DeviceManager ───────────────────────────────────────────────────────
    py::class_<DeviceManager>(m, "DeviceManager")
        .def(py::init<DeviceConfig>(), py::arg("config") = DeviceConfig{})
        .def("connect",       &DeviceManager::connect)
        .def("disconnect",    &DeviceManager::disconnect)
        .def("is_connected",  &DeviceManager::is_connected)
        .def("device_info",   &DeviceManager::device_info)
        .def("start",         &DeviceManager::start)
        .def("stop",          &DeviceManager::stop)
        .def("is_streaming",  &DeviceManager::is_streaming)
        .def("on_lidar",      &DeviceManager::on_lidar)
        .def("on_imu",        &DeviceManager::on_imu)
        .def("on_camera",     &DeviceManager::on_camera)
        .def("on_sync",       &DeviceManager::on_sync)
        .def("sync_bundles_produced", &DeviceManager::sync_bundles_produced);
}

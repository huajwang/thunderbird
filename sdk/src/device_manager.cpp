// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — DeviceManager implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/device_manager.h"
#include "thunderbird/sync_engine.h"
#include "thunderbird/time_sync.h"
#include "thunderbird/data_layer.h"
#include "thunderbird/device_health_monitor.h"
#include "thunderbird/clock_service.h"
#include "thunderbird/lidar_frame_assembler.h"
#include "thunderbird/diagnostics.h"

// Simulated drivers
#include "thunderbird/drivers/simulated_lidar.h"
#include "thunderbird/drivers/simulated_imu.h"
#include "thunderbird/drivers/simulated_camera.h"

// Hardware communication layer
#include "thunderbird/drivers/hardware_driver.h"
#include "thunderbird/connection_manager.h"
#include "thunderbird/ethernet_transport.h"
#include "thunderbird/usb_transport.h"

#include <atomic>
#include <mutex>

namespace thunderbird {
THUNDERBIRD_ABI_NAMESPACE_BEGIN

// ─── PImpl ──────────────────────────────────────────────────────────────────

struct DeviceManager::Impl {
    DeviceConfig config;

    // State
    std::atomic<bool> connected{false};
    std::atomic<bool> streaming{false};

    // Callbacks (set before start(), read from driver threads)
    LidarCallback  user_lidar_cb;
    ImuCallback    user_imu_cb;
    CameraCallback user_camera_cb;

    // Drivers
    std::unique_ptr<ISensorDriver> lidar_driver;
    std::unique_ptr<ISensorDriver> imu_driver;
    std::unique_ptr<ISensorDriver> camera_driver;

    // Sync
    SyncEngine sync_engine;

    // Time sync (Step 3 — SyncedFrame API)
    data::TimeSyncEngine time_sync_engine;

    // Hardware driver (used when not simulated)
    std::shared_ptr<HardwareDriver> hw_driver;

    // Device health monitor (hardware mode only)
    std::unique_ptr<DeviceHealthMonitor> health_monitor;

    // Clock service (unified clock authority)
    ClockService clock_service;

    // LiDAR frame assembler (per-packet → full sweep)
    LidarFrameAssembler frame_assembler;

    // Data Abstraction Layer (Pull + Callback API)
    data::DataLayer data_layer;

    // Diagnostics (unified metrics aggregator)
    DiagnosticsManager diagnostics_mgr;

    explicit Impl(DeviceConfig cfg)
        : config(std::move(cfg)),
          sync_engine(config.sync),
          time_sync_engine(config.time_sync),
          clock_service(config.clock),
          frame_assembler(config.frame_assembler),
          data_layer(data::DataLayerConfig{})
    {
        // ── Wire shared ClockService into all sync engines ──────────────
        sync_engine.set_clock_service(&clock_service);
        time_sync_engine.set_clock_service(&clock_service);

        // ── Register diagnostics collectors ─────────────────────────────

        // Time-sync stats.
        diagnostics_mgr.register_collector("time_sync", [this]() -> MetricMap {
            auto s = time_sync_engine.stats();
            return {
                {"frames_produced", {MetricType::Counter, static_cast<double>(s.frames_produced)}},
                {"camera_misses",   {MetricType::Counter, static_cast<double>(s.camera_misses)}},
                {"imu_gaps",        {MetricType::Counter, static_cast<double>(s.imu_gaps)}},
                {"mean_offset_ns",  {MetricType::Gauge,   s.mean_offset_ns}},
                {"drift_ns_per_sec",{MetricType::Gauge,   s.drift_ns_per_sec}},
            };
        });

        // Clock service diagnostics.
        diagnostics_mgr.register_collector("clock", [this]() -> MetricMap {
            auto d = clock_service.diagnostics();
            return {
                {"offset_ns",        {MetricType::Gauge,   d.offset_ns}},
                {"drift_ns_per_sec", {MetricType::Gauge,   d.drift_ns_per_sec}},
                {"offset_stddev_ns", {MetricType::Gauge,   d.offset_stddev_ns}},
                {"observations",     {MetricType::Counter, static_cast<double>(d.observations)}},
                {"jumps_detected",   {MetricType::Counter, static_cast<double>(d.jumps_detected)}},
                {"outliers_rejected",{MetricType::Counter, static_cast<double>(d.outliers_rejected)}},
                {"calibrated",       {MetricType::Flag,    d.calibrated ? 1.0 : 0.0}},
                {"pps_locked",       {MetricType::Flag,    d.pps_locked ? 1.0 : 0.0}},
            };
        });

        // Frame assembler stats.
        diagnostics_mgr.register_collector("assembler", [this]() -> MetricMap {
            auto s = frame_assembler.stats();
            return {
                {"frames_emitted",       {MetricType::Counter, static_cast<double>(s.frames_emitted)}},
                {"partial_frames",       {MetricType::Counter, static_cast<double>(s.partial_frames)}},
                {"packets_ingested",     {MetricType::Counter, static_cast<double>(s.packets_ingested)}},
                {"points_ingested",      {MetricType::Counter, static_cast<double>(s.points_ingested)}},
                {"dropped_packets_total",{MetricType::Counter, static_cast<double>(s.dropped_packets_total)}},
                {"avg_points_per_frame", {MetricType::Gauge,   s.avg_points_per_frame}},
                {"measured_rate_hz",     {MetricType::Gauge,   s.measured_rate_hz}},
            };
        });

        // Device health collector — registered once in Impl ctor.
        // Returns empty map when health_monitor is null (simulated mode).
        diagnostics_mgr.register_collector("device_health", [this]() -> MetricMap {
            if (!health_monitor) return {};
            auto snap = health_monitor->snapshot();
            return {
                {"health_score",     {MetricType::Gauge,   snap.health_score}},
                {"lidar_hz",         {MetricType::Gauge,   snap.lidar_hz}},
                {"imu_hz",           {MetricType::Gauge,   snap.imu_hz}},
                {"camera_fps",       {MetricType::Gauge,   snap.camera_fps}},
                {"bytes_total",      {MetricType::Counter, static_cast<double>(snap.bytes_total)}},
                {"packets_total",    {MetricType::Counter, static_cast<double>(snap.packets_total)}},
                {"crc_errors_total", {MetricType::Counter, static_cast<double>(snap.crc_errors_total)}},
                {"reconnect_count",  {MetricType::Counter, static_cast<double>(snap.reconnect_count)}},
                {"heartbeat_rtt_ms", {MetricType::Gauge,   snap.heartbeat_rtt_ms}},
                {"flap_detected",    {MetricType::Flag,    snap.flap_detected ? 1.0 : 0.0}},
            };
        });
    }
};

// ─── Construction / destruction ─────────────────────────────────────────────

DeviceManager::DeviceManager(DeviceConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

DeviceManager::~DeviceManager() {
    if (impl_->streaming) stop();
    if (impl_->connected) disconnect();
}

// ─── Connection ─────────────────────────────────────────────────────────────

Status DeviceManager::connect() {
    if (impl_->connected) return Status::OK;

    // --- LiDAR callback wiring ---
    auto lidar_cb = [this](std::shared_ptr<const LidarFrame> f) {
        // Health monitor packet counting (relaxed atomic, ~2 ns).
        if (impl_->health_monitor) impl_->health_monitor->count_lidar();
        impl_->sync_engine.feed_lidar(f);
        impl_->data_layer.ingestLidar(f);
        // Feed time-sync engine (convert internal → public type inline)
        {
            data::LidarFrame df;
            df.timestamp_ns = f->timestamp.nanoseconds;
            df.sequence = 0;
            df.points.reserve(f->points.size());
            for (const auto& p : f->points) {
                df.points.push_back({p.x, p.y, p.z, p.intensity, 0});
            }
            impl_->time_sync_engine.feedLidar(df);
        }
        if (impl_->user_lidar_cb) impl_->user_lidar_cb(f);
    };

    // --- IMU callback wiring ---
    auto imu_cb = [this](std::shared_ptr<const ImuSample> s) {
        // Health monitor packet counting.
        if (impl_->health_monitor) impl_->health_monitor->count_imu();
        impl_->sync_engine.feed_imu(s);
        impl_->data_layer.ingestImu(s);
        // Feed time-sync engine
        {
            data::ImuFrame df;
            df.timestamp_ns = s->timestamp.nanoseconds;
            df.accel[0] = s->accel[0]; df.accel[1] = s->accel[1]; df.accel[2] = s->accel[2];
            df.gyro[0]  = s->gyro[0];  df.gyro[1]  = s->gyro[1];  df.gyro[2]  = s->gyro[2];
            impl_->time_sync_engine.feedImu(df);
        }
        if (impl_->user_imu_cb) impl_->user_imu_cb(s);
    };

    // --- Camera callback wiring ---
    auto camera_cb = [this](std::shared_ptr<const CameraFrame> f) {
        // Health monitor packet counting.
        if (impl_->health_monitor) impl_->health_monitor->count_camera();
        impl_->sync_engine.feed_camera(f);
        impl_->data_layer.ingestCamera(f);
        // Feed time-sync engine
        {
            data::ImageFrame df;
            df.timestamp_ns = f->timestamp.nanoseconds;
            df.width  = f->width;
            df.height = f->height;
            df.stride = f->width * 3;
            df.format = data::PixelFormat::RGB8;
            df.sequence = 0;
            df.data = std::make_shared<const std::vector<uint8_t>>(f->data);
            impl_->time_sync_engine.feedCamera(df);
        }
        if (impl_->user_camera_cb) impl_->user_camera_cb(f);
    };

#if defined(THUNDERBIRD_SIMULATED) && THUNDERBIRD_SIMULATED
    impl_->lidar_driver  = std::make_unique<SimulatedLidarDriver>(
        lidar_cb, impl_->config.lidar_hz);
    impl_->imu_driver    = std::make_unique<SimulatedImuDriver>(
        imu_cb, impl_->config.imu_hz);
    impl_->camera_driver = std::make_unique<SimulatedCameraDriver>(
        camera_cb,
        impl_->config.camera_width,
        impl_->config.camera_height,
        impl_->config.camera_fps);
#else
    // ── Real hardware path ────────────────────────────────────────────────
    // Determine transport type from URI scheme.
    std::unique_ptr<ITransport> transport;
    const auto& uri = impl_->config.uri;

    if (uri.substr(0, 6) == "eth://") {
        transport = std::make_unique<EthernetTransport>();
    } else if (uri.substr(0, 6) == "usb://") {
        transport = std::make_unique<UsbTransport>();
    } else {
        return Status::InvalidParameter;
    }

    // Build connection manager with retry support.
    auto conn_mgr = std::make_unique<ConnectionManager>(
        std::move(transport),
        impl_->config.connection,
        impl_->config.retry);

    // Create unified hardware driver.
    impl_->hw_driver = std::make_shared<HardwareDriver>(std::move(conn_mgr));
    impl_->hw_driver->set_lidar_callback(lidar_cb);
    impl_->hw_driver->set_imu_callback(imu_cb);
    impl_->hw_driver->set_camera_callback(camera_cb);

    Status hw_connect = impl_->hw_driver->connect(uri);
    if (hw_connect != Status::OK) return hw_connect;

    // Create device health monitor.
    {
        DeviceHealthConfig hcfg;
        hcfg.expected_lidar_hz  = impl_->config.lidar_hz;
        hcfg.expected_imu_hz    = impl_->config.imu_hz;
        hcfg.expected_camera_fps = impl_->config.camera_fps;

        auto& cmgr = impl_->hw_driver->connection();

        // Wire the clock service observation hook into the decoder.
        auto* native_parser = dynamic_cast<PacketParser*>(&cmgr.decoder());
        if (native_parser) {
            native_parser->set_clock_observe(
                [this](int64_t hw_ns, int64_t host_ns) {
                    impl_->clock_service.observe(hw_ns, host_ns);
                });
        }

        impl_->health_monitor = std::make_unique<DeviceHealthMonitor>(
            cmgr, cmgr.decoder(), hcfg);

    }

    // Create facade drivers that the rest of DeviceManager expects.
    impl_->lidar_driver  = std::make_unique<HardwareSensorFacade>(
        SensorType::LiDAR, impl_->hw_driver);
    impl_->imu_driver    = std::make_unique<HardwareSensorFacade>(
        SensorType::IMU, impl_->hw_driver);
    impl_->camera_driver = std::make_unique<HardwareSensorFacade>(
        SensorType::Camera, impl_->hw_driver);
#endif

    // Initialise each driver
    for (auto* drv : {impl_->lidar_driver.get(),
                      impl_->imu_driver.get(),
                      impl_->camera_driver.get()}) {
        Status s = drv->initialize();
        if (s != Status::OK) return s;
    }

    impl_->connected = true;
    return Status::OK;
}

Status DeviceManager::disconnect() {
    if (impl_->streaming) stop();
    if (impl_->health_monitor) impl_->health_monitor->stop();
    impl_->lidar_driver.reset();
    impl_->imu_driver.reset();
    impl_->camera_driver.reset();
    impl_->health_monitor.reset();
    impl_->connected = false;
    return Status::OK;
}

bool DeviceManager::is_connected() const { return impl_->connected; }

DeviceInfo DeviceManager::device_info() const {
    DeviceInfo info;
#if defined(THUNDERBIRD_SIMULATED) && THUNDERBIRD_SIMULATED
    info.serial_number    = "SIM-000001";
    info.firmware_version = "0.0.0-sim";
    info.model_name       = "Thunderbird-Sim";
#else
    if (impl_->hw_driver)
        info = impl_->hw_driver->device_info();
    else
        info.model_name = "Thunderbird";
#endif
    return info;
}

// ─── Streaming ──────────────────────────────────────────────────────────────

Status DeviceManager::start() {
    if (!impl_->connected) return Status::NotConnected;
    if (impl_->streaming)  return Status::AlreadyStreaming;

    impl_->sync_engine.start();
    impl_->time_sync_engine.start();

    for (auto* drv : {impl_->lidar_driver.get(),
                      impl_->imu_driver.get(),
                      impl_->camera_driver.get()}) {
        Status s = drv->start_streaming();
        if (s != Status::OK) return s;
    }

    impl_->streaming = true;

    // Start device health monitor after streaming begins.
    if (impl_->health_monitor) impl_->health_monitor->start();

    // Start diagnostics collection.
    impl_->diagnostics_mgr.start();

    return Status::OK;
}

Status DeviceManager::stop() {
    if (!impl_->streaming) return Status::OK;

    // Stop diagnostics collection.
    impl_->diagnostics_mgr.stop();

    // Stop health monitor before stopping streams.
    if (impl_->health_monitor) impl_->health_monitor->stop();

    for (auto* drv : {impl_->lidar_driver.get(),
                      impl_->imu_driver.get(),
                      impl_->camera_driver.get()}) {
        if (drv) drv->stop_streaming();
    }

    impl_->sync_engine.stop();
    impl_->time_sync_engine.stop();
    impl_->streaming = false;
    return Status::OK;
}

bool DeviceManager::is_streaming() const { return impl_->streaming; }

// ─── Callbacks ──────────────────────────────────────────────────────────────

void DeviceManager::on_lidar(LidarCallback cb)   { impl_->user_lidar_cb  = std::move(cb); }
void DeviceManager::on_imu(ImuCallback cb)        { impl_->user_imu_cb    = std::move(cb); }
void DeviceManager::on_camera(CameraCallback cb)  { impl_->user_camera_cb = std::move(cb); }

void DeviceManager::on_sync(SyncCallback cb) {
    impl_->sync_engine.set_callback(std::move(cb));
}

// ─── Stats ──────────────────────────────────────────────────────────────────

uint64_t DeviceManager::sync_bundles_produced() const {
    return impl_->sync_engine.bundles_produced();
}

data::DataLayer& DeviceManager::data_layer() {
    return impl_->data_layer;
}

const data::DataLayer& DeviceManager::data_layer() const {
    return impl_->data_layer;
}
// ─── Time Sync Layer (Step 3) ──────────────────────────────────────────────────

data::TimeSyncEngine& DeviceManager::time_sync() {
    return impl_->time_sync_engine;
}

const data::TimeSyncEngine& DeviceManager::time_sync() const {
    return impl_->time_sync_engine;
}

void DeviceManager::on_synced_frame(data::SyncedFrameCallback cb) {
    impl_->time_sync_engine.onSyncedFrame(std::move(cb));
}

// ─── Device Health ─────────────────────────────────────────────────────────────────

DeviceHealthMonitor* DeviceManager::health_monitor() {
    return impl_->health_monitor.get();
}

const DeviceHealthMonitor* DeviceManager::health_monitor() const {
    return impl_->health_monitor.get();
}

DeviceHealthState DeviceManager::device_health() const {
    if (impl_->health_monitor)
        return impl_->health_monitor->state();
    return DeviceHealthState::Disconnected;
}

// ─── Clock Service ─────────────────────────────────────────────────────────────────

ClockService& DeviceManager::clock_service() {
    return impl_->clock_service;
}

const ClockService& DeviceManager::clock_service() const {
    return impl_->clock_service;
}

// ─── LiDAR Frame Assembler ─────────────────────────────────────────────────────────

LidarFrameAssembler& DeviceManager::frame_assembler() {
    return impl_->frame_assembler;
}

const LidarFrameAssembler& DeviceManager::frame_assembler() const {
    return impl_->frame_assembler;
}

// ─── Diagnostics ───────────────────────────────────────────────────────────────────────

DiagnosticsManager& DeviceManager::diagnostics() {
    return impl_->diagnostics_mgr;
}

const DiagnosticsManager& DeviceManager::diagnostics() const {
    return impl_->diagnostics_mgr;
}

DiagnosticsSnapshot DeviceManager::diagnostics_snapshot() const {
    return impl_->diagnostics_mgr.latest_snapshot();
}

std::string DeviceManager::diagnostics_json() const {
    return impl_->diagnostics_mgr.to_json();
}

THUNDERBIRD_ABI_NAMESPACE_END
} // namespace thunderbird

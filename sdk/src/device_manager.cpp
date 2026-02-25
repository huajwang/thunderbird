// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — DeviceManager implementation
// ─────────────────────────────────────────────────────────────────────────────
#include "thunderbird/device_manager.h"
#include "thunderbird/sync_engine.h"
#include "thunderbird/time_sync.h"
#include "thunderbird/data_layer.h"

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

    // Data Abstraction Layer (Pull + Callback API)
    data::DataLayer data_layer;

    explicit Impl(DeviceConfig cfg)
        : config(std::move(cfg)),
          sync_engine(config.sync),
          time_sync_engine(config.time_sync),
          data_layer(data::DataLayerConfig{}) {}
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
    impl_->lidar_driver.reset();
    impl_->imu_driver.reset();
    impl_->camera_driver.reset();
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
    return Status::OK;
}

Status DeviceManager::stop() {
    if (!impl_->streaming) return Status::OK;

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
} // namespace thunderbird

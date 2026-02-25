"""
Thunderbird Spatial SDK — Python package.

Usage::

    import spatial_sdk

    dev = spatial_sdk.DeviceManager()
    dev.connect()
    dev.start()

    # Pull API
    frame = dev.get_next_lidar_frame()

    # Callback API
    dev.register_lidar_callback(lambda f: print(f))

    # Recorder / Player
    rec = spatial_sdk.Recorder("session.tbrec")
    player = spatial_sdk.Player("session.tbrec")
"""

# Re-export everything from the C++ extension module.
from _spatial_sdk_core import *   # noqa: F401, F403

# Provide package-level metadata
__version__ = "0.1.0"
__all__ = [
    # Enums
    "Status",
    "PixelFormat",
    # Data types
    "DeviceInfo",
    "LidarFrame",
    "ImuFrame",
    "ImageFrame",
    "SyncedFrame",
    "SyncStats",
    "RecorderDeviceInfo",
    "RecorderStats",
    "PlayerConfig",
    "PlayerStats",
    # Device
    "DeviceConfig",
    "DeviceManager",
    # Recorder / Player
    "Recorder",
    "Player",
]

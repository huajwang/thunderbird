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
# Wheel: extension lives inside this package → relative import.
# Dev:   extension is a top-level module (PYTHONPATH) → absolute import.
try:
    from ._spatial_sdk_core import *   # noqa: F401, F403 — installed wheel
except ImportError:
    from _spatial_sdk_core import *    # noqa: F401, F403 — local dev build

# ── Version ──────────────────────────────────────────────────────────────────
# Priority chain:
#   1. importlib.metadata (installed wheel — written by scikit-build-core)
#   2. C++ extension module (compiled with THUNDERBIRD_VERSION_STRING)
#   3. Fallback dev version
def _resolve_version() -> str:
    # 1. Installed package metadata (wheels set this automatically)
    try:
        from importlib.metadata import version as _pkg_version
        return _pkg_version("spatial-sdk")
    except Exception:
        pass

    # 2. C++ extension (version.h baked in at CMake configure time)
    try:
        import _spatial_sdk_core as _core
        if hasattr(_core, "THUNDERBIRD_VERSION"):
            return _core.THUNDERBIRD_VERSION
    except Exception:
        pass

    # 3. Fallback for editable / dev installs
    return "0.0.0.dev0"


__version__ = _resolve_version()
del _resolve_version
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

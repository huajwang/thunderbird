// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — Symbol export / visibility macros
// ─────────────────────────────────────────────────────────────────────────────
//
// Usage:
//   class THUNDERBIRD_API DeviceManager { ... };
//   THUNDERBIRD_API Status connect();
//
// Build modes:
//   Building shared library → define THUNDERBIRD_SDK_EXPORTS
//   Consuming static lib    → define THUNDERBIRD_SDK_STATIC  (auto via CMake)
//   Consuming shared lib    → neither (import is the default)
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

// ── Platform-specific helpers ───────────────────────────────────────────────

#if defined(_WIN32) || defined(__CYGWIN__)
#  define THUNDERBIRD_HELPER_DLL_IMPORT __declspec(dllimport)
#  define THUNDERBIRD_HELPER_DLL_EXPORT __declspec(dllexport)
#  define THUNDERBIRD_HELPER_DLL_LOCAL
#elif defined(__GNUC__) || defined(__clang__)
#  define THUNDERBIRD_HELPER_DLL_IMPORT __attribute__((visibility("default")))
#  define THUNDERBIRD_HELPER_DLL_EXPORT __attribute__((visibility("default")))
#  define THUNDERBIRD_HELPER_DLL_LOCAL  __attribute__((visibility("hidden")))
#else
#  define THUNDERBIRD_HELPER_DLL_IMPORT
#  define THUNDERBIRD_HELPER_DLL_EXPORT
#  define THUNDERBIRD_HELPER_DLL_LOCAL
#endif

// ── Public API macro ────────────────────────────────────────────────────────
//
// THUNDERBIRD_API   — marks symbols that are part of the public ABI.
// THUNDERBIRD_LOCAL  — marks symbols that must NOT be exported.

#if defined(THUNDERBIRD_SDK_STATIC)
   // Static library: no import/export needed.
#  define THUNDERBIRD_API
#  define THUNDERBIRD_LOCAL
#elif defined(THUNDERBIRD_SDK_EXPORTS)
   // Building the shared library: export symbols.
#  define THUNDERBIRD_API   THUNDERBIRD_HELPER_DLL_EXPORT
#  define THUNDERBIRD_LOCAL THUNDERBIRD_HELPER_DLL_LOCAL
#else
   // Consuming the shared library: import symbols.
#  define THUNDERBIRD_API   THUNDERBIRD_HELPER_DLL_IMPORT
#  define THUNDERBIRD_LOCAL THUNDERBIRD_HELPER_DLL_LOCAL
#endif

// ── Deprecation ─────────────────────────────────────────────────────────────

#if defined(__cplusplus) && __cplusplus >= 201402L
#  define THUNDERBIRD_DEPRECATED(msg)      [[deprecated(msg)]]
#  define THUNDERBIRD_DEPRECATED_SINCE(v)  [[deprecated("Deprecated since v" #v ". See migration guide.")]]
#else
#  define THUNDERBIRD_DEPRECATED(msg)
#  define THUNDERBIRD_DEPRECATED_SINCE(v)
#endif

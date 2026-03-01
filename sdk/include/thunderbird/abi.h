// ─────────────────────────────────────────────────────────────────────────────
// Thunderbird SDK — ABI versioned inline namespace
// ─────────────────────────────────────────────────────────────────────────────
//
// Every Tier 1 / Tier 2 public header wraps its namespace body with:
//
//   namespace thunderbird {
//   THUNDERBIRD_ABI_NAMESPACE_BEGIN
//   ...
//   THUNDERBIRD_ABI_NAMESPACE_END
//   }
//
// External users write `thunderbird::DeviceManager` as usual — the inline
// namespace is transparent at the source level.
//
// However, mangled symbols contain the ABI tag (e.g. `abi_v1`), which
// causes a linker error if headers from ABI version N are linked against
// a library built with ABI version M.  This turns silent, undefined-
// behaviour ABI mismatches into loud, deterministic link failures.
//
// Pattern used by: libc++ (std::__1), abseil-cpp (absl::lts_*),
//                  folly (folly::v2022_*).
//
// Bump THUNDERBIRD_ABI_VERSION on:
//   • struct layout changes (add/remove/reorder fields)
//   • vtable changes (add/remove/reorder virtual methods)
//   • function signature changes
//   • enum value removal / reorder
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

/// Current ABI version.  Must match SOVERSION.
#define THUNDERBIRD_ABI_VERSION 0

// Implementation note: the token-paste must go through a helper macro
// so that THUNDERBIRD_ABI_VERSION is expanded before pasting.
#define THUNDERBIRD_ABI_NS_CONCAT_(x) abi_v##x
#define THUNDERBIRD_ABI_NS_NAME_(x)   THUNDERBIRD_ABI_NS_CONCAT_(x)

#define THUNDERBIRD_ABI_NAMESPACE_BEGIN \
    inline namespace THUNDERBIRD_ABI_NS_NAME_(THUNDERBIRD_ABI_VERSION) {
#define THUNDERBIRD_ABI_NAMESPACE_END \
    }

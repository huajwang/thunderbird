#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Thunderbird SDK — Debian package builder
# ─────────────────────────────────────────────────────────────────────────────
#
# Usage:
#   ./packaging/build_deb.sh <VERSION> <ARCH> <BUILD_DIR>
#
# Example:
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr
#   cmake --build build
#   ./packaging/build_deb.sh 1.2.3 amd64 build
#
# The VERSION argument can also be "auto" to extract from CMakeLists.txt:
#   ./packaging/build_deb.sh auto amd64 build
#
# Produces:  dist/thunderbird-sdk_<VERSION>_<ARCH>.deb
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

VERSION="${1:?Usage: $0 <VERSION|auto> <ARCH> <BUILD_DIR>}"
ARCH="${2:?Usage: $0 <VERSION|auto> <ARCH> <BUILD_DIR>}"
BUILD_DIR="${3:?Usage: $0 <VERSION|auto> <ARCH> <BUILD_DIR>}"

# ── Resolve "auto" version ───────────────────────────────────────────────────
if [[ "${VERSION}" == "auto" ]]; then
    if [[ -x "${REPO_ROOT}/scripts/extract-version.sh" ]]; then
        VERSION="$("${REPO_ROOT}/scripts/extract-version.sh" --from-cmake)"
    else
        VERSION=$(grep -oP 'project\s*\(.*?VERSION\s+\K[0-9]+\.[0-9]+\.[0-9]+' "${REPO_ROOT}/CMakeLists.txt")
    fi
    echo "==> Auto-detected version: ${VERSION}"
fi

# ── Validate version ────────────────────────────────────────────────────────
if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: Invalid version '${VERSION}' — expected X.Y.Z" >&2
    exit 1
fi

PKG_NAME="thunderbird-sdk"
PKG_DIR="$(pwd)/dist/.debpkg"
OUT_DIR="$(pwd)/dist"

echo "==> Building ${PKG_NAME}_${VERSION}_${ARCH}.deb"

# ── Clean & prepare staging tree ─────────────────────────────────────────────
rm -rf "${PKG_DIR}"
mkdir -p "${PKG_DIR}/DEBIAN"
mkdir -p "${PKG_DIR}/usr/lib"
mkdir -p "${PKG_DIR}/usr/include/thunderbird"
mkdir -p "${PKG_DIR}/usr/share/doc/${PKG_NAME}"

# ── Install built artifacts into the staging tree ────────────────────────────

# Static library
if [ -f "${BUILD_DIR}/sdk/libthunderbird_sdk.a" ]; then
    cp "${BUILD_DIR}/sdk/libthunderbird_sdk.a" "${PKG_DIR}/usr/lib/"
fi

# Headers (public API)
cp -r sdk/include/thunderbird/* "${PKG_DIR}/usr/include/thunderbird/"

# Documentation
if [ -f README.md ]; then
    cp README.md "${PKG_DIR}/usr/share/doc/${PKG_NAME}/"
fi

# ── Control file ─────────────────────────────────────────────────────────────
cat > "${PKG_DIR}/DEBIAN/control" << EOF
Package: ${PKG_NAME}
Version: ${VERSION}
Section: libs
Priority: optional
Architecture: ${ARCH}
Depends: libc6 (>= 2.35), libstdc++6 (>= 12)
Maintainer: Thunderbird SDK Team <sdk@thunderbird.dev>
Description: Thunderbird multi-sensor SDK
 C++20 SDK for multi-sensor devices (LiDAR + IMU + Camera).
 Provides device communication, data abstraction, time synchronization,
 recording/playback, and Python bindings.
 .
 This package contains the static library and development headers.
Homepage: https://github.com/thunderbird-sdk/thunderbird
EOF

# ── Copyright file ───────────────────────────────────────────────────────────
cat > "${PKG_DIR}/usr/share/doc/${PKG_NAME}/copyright" << EOF
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: ${PKG_NAME}
Source: https://github.com/thunderbird-sdk/thunderbird

Files: *
Copyright: $(date +%Y) Thunderbird SDK Team
License: MIT
EOF

# ── Build the .deb ───────────────────────────────────────────────────────────
mkdir -p "${OUT_DIR}"
dpkg-deb --build --root-owner-group "${PKG_DIR}" \
    "${OUT_DIR}/${PKG_NAME}_${VERSION}_${ARCH}.deb"

# ── Cleanup staging ──────────────────────────────────────────────────────────
rm -rf "${PKG_DIR}"

echo "==> Built: ${OUT_DIR}/${PKG_NAME}_${VERSION}_${ARCH}.deb"
ls -lh "${OUT_DIR}/${PKG_NAME}_${VERSION}_${ARCH}.deb"

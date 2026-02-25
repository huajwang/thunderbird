#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# update-changelog.sh — Generate/update debian/changelog from Git tag version
# ─────────────────────────────────────────────────────────────────────────────
#
# Usage:
#   ./scripts/update-changelog.sh [VERSION] [DISTRIBUTION]
#
# If VERSION is omitted, auto-detects from Git tag or CMakeLists.txt.
# DISTRIBUTION defaults to "jammy" (Ubuntu 22.04).
#
# This script is called by CI before dpkg-buildpackage to stamp the correct
# version into debian/changelog.  It uses `dch` (devscripts) if available,
# otherwise writes the file directly.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Resolve version ─────────────────────────────────────────────────────────
VERSION="${1:-}"
DISTRIBUTION="${2:-jammy}"

if [[ -z "${VERSION}" ]]; then
    if [[ -x "${REPO_ROOT}/scripts/extract-version.sh" ]]; then
        VERSION="$("${REPO_ROOT}/scripts/extract-version.sh")"
    else
        VERSION=$(grep -oP 'project\s*\(.*?VERSION\s+\K[0-9]+\.[0-9]+\.[0-9]+' \
                  "${REPO_ROOT}/CMakeLists.txt")
    fi
fi

if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: Invalid version '${VERSION}'" >&2
    exit 1
fi

MAINTAINER="${DEBFULLNAME:-Thunderbird SDK Team} <${DEBEMAIL:-sdk@thunderbird.dev}>"
TIMESTAMP="$(date -R)"

echo "==> Generating debian/changelog for ${VERSION} (${DISTRIBUTION})"

# ── Write changelog ─────────────────────────────────────────────────────────
cat > "${REPO_ROOT}/debian/changelog" << EOF
thunderbird-sdk (${VERSION}) ${DISTRIBUTION}; urgency=medium

  * Release ${VERSION}

 -- ${MAINTAINER}  ${TIMESTAMP}
EOF

echo "==> debian/changelog updated"
cat "${REPO_ROOT}/debian/changelog"

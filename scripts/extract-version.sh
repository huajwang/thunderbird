#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# extract-version.sh — Single-source version extraction utility
# ─────────────────────────────────────────────────────────────────────────────
#
# Usage:
#   ./scripts/extract-version.sh              # from Git tag or CMakeLists.txt
#   ./scripts/extract-version.sh --from-tag   # Git tag only (fails if no tag)
#   ./scripts/extract-version.sh --from-cmake # CMakeLists.txt only
#   ./scripts/extract-version.sh --component major|minor|patch
#   ./scripts/extract-version.sh --validate <VERSION>
#
# In CI (GitHub Actions), set GITHUB_REF_NAME=vX.Y.Z or pass --from-tag.
#
# Output (stdout): plain version string, e.g. "1.2.3"
# Exit code: 0 on success, 1 on error
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Helpers ──────────────────────────────────────────────────────────────────

die() { echo "ERROR: $*" >&2; exit 1; }

# Validate semver format (strict: X.Y.Z, no pre-release/build metadata)
validate_semver() {
    local ver="$1"
    if [[ ! "${ver}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        die "Invalid semver: '${ver}' — expected X.Y.Z"
    fi
}

# Extract version from Git tag (GITHUB_REF_NAME or git describe)
version_from_tag() {
    local tag=""

    # CI environment: GITHUB_REF_NAME is set for tag-triggered workflows
    if [[ -n "${GITHUB_REF_NAME:-}" && "${GITHUB_REF_NAME}" =~ ^v[0-9] ]]; then
        tag="${GITHUB_REF_NAME}"
    else
        # Local fallback: nearest annotated/lightweight tag
        tag="$(git -C "${REPO_ROOT}" describe --tags --abbrev=0 2>/dev/null || true)"
    fi

    if [[ -z "${tag}" ]]; then
        die "No Git tag found. Use --from-cmake or set GITHUB_REF_NAME."
    fi

    # Strip leading 'v'
    local ver="${tag#v}"
    validate_semver "${ver}"
    echo "${ver}"
}

# Extract version from CMakeLists.txt project() directive
version_from_cmake() {
    local cmake_file="${REPO_ROOT}/CMakeLists.txt"
    [[ -f "${cmake_file}" ]] || die "CMakeLists.txt not found at ${cmake_file}"

    local ver
    ver=$(grep -oP 'project\s*\(.*?VERSION\s+\K[0-9]+\.[0-9]+\.[0-9]+' "${cmake_file}" || true)

    if [[ -z "${ver}" ]]; then
        die "Could not extract VERSION from CMakeLists.txt project() directive"
    fi

    validate_semver "${ver}"
    echo "${ver}"
}

# Extract a single component from a version string
extract_component() {
    local ver="$1" component="$2"
    local major minor patch
    IFS='.' read -r major minor patch <<< "${ver}"

    case "${component}" in
        major) echo "${major}" ;;
        minor) echo "${minor}" ;;
        patch) echo "${patch}" ;;
        *) die "Unknown component: '${component}' — expected major|minor|patch" ;;
    esac
}

# ── Main ─────────────────────────────────────────────────────────────────────

MODE="${1:-auto}"
shift || true

case "${MODE}" in
    --from-tag)
        version_from_tag
        ;;
    --from-cmake)
        version_from_cmake
        ;;
    --component)
        COMPONENT="${1:?Usage: $0 --component major|minor|patch}"
        # Auto-detect source
        VER="$(version_from_tag 2>/dev/null || version_from_cmake)"
        extract_component "${VER}" "${COMPONENT}"
        ;;
    --validate)
        VAL="${1:?Usage: $0 --validate <VERSION>}"
        validate_semver "${VAL}"
        echo "${VAL}"
        ;;
    --check-consistency)
        # Verify that tag version matches CMakeLists.txt version
        TAG_VER="$(version_from_tag)"
        CMAKE_VER="$(version_from_cmake)"
        if [[ "${TAG_VER}" != "${CMAKE_VER}" ]]; then
            die "Version mismatch: Git tag=${TAG_VER}, CMakeLists.txt=${CMAKE_VER}"
        fi
        echo "${TAG_VER}"
        ;;
    auto|"")
        # Prefer tag, fall back to CMakeLists.txt
        # Run in a subshell so die()/exit inside version_from_tag
        # doesn't terminate this script before the || fallback runs.
        (version_from_tag) 2>/dev/null || version_from_cmake
        ;;
    *)
        die "Unknown option: ${MODE}"
        ;;
esac

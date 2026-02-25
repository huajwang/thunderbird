#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# generate-release-notes.sh — Auto-generate release notes from Git history
# ─────────────────────────────────────────────────────────────────────────────
#
# Usage:
#   ./scripts/generate-release-notes.sh <VERSION> [ARTIFACTS_DIR]
#
# Generates structured Markdown release notes by:
#   1. Finding the previous release tag
#   2. Extracting commits between previous tag and HEAD
#   3. Categorizing commits by conventional-commit prefix
#   4. Building an artifacts table from files in ARTIFACTS_DIR
#   5. Adding Docker pull commands and verification instructions
#
# Output: writes to stdout (pipe to a file or --notes-file)
#
# Commit categories (conventional commits):
#   feat:     → ✨ Features
#   fix:      → 🐛 Bug Fixes
#   perf:     → ⚡ Performance
#   docs:     → 📚 Documentation
#   ci:       → 🏗️ CI/CD
#   build:    → 📦 Build
#   refactor: → ♻️ Refactoring
#   test:     → 🧪 Tests
#   chore:    → 🔧 Maintenance
#   (other)   → 📝 Other Changes
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

VERSION="${1:?Usage: generate-release-notes.sh <VERSION> [ARTIFACTS_DIR]}"
ARTIFACTS_DIR="${2:-}"
REPO="${GITHUB_REPOSITORY:-thunderbird-sdk/thunderbird}"

# ── Find previous release tag ───────────────────────────────────────────────
find_previous_tag() {
    # Get all version tags sorted by version, find the one before current
    local current_tag="v${VERSION}"
    local prev_tag=""

    # List all v* tags by creation date (newest first)
    prev_tag=$(git tag -l 'v[0-9]*.[0-9]*.[0-9]*' --sort=-version:refname \
        | grep -v "^${current_tag}$" \
        | head -1 || true)

    if [[ -n "${prev_tag}" ]]; then
        echo "${prev_tag}"
    else
        # No previous tag — use the initial commit
        git rev-list --max-parents=0 HEAD | head -1
    fi
}

# ── Categorize commits ──────────────────────────────────────────────────────
generate_changelog() {
    local since="$1"
    local until="${2:-HEAD}"

    # Arrays for each category
    local -a feats=() fixes=() perfs=() docs=() cis=() builds=() refactors=() tests=() chores=() others=()

    while IFS= read -r line; do
        [[ -z "${line}" ]] && continue

        # Extract short hash and message
        local hash="${line%% *}"
        local msg="${line#* }"

        # Categorize by conventional commit prefix
        case "${msg}" in
            feat:*|feat\(*) feats+=("- ${msg} (\`${hash}\`)") ;;
            fix:*|fix\(*)   fixes+=("- ${msg} (\`${hash}\`)") ;;
            perf:*|perf\(*) perfs+=("- ${msg} (\`${hash}\`)") ;;
            docs:*|docs\(*) docs+=("- ${msg} (\`${hash}\`)") ;;
            ci:*|ci\(*)     cis+=("- ${msg} (\`${hash}\`)") ;;
            build:*|build\(*) builds+=("- ${msg} (\`${hash}\`)") ;;
            refactor:*|refactor\(*) refactors+=("- ${msg} (\`${hash}\`)") ;;
            test:*|test\(*) tests+=("- ${msg} (\`${hash}\`)") ;;
            chore:*|chore\(*) chores+=("- ${msg} (\`${hash}\`)") ;;
            *)              others+=("- ${msg} (\`${hash}\`)") ;;
        esac
    done < <(git log "${since}..${until}" --pretty=format:'%h %s' --no-merges 2>/dev/null || true)

    # Emit sections (only if non-empty)
    local has_changes=false

    emit_section() {
        local title="$1"
        shift
        local -a items=("$@")
        if [[ ${#items[@]} -gt 0 ]]; then
            echo ""
            echo "### ${title}"
            echo ""
            printf '%s\n' "${items[@]}"
            has_changes=true
        fi
    }

    emit_section "✨ Features"      "${feats[@]+"${feats[@]}"}"
    emit_section "🐛 Bug Fixes"     "${fixes[@]+"${fixes[@]}"}"
    emit_section "⚡ Performance"   "${perfs[@]+"${perfs[@]}"}"
    emit_section "♻️ Refactoring"   "${refactors[@]+"${refactors[@]}"}"
    emit_section "📚 Documentation" "${docs[@]+"${docs[@]}"}"
    emit_section "🧪 Tests"         "${tests[@]+"${tests[@]}"}"
    emit_section "🏗️ CI/CD"        "${cis[@]+"${cis[@]}"}"
    emit_section "📦 Build"         "${builds[@]+"${builds[@]}"}"
    emit_section "🔧 Maintenance"   "${chores[@]+"${chores[@]}"}"
    emit_section "📝 Other Changes" "${others[@]+"${others[@]}"}"

    if [[ "${has_changes}" != "true" ]]; then
        echo ""
        echo "### Changes"
        echo ""
        echo "- Initial release"
    fi
}

# ── Build artifacts table ────────────────────────────────────────────────────
generate_artifacts_table() {
    local dir="$1"
    [[ -d "${dir}" ]] || return 0

    echo ""
    echo "### 📦 Artifacts"
    echo ""
    echo "| Artifact | Type | Platform | Size |"
    echo "|----------|------|----------|------|"

    for f in "${dir}"/*.deb "${dir}"/*.whl; do
        [[ -f "${f}" ]] || continue
        local fname
        fname=$(basename "${f}")
        local size
        size=$(du -h "${f}" | cut -f1)

        # Detect type and platform
        local type="" platform=""
        case "${fname}" in
            libthunderbird-sdk0_*)    type="Debian (runtime)" ;;
            libthunderbird-sdk-dev_*) type="Debian (dev)" ;;
            *.whl)                    type="Python wheel" ;;
            *.deb)                    type="Debian" ;;
        esac

        # Extract architecture
        if [[ "${fname}" =~ (amd64|x86_64) ]]; then
            platform="x86_64"
        elif [[ "${fname}" =~ (arm64|aarch64) ]]; then
            platform="aarch64"
        else
            platform="—"
        fi

        # Extract Python version from wheel name
        if [[ "${fname}" =~ cp([0-9]+) ]]; then
            local pyver="${BASH_REMATCH[1]}"
            local pymajor="${pyver:0:1}"
            local pyminor="${pyver:1}"
            platform="${platform} / Python ${pymajor}.${pyminor}"
        fi

        echo "| \`${fname}\` | ${type} | ${platform} | ${size} |"
    done
}

# ── Main output ──────────────────────────────────────────────────────────────

PREV_TAG=$(find_previous_tag)
if [[ "${PREV_TAG}" =~ ^v ]]; then
    PREV_DISPLAY="${PREV_TAG}"
else
    PREV_DISPLAY="(initial)"
fi

cat << HEADER
## Thunderbird SDK ${VERSION}

**Full Changelog**: [\`${PREV_DISPLAY}\`...v${VERSION}](https://github.com/${REPO}/compare/${PREV_TAG}...v${VERSION})

---

## What's Changed
HEADER

generate_changelog "${PREV_TAG}"

if [[ -n "${ARTIFACTS_DIR}" ]]; then
    generate_artifacts_table "${ARTIFACTS_DIR}"
fi

cat << 'DOCKER'

---

### 🐳 Docker Images

```bash
# Runtime (shared library only, ~30 MB)
DOCKER

echo "docker pull ghcr.io/${REPO}-runtime:${VERSION}"
echo ""
echo "# Development (full toolchain + source)"
echo "docker pull ghcr.io/${REPO}-dev:${VERSION}"

cat << 'VERIFY'
```

### 🔐 Verification

All artifacts are checksummed and signed via [Sigstore](https://www.sigstore.dev/) cosign (keyless OIDC).

```bash
# Verify file checksums
sha256sum -c SHA256SUMS.txt

# Verify checksum signature (requires cosign)
cosign verify-blob \
  --signature SHA256SUMS.txt.sig \
  --certificate SHA256SUMS.txt.cert \
VERIFY

echo "  --certificate-identity-regexp \"github.com/${REPO}\" \\"
echo '  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \'
echo '  SHA256SUMS.txt'

cat << DOCKER_VERIFY

# Verify Docker image signature
DOCKER_VERIFY

echo "cosign verify \\"
echo "  --certificate-identity-regexp \"github.com/${REPO}\" \\"
echo '  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \'
echo "  ghcr.io/${REPO}-runtime:${VERSION}"

cat << 'INSTALL'
```

### 📥 Install

```bash
# Python (from release wheel)
pip install spatial-sdk

# Debian runtime
sudo dpkg -i libthunderbird-sdk0_*.deb

# Debian development
sudo dpkg -i libthunderbird-sdk-dev_*.deb
```
INSTALL

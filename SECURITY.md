# Security Policy

## Supported Versions

| Version | Supported          |
|---------|--------------------|
| 0.1.x   | :white_check_mark: |
| < 0.1   | :x:                |

## Reporting a Vulnerability

**Do NOT report security vulnerabilities through public GitHub issues.**

Instead, please report them responsibly via one of:

1. **GitHub Security Advisories** — [Create a private advisory](../../security/advisories/new)
2. **Email** — security@thunderbird-sdk.dev (if configured)

### What to include

- Description of the vulnerability
- Steps to reproduce or proof-of-concept
- Affected version(s)
- Potential impact assessment
- Suggested fix (if any)

### Response timeline

| Stage | Target |
|-------|--------|
| Acknowledgment | 48 hours |
| Triage & severity assessment | 5 business days |
| Fix development | Depends on severity |
| Coordinated disclosure | 90 days max |

### Severity classification

We follow [CVSS v3.1](https://www.first.org/cvss/calculator/3.1) for scoring:

| CVSS Score | Severity | Response |
|------------|----------|----------|
| 9.0–10.0 | Critical | Immediate patch release |
| 7.0–8.9 | High | Patch within 7 days |
| 4.0–6.9 | Medium | Fix in next scheduled release |
| 0.1–3.9 | Low | Fix when convenient |

## Security Measures

### Supply Chain Security

- **SLSA Build Level 2+** — All release artifacts include Sigstore-backed
  provenance attestations via `actions/attest-build-provenance`
- **Cosign keyless signing** — Docker images signed at the manifest digest
  level using Sigstore OIDC (no secrets to manage, identity bound to
  GitHub Actions workflow)
- **Artifact checksums** — SHA256SUMS.txt signed as a cosign blob with
  certificate + signature attached to every GitHub Release
- **SBOM generation** — SPDX SBOMs generated via Syft for source tree,
  container images, and release artifacts; attested via cosign

### Dependency Management

- **Dependency Review** — License + vulnerability checks on every PR
  (blocks high/critical CVEs and copyleft licenses)
- **Dependabot** — Automated security updates for GitHub Actions and
  Python dependencies
- **Minimal dependencies** — The C++ SDK has zero runtime dependencies
  beyond the standard library and pthreads; Python wheels statically
  link libstdc++

### Static Analysis

- **CodeQL** — C++ and Python static analysis on every push/PR + weekly
  schedule, using the `security-and-quality` query suite
- **OpenSSF Scorecard** — Automated supply chain health assessment

### Container Security

- **Trivy scanning** — Container images scanned for vulnerabilities on
  every push, weekly, and during release (CRITICAL blocks release)
- **Non-root runtime** — The runtime Docker image runs as a non-root
  `thunderbird` system user
- **Minimal attack surface** — Runtime image contains only the shared
  library on Ubuntu 24.04 minimal (~30 MB)

### Secret Protection

- **No long-lived secrets** — All signing uses Sigstore OIDC keyless
  (identity from the GitHub Actions OIDC token)
- **Least-privilege permissions** — Each workflow job declares only the
  minimum GitHub token permissions it needs
- **TruffleHog scanning** — Automated secret detection in commit history
- **`GITHUB_TOKEN` only** — No personal access tokens or deploy keys

### Build Integrity

- **Reproducible builds** — CMake Release builds with version injection
  from Git tags; Debian packages built via `dpkg-buildpackage`
- **Hermetic containers** — Wheels built inside official PyPA manylinux2014
  containers; Docker images use multi-stage builds
- **Build provenance** — GitHub-native attestations create unforgeable
  records linking each artifact to its source commit + build log

## Verifying Artifacts

### Verify Docker image signature

```bash
cosign verify \
  --certificate-identity-regexp "github.com/OWNER/thunderbird" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  ghcr.io/OWNER/thunderbird-runtime:1.2.3
```

### Verify release checksums

```bash
# Download checksums + signature + certificate from the release
cosign verify-blob \
  --signature SHA256SUMS.txt.sig \
  --certificate SHA256SUMS.txt.cert \
  --certificate-identity-regexp "github.com/OWNER/thunderbird" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  SHA256SUMS.txt

# Then verify individual files
sha256sum -c SHA256SUMS.txt
```

### Verify build provenance (SLSA)

```bash
# Requires GitHub CLI with attestation extension
gh attestation verify ./libthunderbird-sdk0_1.2.3_amd64.deb \
  --repo OWNER/thunderbird
```

### Verify container SBOM attestation

```bash
cosign verify-attestation \
  --type spdxjson \
  --certificate-identity-regexp "github.com/OWNER/thunderbird" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  ghcr.io/OWNER/thunderbird-runtime:1.2.3
```

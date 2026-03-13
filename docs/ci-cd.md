# CI/CD & Security

Four GitHub Actions workflows implement a three-tier artifact promotion model
(dev → staging → production):

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `ci.yml` | Push/PR | Build matrix + smoke tests + dev Docker publish |
| `release.yml` | `vX.Y.Z` / `vX.Y.Z-rc.N` tag | Full release (`.deb`, `.whl`, Docker, GitHub Release) |
| `promote.yml` | Manual | Zero-rebuild RC → production promotion |
| `security.yml` | Push/PR + weekly | CodeQL, Trivy, SBOM, Scorecard, secret scan |

**Security**: Cosign keyless signing (Sigstore OIDC), SLSA L2+ provenance,
Syft SBOM, Trivy container scanning, Dependabot, OpenSSF Scorecard.
See [SECURITY.md](../SECURITY.md) for vulnerability reporting.
See [ENVIRONMENTS.md](../ENVIRONMENTS.md) for environment protection rules.

> **Full CI/CD architecture**, caching strategy, release pipeline stages,
> artifact promotion model, and security details are documented in
> [`.github/copilot-instructions.md`](../.github/copilot-instructions.md).

## Release Artifacts

A single tag push (`git tag v1.2.3 && git push --tags`) produces:

| Artifact | Format | Platforms |
|----------|--------|-----------|
| Debian runtime pkg | `libthunderbird-sdk0_*.deb` | amd64, arm64 |
| Debian dev pkg | `libthunderbird-sdk-dev_*.deb` | amd64, arm64 |
| Python wheels | `.whl` (manylinux2014) | x86_64, aarch64 × CPython 3.9–3.12 |
| Docker runtime image | OCI multi-arch | linux/amd64, linux/arm64 |
| Docker dev image | OCI multi-arch | linux/amd64, linux/arm64 |
| Checksums | `SHA256SUMS.txt` + cosign signature | — |

## Creating a Release

```bash
# 1. Update CMakeLists.txt: project(thunderbird_sdk VERSION X.Y.Z ...)
# 2. Commit and tag:
git commit -am "release: v1.2.3"
git tag v1.2.3
git push origin main --tags
# 3. CI builds all artifacts with version injected automatically
```

For the promoted-release workflow (RC → production), see [ENVIRONMENTS.md](../ENVIRONMENTS.md).

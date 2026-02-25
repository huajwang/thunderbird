# Environment Protection Rules

Thunderbird SDK uses three GitHub environments to govern the artifact promotion pipeline. Configure these in **Settings → Environments**.

## Environment Tiers

### `dev`

| Setting | Value |
|---------|-------|
| **Triggered by** | Push to `main` (ci.yml) |
| **Required reviewers** | None (auto-approve) |
| **Wait timer** | None |
| **Deployment branches** | `main` only |
| **Artifacts** | Docker `:dev` and `:dev-<sha>` tags |
| **Purpose** | Continuous availability of latest main builds for internal testing |

### `staging`

| Setting | Value |
|---------|-------|
| **Triggered by** | RC tag push `vX.Y.Z-rc.N` (release.yml) |
| **Required reviewers** | None (auto-approve) |
| **Wait timer** | None |
| **Deployment branches** | Tags matching `v*-rc.*` |
| **Artifacts** | Docker `:X.Y.Z-rc.N` tags, pre-release GitHub Release |
| **Purpose** | Integration testing and stakeholder validation before production |

### `production`

| Setting | Value |
|---------|-------|
| **Triggered by** | Production tag `vX.Y.Z` (release.yml) or manual promote (promote.yml) |
| **Required reviewers** | 1 (minimum) |
| **Wait timer** | 0 minutes (optional: add for cool-down) |
| **Deployment branches** | Tags matching `v*` (excluding `*-rc.*` — enforced by workflow logic) |
| **Artifacts** | Docker `:X.Y.Z`, `:X.Y`, `:X`, `:latest` tags; production GitHub Release |
| **Purpose** | Production releases with full governance |

## Immutable Artifact Policy

| Rule | Enforcement |
|------|-------------|
| **Docker production tags are write-once** | `promote.yml` verifies target tags don't exist via `crane manifest` before copying |
| **GitHub Releases are write-once** | `release.yml` checks for existing release and skips gracefully; `promote.yml` fails if production release exists |
| **RC artifacts are preserved** | Promotion copies artifacts to production — RC release remains as historical record |
| **Digest preservation** | `promote.yml` verifies manifest digest matches after `crane copy` (byte-identical) |
| **No manual overwrites** | Production environment reviewer approval required for any production publish |

## Promotion Flow

```
┌──────────────────┐     push to main     ┌──────────────────┐
│    Developer      │ ──────────────────► │      dev          │
│    commits        │                      │  :dev, :dev-<sha> │
└──────────────────┘                      └──────────────────┘
        │
        │ git tag v1.2.3-rc.1
        ▼
┌──────────────────┐     auto-build       ┌──────────────────┐
│    RC tag push    │ ──────────────────► │    staging        │
│  v1.2.3-rc.1     │                      │  :1.2.3-rc.1     │
└──────────────────┘                      │  pre-release GH   │
        │                                  └──────────────────┘
        │                                          │
        │     ┌────────────────────────────────────┘
        │     │
        ▼     ▼  Two paths to production:
┌──────────────────────────────────────────────────────────────┐
│  Path A: promote.yml (zero-rebuild)                          │
│    • Manual trigger with RC tag input                        │
│    • Requires production environment approval                │
│    • crane copy: re-tags Docker manifests (no layer xfer)    │
│    • Downloads RC artifacts, verifies SHA256 + cosign        │
│    • Creates production GitHub Release with identical files   │
├──────────────────────────────────────────────────────────────┤
│  Path B: release.yml (full rebuild)                          │
│    • Push vX.Y.Z tag                                         │
│    • Full build + test + Docker push + Release               │
│    • Requires production environment approval                │
│    • Skips gracefully if promote.yml already created release │
└──────────────────────────────────────────────────────────────┘
        │
        ▼
┌──────────────────┐
│   production      │
│  :1.2.3           │
│  :1.2, :1         │
│  :latest          │
│  GitHub Release   │
└──────────────────┘
```

## Setup Instructions

### 1. Create environments in GitHub Settings

```
Settings → Environments → New environment
```

Create three environments: `dev`, `staging`, `production`.

### 2. Configure `production` environment

- **Required reviewers**: Add at least 1 team member
- **Deployment branches**: Select "Selected branches and tags"
  - Add rule: `v*` (allows both `vX.Y.Z` and `vX.Y.Z-rc.N` tags — the workflow handles the distinction)
- **Optional**: Enable "Prevent self-review" for additional governance

### 3. Configure `staging` environment

- **Required reviewers**: None (or add reviewers for extra caution)
- **Deployment branches**: Select "Selected branches and tags"
  - Add rule: `v*-rc.*`

### 4. Configure `dev` environment

- **Required reviewers**: None
- **Deployment branches**: Select "Selected branches and tags"
  - Add rule: `main`

### 5. Verify workflow permissions

Ensure the repository has:
- **Actions → General → Workflow permissions**: "Read and write permissions"
- **Actions → General → Fork pull request workflows**: Restricted (no secrets to forks)

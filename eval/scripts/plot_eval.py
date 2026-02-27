#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# Thunderbird SDK — slam_eval: Plot Generator (headless PNG output)
# ─────────────────────────────────────────────────────────────────────────────
#
# Reads TUM trajectory files + summary JSON from an eval output directory and
# generates:
#   1. trajectory_overlay.png  — Estimated vs Ground Truth 2D overlay
#   2. drift_curve.png         — Position error vs distance traveled
#
# Usage:
#   python plot_eval.py <output_dir>
#
# Requires:  numpy, matplotlib
# ─────────────────────────────────────────────────────────────────────────────

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

# ── Force non-interactive backend (no GUI) ───────────────────────────────────
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ═════════════════════════════════════════════════════════════════════════════
#  I/O helpers
# ═════════════════════════════════════════════════════════════════════════════

def read_tum(path: str | Path) -> np.ndarray:
    """Read a TUM trajectory file.

    Returns (N, 8) array: [timestamp, tx, ty, tz, qx, qy, qz, qw].
    Timestamps are in seconds.
    """
    rows: list[list[float]] = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                rows.append([float(x) for x in parts[:8]])
    if not rows:
        return np.empty((0, 8))
    return np.asarray(rows)


def associate(
    est: np.ndarray,
    gt: np.ndarray,
    max_diff_s: float = 0.05,
) -> list[tuple[int, int]]:
    """Nearest-neighbour timestamp association with rejection threshold."""
    matches: list[tuple[int, int]] = []
    gt_ts = gt[:, 0]
    for i in range(len(est)):
        idx = int(np.argmin(np.abs(gt_ts - est[i, 0])))
        if abs(gt_ts[idx] - est[i, 0]) <= max_diff_s:
            matches.append((i, idx))
    return matches


# ═════════════════════════════════════════════════════════════════════════════
#  SE(3) Umeyama alignment
# ═════════════════════════════════════════════════════════════════════════════

def umeyama_alignment(
    src: np.ndarray,
    tgt: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rigid SE(3) alignment (no scale).

    Finds R, t  such that  tgt ≈ R @ src + t.

    Parameters
    ----------
    src, tgt : (N, 3) arrays

    Returns
    -------
    R : (3, 3)  rotation matrix
    t : (3,)    translation vector
    """
    assert src.shape == tgt.shape and src.ndim == 2 and src.shape[1] == 3
    mu_s = src.mean(axis=0)
    mu_t = tgt.mean(axis=0)
    H = (src - mu_s).T @ (tgt - mu_t) / len(src)
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(Vt))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_t - R @ mu_s
    return R, t


# ═════════════════════════════════════════════════════════════════════════════
#  Plot 1: Estimated vs Ground Truth overlay (2D top-down XY)
# ═════════════════════════════════════════════════════════════════════════════

_BLUE = "#1f77b4"
_RED = "#d62728"
_GREEN = "#2ca02c"


def plot_trajectory_overlay(
    est_xyz: np.ndarray,
    gt_xyz: np.ndarray,
    out_path: str | Path,
    dataset_name: str = "",
) -> None:
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(
        gt_xyz[:, 0], gt_xyz[:, 1],
        color=_RED, linewidth=1.6, label="Ground Truth", alpha=0.85, zorder=2,
    )
    ax.plot(
        est_xyz[:, 0], est_xyz[:, 1],
        color=_BLUE, linewidth=1.2, label="Estimated (aligned)", alpha=0.85,
        zorder=3,
    )

    # Start / end markers
    ax.plot(
        gt_xyz[0, 0], gt_xyz[0, 1],
        "o", color=_GREEN, markersize=9, markeredgecolor="k",
        markeredgewidth=0.8, label="Start", zorder=5,
    )
    ax.plot(
        gt_xyz[-1, 0], gt_xyz[-1, 1],
        "s", color=_RED, markersize=9, markeredgecolor="k",
        markeredgewidth=0.8, label="End", zorder=5,
    )

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    title = "Trajectory Overlay"
    if dataset_name:
        title += f"  \u2014  {dataset_name}"
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.legend(fontsize=10, loc="best")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25, linewidth=0.6)

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════════
#  Plot 2: Drift accumulation curve
# ═════════════════════════════════════════════════════════════════════════════

def plot_drift_curve(
    cum_dist: np.ndarray,
    errors: np.ndarray,
    out_path: str | Path,
    dataset_name: str = "",
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(cum_dist, errors, color=_BLUE, linewidth=1.0, alpha=0.8)
    ax.fill_between(cum_dist, 0, errors, alpha=0.12, color=_BLUE)

    # Reference line: mean error
    mean_err = float(np.mean(errors))
    ax.axhline(mean_err, color=_RED, linewidth=0.8, linestyle="--", alpha=0.6,
               label=f"Mean = {mean_err:.3f} m")

    ax.set_xlabel("Distance traveled (m)", fontsize=12)
    ax.set_ylabel("Position error (m)", fontsize=12)
    title = "Drift Accumulation"
    if dataset_name:
        title += f"  \u2014  {dataset_name}"
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.legend(fontsize=10, loc="upper left")
    ax.grid(True, alpha=0.25, linewidth=0.6)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════════
#  Plot (no GT): estimated trajectory only
# ═════════════════════════════════════════════════════════════════════════════

def plot_est_only(
    est_xyz: np.ndarray,
    out_path: str | Path,
    dataset_name: str = "",
) -> None:
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(
        est_xyz[:, 0], est_xyz[:, 1],
        color=_BLUE, linewidth=1.2, label="Estimated", alpha=0.85,
    )
    ax.plot(
        est_xyz[0, 0], est_xyz[0, 1],
        "o", color=_GREEN, markersize=9, markeredgecolor="k",
        markeredgewidth=0.8, label="Start", zorder=5,
    )

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    title = "Estimated Trajectory"
    if dataset_name:
        title += f"  \u2014  {dataset_name}"
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.legend(fontsize=10, loc="best")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25, linewidth=0.6)

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <output_dir>", file=sys.stderr)
        return 1

    output_dir = Path(sys.argv[1])
    est_path = output_dir / "trajectory_est.tum"
    gt_path = output_dir / "trajectory_gt.tum"
    json_path = output_dir / "summary.json"

    if not est_path.exists():
        print(f"[plot] {est_path} not found — nothing to plot", file=sys.stderr)
        return 1

    est = read_tum(est_path)
    if est.shape[0] < 2:
        print("[plot] estimated trajectory has < 2 poses — skipping", file=sys.stderr)
        return 1

    # Read dataset name from summary JSON (optional).
    dataset_name = ""
    if json_path.exists():
        try:
            with open(json_path) as fh:
                dataset_name = json.load(fh).get("dataset", "")
        except (json.JSONDecodeError, KeyError):
            pass

    has_gt = gt_path.exists()

    if has_gt:
        gt = read_tum(gt_path)
        if gt.shape[0] < 2:
            print("[plot] GT trajectory has < 2 poses — plotting est only",
                  file=sys.stderr)
            has_gt = False

    if has_gt:
        # ── Associate & align ────────────────────────────────────────────
        matches = associate(est, gt)
        if len(matches) < 3:
            print(f"[plot] only {len(matches)} timestamp matches — "
                  "plotting est only", file=sys.stderr)
            has_gt = False

    if has_gt:
        est_idx = np.array([m[0] for m in matches])
        gt_idx = np.array([m[1] for m in matches])
        est_xyz = est[est_idx, 1:4]
        gt_xyz = gt[gt_idx, 1:4]

        R, t = umeyama_alignment(est_xyz, gt_xyz)
        est_aligned = (R @ est_xyz.T).T + t

        # ── Plot 1: Trajectory overlay ───────────────────────────────────
        overlay_path = output_dir / "trajectory_overlay.png"
        plot_trajectory_overlay(est_aligned, gt_xyz, overlay_path, dataset_name)
        print(f"[plot] wrote {overlay_path}", file=sys.stderr)

        # ── Plot 2: Drift accumulation ───────────────────────────────────
        errors = np.linalg.norm(est_aligned - gt_xyz, axis=1)
        deltas = np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1)
        cum_dist = np.concatenate([[0.0], np.cumsum(deltas)])

        drift_path = output_dir / "drift_curve.png"
        plot_drift_curve(cum_dist, errors, drift_path, dataset_name)
        print(f"[plot] wrote {drift_path}", file=sys.stderr)
    else:
        # No usable GT — plot estimated trajectory only.
        overlay_path = output_dir / "trajectory_overlay.png"
        plot_est_only(est[:, 1:4], overlay_path, dataset_name)
        print(f"[plot] wrote {overlay_path} (no GT)", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())

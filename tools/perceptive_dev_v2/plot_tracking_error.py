"""Stage 8 analysis — WBC tracking error vs MPC plan, baseline vs refined.

Reads two tick.csv logs (baseline and refined arms of the same noise scenario)
and renders a vertically-stacked 2-row plot showing the per-tick tracking error
between the actual robot state (`meas_rbd`) and the MPC plan target
(`opt_state`).  The time axis on both subplots is shifted so the box-edge
crossing moment (base_x first reaches a threshold) lines up at t=0.

Layout of meas_rbd (36 components, OCS2 RBD ordering):
    [0:3]   theta_zyx (Z-Y-X intrinsic Euler)
    [3:6]   r_b (base position, world frame)
    [6:18]  q_j (12 joint positions)
    [18:21] omega (base angular velocity)
    [21:24] r_b_dot (base linear velocity)
    [24:36] q_j_dot (12 joint velocities)

Layout of opt_state (24, centroidal model):
    [0:3]   p_com_dot
    [3:6]   omega_com
    [6:9]   r_b
    [9:12]  theta_zyx
    [12:24] q_j

So the directly comparable slices are:
    base position : opt_x[6:9]   ↔  meas_rbd[3:6]
    theta_zyx     : opt_x[9:12]  ↔  meas_rbd[0:3]
    joint pos     : opt_x[12:24] ↔  meas_rbd[6:18]

Usage:
    python3 plot_tracking_error.py <baseline_dir> <refined_dir>
        [--save out.png] [--align-x 0.8]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless safe — falls back to interactive if DISPLAY available
import matplotlib.pyplot as plt
import numpy as np


def load_tick(csv_path: Path) -> dict:
    """Return dict of column-name -> 1D float array. Read once, fast."""
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader]
    n = len(rows)
    cols = {h: np.empty(n, dtype=float) for h in header}
    for i, row in enumerate(rows):
        for h, v in zip(header, row):
            try:
                cols[h][i] = float(v)
            except ValueError:
                cols[h][i] = np.nan
    return cols


def compute_errors(d: dict) -> dict:
    """Compute per-tick tracking errors.

    Returns dict with keys: t, base_pos_err, theta_err, q_err, total_err,
    base_x, refined_active.
    """
    t = d["t"]
    # Slices
    opt_rb     = np.stack([d[f"opt_x{i}"] for i in range(6, 9)],   axis=1)  # (T,3)
    opt_theta  = np.stack([d[f"opt_x{i}"] for i in range(9, 12)],  axis=1)
    opt_qj     = np.stack([d[f"opt_x{i}"] for i in range(12, 24)], axis=1)

    meas_theta = np.stack([d[f"meas_rbd{i}"] for i in range(0, 3)],   axis=1)
    meas_rb    = np.stack([d[f"meas_rbd{i}"] for i in range(3, 6)],   axis=1)
    meas_qj    = np.stack([d[f"meas_rbd{i}"] for i in range(6, 18)],  axis=1)

    base_err  = meas_rb    - opt_rb
    theta_err = meas_theta - opt_theta
    q_err     = meas_qj    - opt_qj

    return {
        "t":               t,
        "base_pos_err":    np.linalg.norm(base_err,  axis=1),  # (T,)
        "theta_err":       np.linalg.norm(theta_err, axis=1),
        "q_err":           np.linalg.norm(q_err,     axis=1),
        "base_x":          meas_rb[:, 0],
        "base_y":          meas_rb[:, 1],
        "base_z":          meas_rb[:, 2],
        "refined_active":  d["refined_active"],
        # Component-wise raw signals for color shading / debugging
        "opt_rb_x":        opt_rb[:, 0],
        "meas_rb_x":       meas_rb[:, 0],
        "opt_rb_z":        opt_rb[:, 2],
        "meas_rb_z":       meas_rb[:, 2],
    }


def crossing_time(t: np.ndarray, base_x: np.ndarray, threshold: float) -> float | None:
    """Find first time when base_x crosses threshold from below."""
    above = base_x >= threshold
    if not above.any():
        return None
    # First True index
    idx = int(np.argmax(above))
    if idx == 0:
        # Already above at start — invalid
        return None
    # Linear interpolation between idx-1 and idx
    x0, x1 = base_x[idx - 1], base_x[idx]
    t0, t1 = t[idx - 1],     t[idx]
    if x1 == x0:
        return float(t1)
    alpha = (threshold - x0) / (x1 - x0)
    return float(t0 + alpha * (t1 - t0))


def shade_refined_window(ax, t: np.ndarray, refined_active: np.ndarray,
                         t_offset: float = 0.0):
    """Shade the time intervals where refined_active == 1 (light yellow)."""
    if refined_active.size == 0:
        return
    active = refined_active.astype(bool)
    # Find runs
    diff = np.diff(active.astype(int))
    starts = np.where(diff == 1)[0] + 1
    ends = np.where(diff == -1)[0] + 1
    if active[0]:
        starts = np.insert(starts, 0, 0)
    if active[-1]:
        ends = np.append(ends, active.size)
    for s, e in zip(starts, ends):
        ax.axvspan(t[s] - t_offset, t[min(e, len(t) - 1)] - t_offset,
                   color="khaki", alpha=0.35, zorder=0,
                   label="refined override" if s == starts[0] else None)


def plot_pair(baseline_dir: Path, refined_dir: Path, save_path: Path | None,
              align_x: float, t_window=(-2.0, 3.0)):
    print(f"[load] baseline: {baseline_dir}")
    base = compute_errors(load_tick(baseline_dir / "tick.csv"))
    print(f"[load] refined:  {refined_dir}")
    ref  = compute_errors(load_tick(refined_dir  / "tick.csv"))

    # Find box-edge crossing in each, align so crossing → t=0.
    t_x_base = crossing_time(base["t"], base["base_x"], align_x)
    t_x_ref  = crossing_time(ref ["t"], ref ["base_x"], align_x)
    print(f"[align] baseline base_x={align_x} crossing at t={t_x_base}")
    print(f"[align] refined  base_x={align_x} crossing at t={t_x_ref}")

    if t_x_base is None or t_x_ref is None:
        print("[warn] one trial never crosses align-x — using raw timeline (no alignment)")
        t_x_base = t_x_ref = 0.0

    # Compute shared y-axis ranges over the visible window for fair comparison.
    def in_window(d, t_off):
        rel = d["t"] - t_off
        return (rel >= t_window[0]) & (rel <= t_window[1])

    base_mask = in_window(base, t_x_base)
    ref_mask  = in_window(ref,  t_x_ref)

    err_max = max(
        np.nanmax(base["base_pos_err"][base_mask]) if base_mask.any() else 0.0,
        np.nanmax(base["theta_err"]   [base_mask]) if base_mask.any() else 0.0,
        np.nanmax(base["q_err"]       [base_mask]) if base_mask.any() else 0.0,
        np.nanmax(ref ["base_pos_err"][ref_mask])  if ref_mask.any()  else 0.0,
        np.nanmax(ref ["theta_err"]   [ref_mask])  if ref_mask.any()  else 0.0,
        np.nanmax(ref ["q_err"]       [ref_mask])  if ref_mask.any()  else 0.0,
    )
    err_max = max(err_max, 0.05) * 1.1  # padding

    z_min = min(
        np.nanmin(base["meas_rb_z"][base_mask]) if base_mask.any() else 0.0,
        np.nanmin(base["opt_rb_z"] [base_mask]) if base_mask.any() else 0.0,
        np.nanmin(ref ["meas_rb_z"][ref_mask])  if ref_mask.any()  else 0.0,
        np.nanmin(ref ["opt_rb_z"] [ref_mask])  if ref_mask.any()  else 0.0,
    )
    z_max = max(
        np.nanmax(base["meas_rb_z"][base_mask]) if base_mask.any() else 0.0,
        np.nanmax(base["opt_rb_z"] [base_mask]) if base_mask.any() else 0.0,
        np.nanmax(ref ["meas_rb_z"][ref_mask])  if ref_mask.any()  else 0.0,
        np.nanmax(ref ["opt_rb_z"] [ref_mask])  if ref_mask.any()  else 0.0,
    )
    z_pad = (z_max - z_min) * 0.1 + 0.005
    z_lim = (z_min - z_pad, z_max + z_pad)

    # 4-subplot layout, single column, sharex.
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    # ---- 1. Baseline error norms ----
    ax = axes[0]
    t_rel = base["t"] - t_x_base
    shade_refined_window(ax, base["t"], base["refined_active"], t_offset=t_x_base)
    ax.plot(t_rel, base["base_pos_err"], color="tab:blue",   lw=1.2, label="‖base pos err‖ [m]")
    ax.plot(t_rel, base["theta_err"],    color="tab:orange", lw=1.2, label="‖theta_zyx err‖ [rad]")
    ax.plot(t_rel, base["q_err"],        color="tab:green",  lw=1.2, label="‖joint q err‖ [rad]")
    ax.axvline(0.0, color="black", lw=0.7, alpha=0.5, label="box edge crossing")
    ax.set_ylim(0, err_max)
    ax.set_ylabel("error norm")
    ax.set_title(f"Baseline (refiner OFF)  —  base_x crossing 0.80m at t={t_x_base:.2f}s")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # ---- 2. Refined error norms ----
    ax = axes[1]
    t_rel = ref["t"] - t_x_ref
    shade_refined_window(ax, ref["t"], ref["refined_active"], t_offset=t_x_ref)
    ax.plot(t_rel, ref["base_pos_err"], color="tab:blue",   lw=1.2, label="‖base pos err‖ [m]")
    ax.plot(t_rel, ref["theta_err"],    color="tab:orange", lw=1.2, label="‖theta_zyx err‖ [rad]")
    ax.plot(t_rel, ref["q_err"],        color="tab:green",  lw=1.2, label="‖joint q err‖ [rad]")
    ax.axvline(0.0, color="black", lw=0.7, alpha=0.5, label="box edge crossing")
    ax.set_ylim(0, err_max)
    ax.set_ylabel("error norm")
    ax.set_title(f"Refined (refiner ON)  —  base_x crossing 0.80m at t={t_x_ref:.2f}s")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # ---- 3. Baseline base z trajectory (actual vs planned) ----
    ax = axes[2]
    t_rel = base["t"] - t_x_base
    shade_refined_window(ax, base["t"], base["refined_active"], t_offset=t_x_base)
    ax.plot(t_rel, base["meas_rb_z"], color="tab:blue", lw=1.4, label="actual (meas)")
    ax.plot(t_rel, base["opt_rb_z"],  color="tab:red",  lw=1.4, ls="--", label="MPC plan")
    ax.axvline(0.0, color="black", lw=0.7, alpha=0.5)
    ax.set_ylim(z_lim)
    ax.set_ylabel("base z [m]")
    ax.set_title("Baseline — base z: actual vs MPC planned target")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # ---- 4. Refined base z trajectory (actual vs refined target) ----
    ax = axes[3]
    t_rel = ref["t"] - t_x_ref
    shade_refined_window(ax, ref["t"], ref["refined_active"], t_offset=t_x_ref)
    ax.plot(t_rel, ref["meas_rb_z"], color="tab:blue", lw=1.4, label="actual (meas)")
    ax.plot(t_rel, ref["opt_rb_z"],  color="tab:red",  lw=1.4, ls="--",
            label="optimized target (refined when shaded, raw MPC otherwise)")
    ax.axvline(0.0, color="black", lw=0.7, alpha=0.5)
    ax.set_ylim(z_lim)
    ax.set_xlabel("time relative to box-edge crossing  [s]")
    ax.set_ylabel("base z [m]")
    ax.set_title("Refined — base z: actual vs WBC tracking target")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # Apply zoom window
    axes[0].set_xlim(*t_window)

    fig.suptitle(f"WBC tracking analysis (zoom {t_window[0]}~{t_window[1]}s relative to box-edge crossing)",
                 fontsize=12)
    plt.tight_layout(rect=(0, 0, 1, 0.97))

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=120)
        print(f"[save] {save_path}")
    else:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("baseline_dir", type=Path)
    ap.add_argument("refined_dir",  type=Path)
    ap.add_argument("--save",       type=Path, default=None)
    ap.add_argument("--align-x",    type=float, default=0.8,
                    help="Align time at base_x crossing this threshold [m]. "
                         "Default 0.8 = colleague's basic_step.xml box1 edge.")
    ap.add_argument("--t-window", type=float, nargs=2, default=(-2.0, 3.0),
                    metavar=("T_LO", "T_HI"),
                    help="Zoom window relative to box-edge crossing [s,s]. "
                         "Default (-2, 3).")
    args = ap.parse_args()

    plot_pair(args.baseline_dir, args.refined_dir, args.save, args.align_x,
              t_window=tuple(args.t_window))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

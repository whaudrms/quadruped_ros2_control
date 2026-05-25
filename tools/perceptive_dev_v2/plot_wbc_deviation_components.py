#!/usr/bin/env python3
"""WBC tracking deviation, per-component (x/y/z and roll/pitch/yaw), 2×2 panel.

Mirrors note/sizesample.png layout:
  rows: position deviation (x/y/z)  /  orientation deviation (roll/pitch/yaw)
  cols: ON,no-splice  /  OFF
One figure per Δz.

Same scenario monitoring-window crop (option b) as plot_wbc_tracking_error.py:
  tick.csv t_rel ∈ [1.0, 6.0]  (monitoring_start_sec=3.0 to timeout_sec=8.0)
  displayed as x ∈ [0, 5] s.

Single representative trial (run1) per panel — matches sizesample.png style
(per-tick raw deviation, not aggregated).
"""
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


RESULTS = Path("/home/cora/GO2_ws/quadruped_ros2_control/tools/perceptive_dev_v2/results")
OUT_DIR = Path("/home/cora/GO2_ws/quadruped_ros2_control/note")

# Same monitoring-window crop as plot_wbc_tracking_error.py (option b).
T_REL_MONITOR_START = 1.0
T_REL_MONITOR_END   = 6.0
DISPLAY_DURATION    = T_REL_MONITOR_END - T_REL_MONITOR_START  # 5.0 s

# Match sizesample.png aspect (~12 × 7 in); slight width adjust for 2 conditions.
FIG_SIZE = (12, 7)


# Use run1 of each condition as the representative trial (matches sizesample
# style of single-trial raw traces).
TRIAL_RUNS = {
    ("Δz=-0.03", "ON,no-splice"):
        "20260510_191632_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_ON_nosplice_offM03_d05_P10_sqp2_run1",
    ("Δz=-0.03", "OFF"):
        "20260510_191702_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_OFF_offM03_d05_P10_sqp2_run1",
    ("Δz=+0.03", "ON,no-splice"):
        "20260510_193722_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_ON_nosplice_offP03_d05_P10_sqp2_run1",
    ("Δz=+0.03", "OFF"):
        "20260510_193752_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_OFF_offP03_d05_P10_sqp2_run1",
}


def load_tick(tick_csv: Path):
    """Return (t, opt_x[24], meas_rbd[36], planned_mode) per tick."""
    with open(tick_csv) as f:
        reader = csv.reader(f)
        header = next(reader)
        n_expected = len(header)
        t_col = header.index("t")
        opt_x_cols = [header.index(f"opt_x{i}") for i in range(24)]
        meas_cols  = [header.index(f"meas_rbd{i}") for i in range(36)]
        mode_col = header.index("planned_mode")
        ts, opt_xs, meas_xs, modes = [], [], [], []
        for row in reader:
            if len(row) != n_expected:
                continue
            try:
                vals = [float(x) for x in row]
            except ValueError:
                continue
            ts.append(vals[t_col])
            opt_xs.append([vals[c] for c in opt_x_cols])
            meas_xs.append([vals[c] for c in meas_cols])
            modes.append(int(vals[mode_col]))
    if not ts:
        return None
    return np.array(ts), np.array(opt_xs), np.array(meas_xs), np.array(modes)


# Mode encoding (per MotionPhaseDefinition.h:119-123):
#   mode = LF*8 + RF*4 + LH*2 + RH*1   →   bit i of mode = leg [LF, RF, LH, RH][3-i]
LEG_NAMES  = ["LF", "RF", "LH", "RH"]
LEG_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]


def mode_to_contacts(mode: int):
    return np.array([(mode >> (3 - i)) & 1 for i in range(4)], dtype=int)


def find_first_box2_touchdown_disp(t, planned_mode, meas_rbd, body_x_threshold=0.4):
    """Display time of the FIRST scheduled rising-edge touchdown (any leg) AFTER body_x
    crosses the threshold (front feet onto box2). Returns None if not in window.
    """
    body_x = meas_rbd[:, 3]
    prev = mode_to_contacts(int(planned_mode[0]))
    for i in range(1, len(planned_mode)):
        curr = mode_to_contacts(int(planned_mode[i]))
        if body_x[i] >= body_x_threshold:
            for leg in range(4):
                if prev[leg] == 0 and curr[leg] == 1:
                    t_disp = float(t[i] - t[0]) - T_REL_MONITOR_START
                    if 0.0 <= t_disp <= DISPLAY_DURATION:
                        return t_disp
                    return None
        prev = curr
    return None


def signed_deviations(t, opt_x, meas_rbd):
    """Per-tick signed deviations (meas - opt) for position and orientation.

    Layouts:
      opt_x: [v_com(3), w_c(3), r_b(3), theta_zyx(3), q_j(12)]
        → r_b at [6:9],  theta_zyx at [9:12]
      meas_rbd: [theta_zyx(3), r_b(3), q_j(12), w_b(3), r_b_dot(3), q_j_dot(12)]
        → theta_zyx at [0:3],  r_b at [3:6]

    Returns:
      pos_err  shape (n, 3)  components [x, y, z]   in meters
      ori_err  shape (n, 3)  components [yaw, pitch, roll]   in radians
    """
    pos_err = meas_rbd[:, 3:6] - opt_x[:, 6:9]
    ori_err = meas_rbd[:, 0:3] - opt_x[:, 9:12]
    return pos_err, ori_err


def crop_to_monitor_window(t, *series):
    """Trim each series to t_rel ∈ [T_REL_MONITOR_START, T_REL_MONITOR_END].
    Re-zero t to start at 0 for display.
    """
    t_rel = t - t[0]
    mask = (t_rel >= T_REL_MONITOR_START) & (t_rel <= T_REL_MONITOR_END)
    t_disp = t_rel[mask] - T_REL_MONITOR_START
    out = [t_disp]
    for s in series:
        out.append(s[mask])
    return out


def plot_one_dz(dz_label: str, on_dir: str, off_dir: str, out_png: Path):
    fig, axes = plt.subplots(2, 2, figsize=FIG_SIZE, sharex=True)

    # Component colors — match sizesample's default tab cycle (x=blue, y=orange, z=green
    # for position; roll/pitch/yaw for orientation).
    pos_components = [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]
    # theta_zyx layout is [yaw, pitch, roll]. Plot in roll/pitch/yaw order with consistent colors.
    ori_components = [("roll", "tab:blue"), ("pitch", "tab:orange"), ("yaw", "tab:green")]
    ori_idx_map = {"yaw": 0, "pitch": 1, "roll": 2}

    panel_meta = [
        # (ax_row, ax_col, dir_name, condition_label, panel_letter)
        (0, 0, on_dir,  "with robust phase",    "(A)"),
        (0, 1, off_dir, "without robust phase", "(B)"),
        (1, 0, on_dir,  "with robust phase",    "(C)"),
        (1, 1, off_dir, "without robust phase", "(D)"),
    ]
    for row, col, dname, cond_label, letter in panel_meta:
        ax = axes[row, col]
        loaded = load_tick(RESULTS / dname / "tick.csv")
        if loaded is None:
            ax.set_title(f"{letter} {cond_label} — (missing tick.csv)")
            continue
        t, opt_x, meas_rbd, planned_mode = loaded
        pos_err, ori_err = signed_deviations(t, opt_x, meas_rbd)
        if row == 0:
            t_disp, pe = crop_to_monitor_window(t, pos_err)
            for k, (label, color) in enumerate(pos_components):
                ax.plot(t_disp, pe[:, k], color=color, label=label, linewidth=0.7)
            ax.set_ylabel("Position tracking error [m]" if col == 0 else "")
            ax.set_title(f"{letter} {cond_label}: position deviation (x/y/z)", fontsize=11)
        else:
            t_disp, oe = crop_to_monitor_window(t, ori_err)
            for label, color in ori_components:
                k = ori_idx_map[label]
                ax.plot(t_disp, oe[:, k], color=color, label=label, linewidth=0.7)
            ax.set_ylabel("Orientation tracking error [rad]" if col == 0 else "")
            ax.set_title(f"{letter} {cond_label}: orientation deviation (roll/pitch/yaw)", fontsize=11)
            ax.set_xlabel("Time [s]")
        ax.set_xlim(0, DISPLAY_DURATION)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="black", linewidth=0.5, alpha=0.5)
        ax.legend(loc="lower left", fontsize=9)

    fig.suptitle(
        r"comparison of $\mathbf{WBC}$ tracking error" + f"   —  {dz_label}",
        fontsize=13, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"saved {out_png}")


def main():
    plot_one_dz(
        "Δz = -0.03",
        TRIAL_RUNS[("Δz=-0.03", "ON,no-splice")],
        TRIAL_RUNS[("Δz=-0.03", "OFF")],
        OUT_DIR / "wbc_deviation_components_v2_dz-003.png",
    )
    plot_one_dz(
        "Δz = +0.03",
        TRIAL_RUNS[("Δz=+0.03", "ON,no-splice")],
        TRIAL_RUNS[("Δz=+0.03", "OFF")],
        OUT_DIR / "wbc_deviation_components_v2_dz+003.png",
    )


if __name__ == "__main__":
    main()

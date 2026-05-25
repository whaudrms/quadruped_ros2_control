#!/usr/bin/env python3
"""Plot WBC tracking error vs MPC plan, aggregated over n trials per condition.

Goal: visually verify the claim "robust phase makes the MPC policy easier
for WBC to track under terrain perception uncertainty". A successful claim
shows lower tracking error (and tighter inter-trial spread) for ON,no-splice
than for OFF.

Tracking error is computed per-tick as the difference between MPC's
optimized state (opt_x) and the measured RBD state (meas_rbd), in three
categories:

  base_pos_err    = ||meas r_b   - opt r_b   ||_2     [m]
  base_ori_err    = ||meas theta - opt theta ||_2     [rad → deg in plot]
  joint_pos_err   = ||meas q_j   - opt q_j   ||_2     [rad]

Layout keys (per StateOCS2.cpp:99 and plot_robust_phase.py header):
  opt_x:    [v_com(3), w_c(3), r_b(3), theta_zyx(3), q_j(12)]                       (24)
  meas_rbd: [theta_zyx(3), r_b(3), q_j(12), w_b(3), r_b_dot(3), q_j_dot(12)]        (36)

Inputs: hard-coded list of trial directories below.
Output: <out_dir>/wbc_tracking_error_v2_pm03.png
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
OUT_PNG = OUT_DIR / "wbc_tracking_error_v2_pm03.png"


# 4 conditions × 3 trials = 12 trials.
TRIALS = {
    ("Δz=-0.03", "ON,no-splice"): [
        "20260510_191632_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_ON_nosplice_offM03_d05_P10_sqp2_run1",
        "20260510_191731_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_ON_nosplice_offM03_d05_P10_sqp2_run2",
        "20260510_191831_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_ON_nosplice_offM03_d05_P10_sqp2_run3",
    ],
    ("Δz=-0.03", "OFF"): [
        "20260510_191702_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_OFF_offM03_d05_P10_sqp2_run1",
        "20260510_191801_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_OFF_offM03_d05_P10_sqp2_run2",
        "20260510_191901_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_OFF_offM03_d05_P10_sqp2_run3",
    ],
    ("Δz=+0.03", "ON,no-splice"): [
        "20260510_193722_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_ON_nosplice_offP03_d05_P10_sqp2_run1",
        "20260510_193822_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_ON_nosplice_offP03_d05_P10_sqp2_run2",
        "20260510_193922_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_ON_nosplice_offP03_d05_P10_sqp2_run3",
    ],
    ("Δz=+0.03", "OFF"): [
        "20260510_193752_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_OFF_offP03_d05_P10_sqp2_run1",
        "20260510_193852_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_OFF_offP03_d05_P10_sqp2_run2",
        "20260510_193952_basic_step_short_v2_perceptive_dev_v2_v2_ablation_n3_p03_OFF_offP03_d05_P10_sqp2_run3",
    ],
}


def load_tick(tick_csv: Path):
    """Return (t, opt_x[24], meas_rbd[36], planned_mode) per tick.
    Drops malformed lines. planned_mode is int per tick (mode number).
    """
    with open(tick_csv) as f:
        reader = csv.reader(f)
        header = next(reader)
        n_expected = len(header)
        t_col = header.index("t")
        opt_x_cols = [header.index(f"opt_x{i}") for i in range(24)]
        meas_cols = [header.index(f"meas_rbd{i}") for i in range(36)]
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


# Mode encoding: mode = LF*8 + RF*4 + LH*2 + RH*1  (per MotionPhaseDefinition.h:119-123)
# Returns 4-bit array [LF, RF, LH, RH] (1 = stance, 0 = swing).
def mode_to_contacts(mode: int):
    return np.array([(mode >> (3 - i)) & 1 for i in range(4)], dtype=int)


LEG_NAMES   = ["LF", "RF", "LH", "RH"]
LEG_COLORS  = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]   # tab:blue/orange/green/red


def extract_scheduled_touchdowns(t, planned_mode):
    """Return dict: leg_idx -> list of t at scheduled swing→stance rising edges.

    Detected from the planned_mode column (= what the MPC plan said the contact
    pattern is at this tick, post any splice). The first tick is skipped since
    we have no previous-tick comparison.
    """
    out = {leg: [] for leg in range(4)}
    prev = mode_to_contacts(int(planned_mode[0]))
    for i in range(1, len(planned_mode)):
        curr = mode_to_contacts(int(planned_mode[i]))
        for leg in range(4):
            if prev[leg] == 0 and curr[leg] == 1:
                out[leg].append(float(t[i]))
        prev = curr
    return out


def compute_errors(t, opt_x, meas_rbd):
    """Per-tick WBC tracking error SQUARED NORM (||·||² = Σ xᵢ²) per category.
    Units: position [m²], orientation [deg²] (converted from rad²), joint [rad²].
    """
    base_pos_diff = meas_rbd[:, 3:6] - opt_x[:, 6:9]
    base_pos_err = np.sum(base_pos_diff ** 2, axis=1)              # m²

    base_ori_diff_rad = meas_rbd[:, 0:3] - opt_x[:, 9:12]
    base_ori_diff_deg = np.degrees(base_ori_diff_rad)
    base_ori_err = np.sum(base_ori_diff_deg ** 2, axis=1)          # deg²

    joint_pos_diff = meas_rbd[:, 6:18] - opt_x[:, 12:24]
    joint_pos_err = np.sum(joint_pos_diff ** 2, axis=1)            # rad²

    return base_pos_err, base_ori_err, joint_pos_err


def find_first_box2_touchdown_disp(t, planned_mode, meas_rbd, body_x_threshold=0.4):
    """Return display time (in monitoring-window-relative seconds, x=0 ⇔ scenario t=3.0) of
    the first scheduled rising-edge touchdown (any leg) AFTER body_x passes the threshold.

    body_x_threshold=0.4 means front feet (hip offset ≈ +0.2) are already past box1 edge
    (x=0.6) by the time of touchdown — this catches the first box2 contact.
    Returns None if no such event is in the cropped window.
    """
    body_x = meas_rbd[:, 3]
    prev = mode_to_contacts(int(planned_mode[0]))
    for i in range(1, len(planned_mode)):
        curr = mode_to_contacts(int(planned_mode[i]))
        if body_x[i] >= body_x_threshold:
            for leg in range(4):
                if prev[leg] == 0 and curr[leg] == 1:
                    t_rel = float(t[i] - t[0])
                    t_disp = t_rel - T_REL_MONITOR_START
                    if 0.0 <= t_disp <= DISPLAY_DURATION:
                        return t_disp
                    return None
        prev = curr
    return None


# Scenario monitoring window mapping (b):
#   Yaml: stand 1.0 + hold 0.5 + enter_ocs2 0.5 + stabilize_gait 1.0 + forward 5.0 = 8.0 s
#   monitoring_start_sec = 3.0, timeout_sec = 8.0  → 5 s monitoring window
#   StateOCS2 activates at end of enter_ocs2 (≈ scenario t = 2.0 s) → tick.csv t[0]
#   So monitoring window in t_rel = t - t[0]: [1.0, 6.0]
#   We display the cropped window with x-axis re-zeroed to [0, 5] s.
T_REL_MONITOR_START = 1.0   # tick.csv t_rel of monitoring_start_sec=3.0
T_REL_MONITOR_END   = 6.0   # tick.csv t_rel of timeout_sec=8.0
DISPLAY_DURATION    = T_REL_MONITOR_END - T_REL_MONITOR_START   # 5.0 s


def trial_relative_time_and_errors(tick_path: Path):
    """Load trial, compute errors, return scenario-relative time + 3 error series + mode."""
    loaded = load_tick(tick_path)
    if loaded is None:
        return None
    t, opt_x, meas_rbd, planned_mode = loaded
    if t.size < 5:
        return None
    base_pos_err, base_ori_err_deg, joint_pos_err = compute_errors(t, opt_x, meas_rbd)
    t_rel = t - t[0]   # scenario-relative time
    return t_rel, base_pos_err, base_ori_err_deg, joint_pos_err, planned_mode


def aggregate_condition(trials):
    """Interpolate trials onto a common time grid; return (t_grid, mean, std) per metric.

    Cropped to scenario monitoring window (option b): t_rel ∈ [1.0, 6.0], displayed as
    x ∈ [0, 5] s (forward command active period). All 4 conditions use this same grid
    so x-axis lengths match.
    """
    loaded = []
    for tdir in trials:
        rec = trial_relative_time_and_errors(RESULTS / tdir / "tick.csv")
        if rec is None:
            print(f"[warn] skipping malformed tick.csv: {tdir}", file=sys.stderr)
            continue
        loaded.append(rec)
    if not loaded:
        return None
    # Display grid is uniform across all conditions: 0 to DISPLAY_DURATION at 200 Hz.
    display_t_grid = np.arange(0.0, DISPLAY_DURATION + 1e-9, 0.005)
    # Source query times in tick.csv t_rel space.
    src_t_query = display_t_grid + T_REL_MONITOR_START

    out = {"t_grid": display_t_grid}
    for label, idx in [("base_pos", 1), ("base_ori_deg", 2), ("joint_pos", 3)]:
        stack = []
        for rec in loaded:
            t_rel = rec[0]
            err = rec[idx]
            stack.append(np.interp(src_t_query, t_rel, err))
        stack = np.array(stack)
        out[f"{label}_mean"] = stack.mean(axis=0)
        out[f"{label}_std"] = stack.std(axis=0)
        out[f"{label}_n"] = len(loaded)
    return out


def first_touchdown_across_trials(trials):
    """Mean display time of first box2 touchdown across all given trials."""
    times = []
    for tdir in trials:
        tick_path = RESULTS / tdir / "tick.csv"
        loaded = load_tick(tick_path)
        if loaded is None:
            continue
        t, opt_x, meas_rbd, planned_mode = loaded
        td = find_first_box2_touchdown_disp(t, planned_mode, meas_rbd)
        if td is not None:
            times.append(td)
    if not times:
        return None
    return float(np.mean(times))


def main():
    aggregated = {}
    for key, trials in TRIALS.items():
        agg = aggregate_condition(trials)
        if agg is None:
            print(f"[warn] no usable trials for {key}", file=sys.stderr)
            continue
        aggregated[key] = agg

    # Layout: 3 rows (metric) × 2 cols (Δz sign).
    # In each subplot: ON,no-splice (blue) and OFF (orange) mean ± std band.
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharex='col')

    metric_meta = [
        ("base_pos",     "Position tracking error squared norm [m²]"),
        ("base_ori_deg", "Orientation tracking error squared norm [deg²]"),
        ("joint_pos",    "Joint position tracking error squared norm [rad²]"),
    ]
    dz_signs = ["Δz=-0.03", "Δz=+0.03"]
    cond_styles = {
        "ON,no-splice": {"color": "tab:blue",   "label": "with robust phase"},
        "OFF":           {"color": "tab:orange", "label": "without robust phase"},
    }
    for col, dz in enumerate(dz_signs):
        for row, (mkey, mlabel) in enumerate(metric_meta):
            ax = axes[row, col]
            for cond, style in cond_styles.items():
                key = (dz, cond)
                if key not in aggregated:
                    continue
                d = aggregated[key]
                t = d["t_grid"]
                mean = d[f"{mkey}_mean"]
                std = d[f"{mkey}_std"]
                n = d[f"{mkey}_n"]
                ax.plot(t, mean, color=style["color"],
                        label=f"{style['label']} (n={n})", linewidth=1.5)
                ax.fill_between(t, mean - std, mean + std,
                                color=style["color"], alpha=0.18, linewidth=0)
            if row == 0:
                ax.set_title(dz, fontsize=12, fontweight='bold')
            if col == 0:
                ax.set_ylabel(mlabel, fontsize=10)
            if row == 2:
                ax.set_xlabel("Time [s]", fontsize=10)
            ax.set_xlim(0, DISPLAY_DURATION)
            ax.grid(True, alpha=0.3)
            if row == 0 and col == 0:
                ax.legend(loc="upper left", fontsize=9)

    fig.suptitle(
        r"comparison of $\mathbf{WBC}$ tracking error",
        fontsize=13, fontweight='bold',
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PNG, dpi=130)
    print(f"saved {OUT_PNG}")

    # Also print summary statistics: time-mean of per-tick error, per condition.
    print()
    print("=== Time-mean WBC tracking error per condition (mean ± std over n trials) ===")
    print(f"{'condition':30s} | {'base_pos [m]':>15s} | {'base_ori [deg]':>16s} | {'joint_pos [rad]':>17s}")
    print("-" * 90)
    for (dz, cond), d in aggregated.items():
        bp_mean = float(np.mean(d["base_pos_mean"]))
        bp_std  = float(np.mean(d["base_pos_std"]))
        bo_mean = float(np.mean(d["base_ori_deg_mean"]))
        bo_std  = float(np.mean(d["base_ori_deg_std"]))
        jp_mean = float(np.mean(d["joint_pos_mean"]))
        jp_std  = float(np.mean(d["joint_pos_std"]))
        print(f"  {dz} {cond:18s} | {bp_mean:6.4f} ± {bp_std:5.4f} | {bo_mean:6.3f} ± {bo_std:5.3f} | {jp_mean:6.4f} ± {jp_std:5.4f}")


if __name__ == "__main__":
    main()

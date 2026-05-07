#!/usr/bin/env python3
"""Plot foot z trajectory vs robust-phase window markers for an M1'' trial.

Run with the wb-mpc conda env (pinocchio + matplotlib):
  /home/cora/miniforge3/envs/wb-mpc/bin/python plot_robust_phase.py <trial_dir>

Inputs (read from <trial_dir>):
  tick.csv          per-tick OCS2 state/input + measured RBD state
  controller.log    contains [robust_phase] log lines emitted by computeRobustWindows

Output:
  <trial_dir>/robust_phase_foot_z.png   one subplot per leg, opt + meas foot-z + ±d markers

OCS2 centroidal-state layout (24 elements in opt_x*):
  [v_com(3), w_c(3), r_b(3), theta_zyx(3), q_j(12)]
  where theta_zyx = [yaw, pitch, roll] (ZYX intrinsic Euler).

Measured RBD layout (36 elements in meas_rbd*):
  [theta_zyx(3), r_b(3), q_j(12), w_b(3), r_b_dot(3), q_j_dot(12)]
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pinocchio as pin

LEG_NAMES = ["FL", "FR", "RL", "RR"]
FRAME_NAMES = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

ROBUST_LINE_RE = re.compile(
    r"\[robust_phase\] t=([-\d.e+]+) leg=(\d+) active=(\d+) ta=([-\d.e+]+) tb=([-\d.e+]+) "
    r"pz=([-\d.e+]+) d=([-\d.e+]+)(?: offset=([-\d.e+]+))? clamped=(\d+)"
)


def euler_zyx_to_quat_xyzw(theta_zyx: np.ndarray) -> np.ndarray:
    yaw, pitch, roll = float(theta_zyx[0]), float(theta_zyx[1]), float(theta_zyx[2])
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    # R = Rz(yaw) Ry(pitch) Rx(roll), quaternion [x,y,z,w]
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = cy * sp * cr + sy * cp * sr
    qz = sy * cp * cr - cy * sp * sr
    return np.array([qx, qy, qz, qw], dtype=float)


def load_tick(tick_csv: Path):
    with open(tick_csv) as f:
        header = f.readline().strip().split(",")
    n_expected = len(header)
    rows = []
    with open(tick_csv) as f:
        next(f)  # skip header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != n_expected:
                continue  # skip partial / malformed lines (SIGKILL truncation)
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    arr = np.array(rows, dtype=float)
    return header, arr


def slice_cols(header, prefix, count):
    cols = [header.index(f"{prefix}{i}") for i in range(count)]
    return cols


def build_pin_model(urdf_path: Path):
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()
    frame_ids = []
    for fname in FRAME_NAMES:
        fid = model.getFrameId(fname)
        if fid >= model.nframes:
            raise RuntimeError(f"frame {fname} not found in URDF")
        frame_ids.append(fid)
    return model, data, frame_ids


def compute_foot_z_trajectory(model, data, frame_ids, q_pin_traj):
    n = q_pin_traj.shape[0]
    foot_z = np.zeros((n, 4))
    for i in range(n):
        pin.forwardKinematics(model, data, q_pin_traj[i])
        pin.updateFramePlacements(model, data)
        for k, fid in enumerate(frame_ids):
            foot_z[i, k] = data.oMf[fid].translation[2]
    return foot_z


def parse_robust_phase_log(log_path: Path):
    """Returns dict: leg_idx -> list of (t_query, active, ta, tb, pz, d, offset, clamped).

    `offset` is the foot_frame_offset (FK foot frame z above contact along n). It may be
    absent in old logs (pre-review-fix); defaults to 0.0 in that case.
    """
    out = {leg: [] for leg in range(4)}
    with open(log_path) as f:
        for line in f:
            m = ROBUST_LINE_RE.search(line)
            if not m:
                continue
            t_q = float(m.group(1))
            leg = int(m.group(2))
            active = int(m.group(3)) == 1
            ta = float(m.group(4))
            tb = float(m.group(5))
            pz = float(m.group(6))
            d = float(m.group(7))
            offset = float(m.group(8)) if m.group(8) is not None else 0.0
            clamped = int(m.group(9)) == 1
            if 0 <= leg < 4:
                out[leg].append((t_q, active, ta, tb, pz, d, offset, clamped))
    return out


def extract_unique_windows(per_leg_log):
    """Collapse repeated (ta, tb, pz, d, offset) windows so we only mark each one once."""
    out = {leg: [] for leg in range(4)}
    for leg, entries in per_leg_log.items():
        seen = set()
        for (_, active, ta, tb, pz, d, offset, clamped) in entries:
            if not active:
                continue
            key = (round(ta, 5), round(tb, 5))
            if key in seen:
                continue
            seen.add(key)
            out[leg].append((ta, tb, pz, d, offset, clamped))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("trial_dir", type=Path)
    ap.add_argument(
        "--urdf",
        type=Path,
        default=Path(
            "/home/cora/GO2_ws/quadruped_ros2_control/descriptions/unitree/go2_description/urdf/robot.urdf"
        ),
    )
    args = ap.parse_args(argv)

    trial_dir = args.trial_dir
    if not trial_dir.is_dir():
        print(f"trial_dir not found: {trial_dir}", file=sys.stderr)
        return 1

    tick_csv = trial_dir / "tick.csv"
    controller_log = trial_dir / "controller.log"
    out_png = trial_dir / "robust_phase_foot_z.png"

    if not tick_csv.exists():
        print(f"missing tick.csv in {trial_dir}", file=sys.stderr)
        return 1
    if not controller_log.exists():
        print(f"missing controller.log in {trial_dir}", file=sys.stderr)
        return 1

    header, tick = load_tick(tick_csv)
    t_col = header.index("t")
    opt_x_cols = slice_cols(header, "opt_x", 24)
    meas_cols = slice_cols(header, "meas_rbd", 36)

    t = tick[:, t_col]
    opt_x = tick[:, opt_x_cols]              # 24 cols: [v_com, w_c, r_b, theta_zyx, q_j]
    meas_rbd = tick[:, meas_cols]            # 36 cols: [theta_zyx, r_b, q_j, w_b, r_b_dot, q_j_dot]

    # Build pin q (free-flyer) trajectories for both opt and measured states.
    # pin q layout: [r_b(3), quat_xyzw(4), q_j(12)] = 19
    n = t.shape[0]
    q_opt = np.zeros((n, 19))
    q_meas = np.zeros((n, 19))
    for i in range(n):
        # OPT: r_b is opt_x[6:9], theta_zyx is opt_x[9:12], q_j is opt_x[12:24]
        q_opt[i, 0:3] = opt_x[i, 6:9]
        q_opt[i, 3:7] = euler_zyx_to_quat_xyzw(opt_x[i, 9:12])
        q_opt[i, 7:19] = opt_x[i, 12:24]
        # MEAS: theta_zyx is meas_rbd[0:3], r_b is meas_rbd[3:6], q_j is meas_rbd[6:18]
        q_meas[i, 0:3] = meas_rbd[i, 3:6]
        q_meas[i, 3:7] = euler_zyx_to_quat_xyzw(meas_rbd[i, 0:3])
        q_meas[i, 7:19] = meas_rbd[i, 6:18]

    print(f"loaded tick.csv: {n} ticks, t in [{t[0]:.3f}, {t[-1]:.3f}]")

    model, data, frame_ids = build_pin_model(args.urdf)
    foot_z_opt = compute_foot_z_trajectory(model, data, frame_ids, q_opt)
    foot_z_meas = compute_foot_z_trajectory(model, data, frame_ids, q_meas)

    per_leg_log = parse_robust_phase_log(controller_log)
    windows = extract_unique_windows(per_leg_log)

    n_active_total = sum(len(w) for w in windows.values())
    print(f"parsed [robust_phase] log: {n_active_total} unique active windows total")

    # Plot — markers are drawn at the FOOT-FRAME z values that the constraint enforces
    # (i.e. p_plane.z + foot_frame_offset ± d), since the FK in pinocchio gives the URDF
    # foot frame, not the contact point.
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for leg in range(4):
        ax = axes[leg]
        ax.plot(t, foot_z_opt[:, leg], color="tab:blue", lw=1.0, label="opt foot frame z")
        ax.plot(t, foot_z_meas[:, leg], color="tab:orange", lw=1.0, alpha=0.7, label="meas foot frame z")
        ax.axhline(0.0, color="k", lw=0.5, alpha=0.3)
        # Window markers (foot-frame z = p_plane.z + offset ± d)
        for (ta, tb, pz, d, offset, clamped) in windows[leg]:
            color = "tab:red" if not clamped else "tab:purple"
            target_a = pz + offset + d
            target_b = pz + offset - d
            if not clamped:
                ax.plot([ta], [target_a], marker="^", color=color, markersize=6,
                        markeredgecolor="k", linestyle="None", zorder=5)
            ax.plot([tb], [target_b], marker="v", color=color, markersize=6,
                    markeredgecolor="k", linestyle="None", zorder=5)
            ax.axvspan(ta, tb, color=color, alpha=0.07)
        ax.set_ylabel(f"{LEG_NAMES[leg]} foot z [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_ylim(-0.10, 0.30)

    axes[-1].set_xlabel("time [s]")
    fig.suptitle(
        f"M1'' robust-phase verification — {trial_dir.name}\n"
        f"red ▲▼ = nominal (t_a, p_plane+offset+d) (t_b, p_plane+offset-d) foot-frame targets; "
        f"purple = partial-clamped (only t_b)"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=110)
    print(f"saved {out_png}")

    # Print quantitative residuals at boundary nodes (foot-frame z basis)
    print("\n=== boundary residuals (closest tick to t_b/t_a, foot-frame z basis) ===")
    print(" leg     time     target (foot-frame z)    foot_z_opt    residual    contact_residual")
    for leg in range(4):
        for (ta, tb, pz, d, offset, clamped) in windows[leg]:
            i = int(np.argmin(np.abs(t - tb)))
            target_b = pz + offset - d
            res_b = foot_z_opt[i, leg] - target_b
            contact_res_b = res_b  # foot-frame and contact-point residuals coincide (same offset)
            print(f"  {leg}    {tb:7.3f}    {target_b:+.4f}                {foot_z_opt[i, leg]:+.4f}     "
                  f"{res_b:+.4f}   {contact_res_b:+.4f}")
            if not clamped:
                i_a = int(np.argmin(np.abs(t - ta)))
                target_a = pz + offset + d
                res_a = foot_z_opt[i_a, leg] - target_a
                print(f"  {leg}    {ta:7.3f}    {target_a:+.4f}                {foot_z_opt[i_a, leg]:+.4f}     "
                      f"{res_a:+.4f}   {res_a:+.4f}  (t_a)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

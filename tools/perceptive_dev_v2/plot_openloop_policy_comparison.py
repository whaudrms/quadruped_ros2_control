#!/usr/bin/env python3
"""Compare matched frozen pre-contact FL policies over one complete swing."""

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from paper_plot_style import apply_figure_font_sizes
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from paper_plot_style import (
    LINE_WIDTH,
    NOMINAL_COLOR,
    PERCEIVED_COLOR,
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    apply_paper_style,
    paper_legend,
    style_paper_axis,
)
from plot_foothold_policy_vs_actual import (
    _json,
    _optimized_fl_z,
    _select_trials,
    _snapshots,
)
from plot_robust_phase import DEFAULT_URDF
from terrain_descent_events import compute_measured_foot_xyz, load_detection_columns


TARGET_TOLERANCE_M = 0.005


def _select_lower_target_snapshot(trial_dir: Path, perceived_surface: float):
    candidates = [
        snapshot
        for snapshot in _snapshots(trial_dir / "foothold_plan_snapshots.csv")
        if abs(snapshot["touchdown_height"] - perceived_surface)
        <= TARGET_TOLERANCE_M
        and snapshot["policy_start_time"] <= snapshot["touchdown_time"]
    ]
    if not candidates:
        raise RuntimeError(
            f"No pre-contact FL policy targets perceived lower surface "
            f"z={perceived_surface:.3f} m in {trial_dir}"
        )
    # Freeze the completed solve closest to lift-off.  This retains as much of
    # the predicted swing as possible without mixing policies from later MPC
    # replans.
    return min(
        candidates,
        key=lambda item: (
            abs(item["solve_time"] - item["liftoff_time"]),
            -item["solve_time"],
        ),
    )


def _load_policy(trial_dir: Path, urdf_path: Path):
    config = _json(trial_dir / "run_config.json")
    terrain = _json(trial_dir / "terrain_descent_events.json")
    geometry = terrain["geometry"]
    true_surface = float(geometry["lower_surface_z_m"])
    perceived_surface = true_surface + float(config["terrain_z_offset"])
    foot_offset = float(
        config["effective_task_parameters"]["robustPhase"]["foot_frame_offset"]
    )
    snapshot = _select_lower_target_snapshot(trial_dir, perceived_surface)
    policy_visible = (
        snapshot["time"] >= snapshot["policy_start_time"] - 1.0e-9
    )

    times, measured, _modes = load_detection_columns(trial_dir / "tick.csv")
    state_index = int(np.argmin(np.abs(times - snapshot["policy_start_time"])))
    measured_state = measured[state_index]
    foot_xyz = compute_measured_foot_xyz(
        measured[state_index : state_index + 1], urdf_path
    )[0]

    touchdown_time = snapshot["touchdown_time"]
    optimized_time = snapshot["time"][policy_visible] - touchdown_time
    optimized_z = _optimized_fl_z(
        snapshot["optimized_state"][policy_visible], urdf_path
    )
    reference_time = snapshot["time"] - touchdown_time
    reference_z = snapshot["z_ref"] + foot_offset
    return {
        "trial": trial_dir.name,
        "snapshot_id": snapshot["snapshot_id"],
        "solve_time": snapshot["solve_time"],
        "policy_start_relative_s": snapshot["policy_start_time"] - touchdown_time,
        "liftoff_relative_s": snapshot["liftoff_time"] - touchdown_time,
        "touchdown_relative_s": 0.0,
        "swing_duration_s": snapshot["touchdown_time"] - snapshot["liftoff_time"],
        "optimized_time": optimized_time,
        "optimized_z": optimized_z,
        "reference_time": reference_time,
        "reference_z": reference_z,
        "true_contact_level": true_surface + foot_offset,
        "perceived_lower_level": perceived_surface + foot_offset,
        "planned_touchdown_level": snapshot["touchdown_height"] + foot_offset,
        "window_active": snapshot["window_active"],
        "window_ta": snapshot["window_ta"] - touchdown_time,
        "window_tb": snapshot["window_tb"] - touchdown_time,
        "window_lower": snapshot["plane_z"] + foot_offset - snapshot["d"],
        "window_upper": snapshot["plane_z"] + foot_offset + snapshot["d"],
        "measured_state_at_policy_start": measured_state,
        "foot_xyz_at_policy_start": foot_xyz,
    }


def _state_deltas(nominal: dict, proposed: dict):
    nominal_state = nominal["measured_state_at_policy_start"]
    proposed_state = proposed["measured_state_at_policy_start"]
    return {
        "base_position_delta_m": float(
            np.linalg.norm(nominal_state[3:6] - proposed_state[3:6])
        ),
        "base_orientation_delta_rad": float(
            np.linalg.norm(nominal_state[0:3] - proposed_state[0:3])
        ),
        "joint_position_rms_delta_rad": float(
            np.sqrt(np.mean((nominal_state[6:18] - proposed_state[6:18]) ** 2))
        ),
        "max_foot_position_delta_m": float(
            np.max(
                np.linalg.norm(
                    nominal["foot_xyz_at_policy_start"]
                    - proposed["foot_xyz_at_policy_start"],
                    axis=1,
                )
            )
        ),
        "fl_foot_position_delta_m": float(
            np.linalg.norm(
                nominal["foot_xyz_at_policy_start"][0]
                - proposed["foot_xyz_at_policy_start"][0]
            )
        ),
        "swing_duration_delta_s": float(
            abs(nominal["swing_duration_s"] - proposed["swing_duration_s"])
        ),
        "planned_touchdown_level_delta_m": float(
            abs(
                nominal["planned_touchdown_level"]
                - proposed["planned_touchdown_level"]
            )
        ),
    }


def _validate_match(deltas: dict, args):
    failures = []
    limits = {
        "base_position_delta_m": args.max_base_position_delta_m,
        "base_orientation_delta_rad": args.max_base_orientation_delta_rad,
        "joint_position_rms_delta_rad": args.max_joint_position_rms_delta_rad,
        "max_foot_position_delta_m": args.max_foot_position_delta_m,
        "swing_duration_delta_s": args.max_swing_duration_delta_s,
        "planned_touchdown_level_delta_m": args.max_target_delta_m,
    }
    for key, limit in limits.items():
        if deltas[key] > limit:
            failures.append(f"{key}={deltas[key]:.6f} > {limit:.6f}")
    if failures and not args.allow_state_mismatch:
        raise RuntimeError(
            "Frozen-policy pair is not state-matched; refusing a misleading "
            "open-loop comparison:\n  " + "\n  ".join(failures)
        )
    return failures


def _first_true_terrain_intersection(time: np.ndarray, z: np.ndarray, level: float):
    if time.size < 2:
        return None
    apex = int(np.argmax(z))
    for index in range(max(1, apex + 1), z.size):
        if z[index] <= level < z[index - 1]:
            dz = z[index] - z[index - 1]
            alpha = 0.0 if abs(dz) < 1.0e-12 else (level - z[index - 1]) / dz
            crossing_time = time[index - 1] + alpha * (time[index] - time[index - 1])
            return float(crossing_time), float(level)
    return None


def plot(args):
    apply_paper_style()
    trials = _select_trials(args.results_dir)
    data = {
        mode: _load_policy(trial_dir, args.urdf)
        for mode, trial_dir in trials.items()
    }
    deltas = _state_deltas(data["nominal"], data["proposed"])
    match_failures = _validate_match(deltas, args)

    colors = {"nominal": NOMINAL_COLOR, "proposed": PROPOSED_COLOR}
    all_time = np.concatenate([
        np.concatenate((item["optimized_time"], item["reference_time"]))
        for item in data.values()
    ])
    all_z = np.concatenate([
        np.concatenate(
            (
                item["optimized_z"], item["reference_z"],
                [item["true_contact_level"], item["perceived_lower_level"]],
            )
        )
        for item in data.values()
    ])
    x_pad = max(0.015, 0.05 * float(np.ptp(all_time)))
    y_pad = max(0.012, 0.06 * float(np.ptp(all_z)))

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.0), sharex=True, sharey=True)
    if match_failures:
        fig.suptitle(
            "UNMATCHED INITIAL STATES — DIAGNOSTIC ONLY",
            color="#C62828", fontsize=14, fontweight="bold", y=0.985,
        )
    intersections = {}
    for axis, mode, panel in zip(
        axes, ("nominal", "proposed"), ("(a) Baseline", "(b) Proposed")
    ):
        item = data[mode]
        if mode == "proposed" and item["window_active"]:
            axis.fill_between(
                [item["window_ta"], item["window_tb"]],
                item["window_lower"], item["window_upper"],
                color=PROPOSED_COLOR, alpha=0.13, linewidth=0, zorder=0,
            )
        axis.axhline(
            item["true_contact_level"], color=REFERENCE_COLOR,
            linestyle=":", linewidth=1.7, zorder=1,
        )
        axis.axhline(
            item["perceived_lower_level"], color=PERCEIVED_COLOR,
            linestyle="-.", linewidth=1.5, zorder=1,
        )
        axis.axvline(
            0.0, color=REFERENCE_COLOR, linestyle=(0, (5, 2, 1, 2)),
            linewidth=1.1, zorder=1,
        )
        axis.plot(
            item["reference_time"], item["reference_z"],
            color=PERCEIVED_COLOR, linestyle="--", linewidth=1.6, zorder=2,
        )
        axis.plot(
            item["optimized_time"], item["optimized_z"], color=colors[mode],
            linestyle="--", linewidth=LINE_WIDTH, zorder=4,
        )
        intersection = _first_true_terrain_intersection(
            item["optimized_time"], item["optimized_z"],
            item["true_contact_level"],
        )
        intersections[mode] = intersection
        if intersection is not None:
            axis.scatter(
                [intersection[0]], [intersection[1]], marker="o", s=72,
                facecolor="white", edgecolor=colors[mode], linewidth=1.5,
                zorder=7,
            )
            axis.annotate(
                f"Terrain intersection\n"
                f"$\\Delta t$={intersection[0] * 1000:+.0f} ms",
                xy=intersection, xytext=(-8, 18), textcoords="offset points",
                fontsize=10.5, color=colors[mode], ha="right", va="bottom",
                arrowprops={"arrowstyle": "-", "color": colors[mode], "lw": 0.9},
            )
        axis.text(
            0.025, 0.95, panel, transform=axis.transAxes,
            ha="left", va="top", fontsize=15,
        )
        axis.set_xlabel("Time relative to scheduled touchdown [s]")
        axis.set_xlim(float(np.min(all_time) - x_pad), float(np.max(all_time) + x_pad))
        axis.set_ylim(float(np.min(all_z) - y_pad), float(np.max(all_z) + y_pad))
        style_paper_axis(axis, minor=True)
    axes[0].set_ylabel("World-frame FL foot z [m]")

    handles = [
        Line2D([0], [0], color=REFERENCE_COLOR, linestyle="--", linewidth=LINE_WIDTH,
               label="Frozen optimized policy"),
        Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="--", linewidth=1.6,
               label="Full swing reference"),
        Line2D([0], [0], color=REFERENCE_COLOR, linestyle=":", linewidth=1.7,
               label="True lower contact level"),
        Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="-.", linewidth=1.5,
               label="Perceived lower level"),
        Line2D([0], [0], marker="o", linestyle="none", markerfacecolor="white",
               markeredgecolor=REFERENCE_COLOR, markersize=7,
               label="Predicted true-terrain intersection"),
        Patch(facecolor=PROPOSED_COLOR, alpha=0.13, edgecolor="none",
              label="Robust window"),
    ]
    paper_legend(
        axes[0], handles=handles, loc="upper center", bbox_to_anchor=(1.02, -0.18),
        ncol=3,
    )
    top = 0.92 if match_failures else 0.97
    apply_figure_font_sizes(fig, args.output)
    fig.subplots_adjust(left=0.09, right=0.985, top=top, bottom=0.25, wspace=0.10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=300)
    fig.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)

    report = {
        "comparison": "independent frozen pre-contact MPC policies",
        "strict_state_match": not args.allow_state_mismatch,
        "state_match_failures": match_failures,
        "state_deltas": deltas,
        "methods": {},
    }
    for mode, item in data.items():
        report["methods"][mode] = {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in item.items()
            if key not in {
                "optimized_time", "optimized_z", "reference_time", "reference_z",
                "measured_state_at_policy_start", "foot_xyz_at_policy_start",
            }
        }
        report["methods"][mode]["predicted_true_terrain_intersection"] = (
            list(intersections[mode]) if intersections[mode] is not None else None
        )
    args.output.with_suffix(".json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--allow-state-mismatch", action="store_true")
    parser.add_argument("--max-base-position-delta-m", type=float, default=0.03)
    parser.add_argument("--max-base-orientation-delta-rad", type=float, default=0.10)
    parser.add_argument("--max-joint-position-rms-delta-rad", type=float, default=0.15)
    parser.add_argument("--max-foot-position-delta-m", type=float, default=0.04)
    parser.add_argument("--max-swing-duration-delta-s", type=float, default=0.02)
    parser.add_argument("--max-target-delta-m", type=float, default=0.005)
    args = parser.parse_args()
    args.results_dir = args.results_dir.resolve()
    args.urdf = args.urdf.resolve()
    args.output = (
        args.output.resolve()
        if args.output
        else args.results_dir / "all_visualizations" / "fig_openloop_policy_comparison.png"
    )
    plot(args)
    print(args.output)


if __name__ == "__main__":
    main()

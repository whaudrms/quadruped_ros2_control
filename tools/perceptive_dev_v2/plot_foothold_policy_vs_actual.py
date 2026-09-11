#!/usr/bin/env python3
"""Compare the last pre-contact MPC FL policy with the measured touchdown."""

import argparse
import csv
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
    NOMINAL_MARKER,
    PERCEIVED_COLOR,
    PROPOSED_COLOR,
    PROPOSED_MARKER,
    REFERENCE_COLOR,
    apply_paper_style,
    paper_legend,
    style_paper_axis,
)
from plot_fl_foothold_extremes import _foot_z_for_trial
from plot_robust_phase import (
    DEFAULT_URDF,
    build_pin_model,
    compute_foot_z_trajectory,
    euler_zyx_to_quat_xyzw,
)


FULL_TIME_LIMITS_S = (-2.0, 1.75)


def _json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _mode(trial_dir: Path):
    config = _json(trial_dir / "run_config.json")
    enabled = config["effective_task_parameters"]["robustPhase"]["enabled"]
    return "proposed" if str(enabled).lower() == "true" else "nominal"


def _select_trials(results_dir: Path):
    selected = {}
    for trial_dir in sorted(results_dir.iterdir()):
        if not trial_dir.is_dir() or not (trial_dir / "run_config.json").is_file():
            continue
        mode = _mode(trial_dir)
        if mode in selected:
            raise RuntimeError(f"Expected one {mode} trial in {results_dir}")
        selected[mode] = trial_dir
    if set(selected) != {"nominal", "proposed"}:
        raise RuntimeError(f"Expected one nominal and one proposed trial in {results_dir}")
    return selected


def _snapshots(path: Path):
    if not path.is_file():
        raise FileNotFoundError(
            f"{path} is missing; rerun exp_foothold.sh with the rebuilt controller"
        )
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    state_fields = [f"opt_x{index}" for index in range(24)]
    required = {"z_ref", "snapshot_id", "solve_time", *state_fields}
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError(f"Invalid foothold snapshot schema: {path}")

    grouped = {}
    for row in rows:
        grouped.setdefault(int(row["snapshot_id"]), []).append(row)
    parsed = []
    for snapshot_id, group in grouped.items():
        group.sort(key=lambda row: int(row["sample_index"]))
        first = group[0]
        parsed.append(
            {
                "snapshot_id": snapshot_id,
                "solve_time": float(first["solve_time"]),
                "policy_start_time": float(first["policy_start_time"]),
                "liftoff_time": float(first["liftoff_time"]),
                "touchdown_time": float(first["touchdown_time"]),
                "touchdown_height": float(first["touchdown_height"]),
                "window_active": bool(int(first["window_active"])),
                "window_ta": float(first["window_ta"]),
                "window_tb": float(first["window_tb"]),
                "plane_z": float(first["plane_z"]),
                "d": float(first["d"]),
                "foot_frame_offset": float(first["foot_frame_offset"]),
                "time": np.asarray([float(row["sample_time"]) for row in group]),
                "z_ref": np.asarray([float(row["z_ref"]) for row in group]),
                "optimized_state": np.asarray(
                    [
                        [float(row[field]) for field in state_fields]
                        for row in group
                    ]
                ),
            }
        )
    return sorted(parsed, key=lambda item: item["solve_time"])


def _optimized_fl_z(optimized_state: np.ndarray, urdf_path: Path):
    """Evaluate FL FK offline so diagnostics cannot affect the controller."""
    sample_count = optimized_state.shape[0]
    q_trajectory = np.zeros((sample_count, 19))
    q_trajectory[:, 0:3] = optimized_state[:, 6:9]
    for index in range(sample_count):
        q_trajectory[index, 3:7] = euler_zyx_to_quat_xyzw(
            optimized_state[index, 9:12]
        )
    q_trajectory[:, 7:19] = optimized_state[:, 12:24]
    model, data, frame_ids = build_pin_model(urdf_path)
    return compute_foot_z_trajectory(model, data, frame_ids, q_trajectory)[:, 0]


def _online_swing_reference(snapshots, query_time, foot_frame_offset):
    """Sample the latest completed MPC solve's active FL swing reference."""
    reference = np.full(query_time.shape, np.nan, dtype=float)
    for index, sample_time in enumerate(query_time):
        candidates = []
        for snapshot in snapshots:
            # The first logged z_ref sample is a zero-valued sentinel rather
            # than a physical world-frame height.  Do not interpolate through
            # it, or the displayed reference acquires a false vertical drop.
            valid = np.isfinite(snapshot["z_ref"]) & (
                np.abs(snapshot["z_ref"]) > 1.0e-12
            )
            if not np.any(valid):
                continue
            valid_time = snapshot["time"][valid]
            if (
                snapshot["solve_time"] <= sample_time + 1.0e-9
                and valid_time[0] - 1.0e-9 <= sample_time
                and sample_time <= valid_time[-1] + 1.0e-9
            ):
                candidates.append((snapshot, valid))
        if not candidates:
            continue
        snapshot, valid = max(
            candidates, key=lambda item: item[0]["solve_time"]
        )
        reference[index] = np.interp(
            sample_time, snapshot["time"][valid], snapshot["z_ref"][valid]
        ) + foot_frame_offset
    return reference


def _select_contact_and_snapshot(trial_dir: Path):
    terrain = _json(trial_dir / "terrain_descent_events.json")
    edge_time = float(terrain["feet"]["FL"]["edge_crossing"]["raw_time_sec"])
    z_limit = float(terrain["geometry"].get("lower_touchdown_z_max_m", 0.21))
    runtime = _json(trial_dir / "runtime_metrics.json")
    contacts = sorted(
        (
            event for event in runtime.get("touchdown_events", [])
            if event.get("leg") == "FL"
            and float(event["time_s"]) >= edge_time
            and float(event["foot_xyz_m"][0]) >= 0.55
            and float(event["foot_xyz_m"][2]) <= z_limit
        ),
        key=lambda event: float(event["time_s"]),
    )
    snapshots = _snapshots(trial_dir / "foothold_plan_snapshots.csv")

    # Anchor both panels to the first physical FL contact on the lower step.
    # Selecting by the snapshot's touchdown height can skip that contact when
    # the planner still targets the upper plane, and then incorrectly attach
    # the plot to a later swing that lifts off from the lower step.
    for contact in contacts:
        contact_time = float(contact["time_s"])
        candidates = [
            snapshot for snapshot in snapshots
            if snapshot["solve_time"] < contact_time
            # A touchdown cannot belong to a swing that has not lifted off.
            # The previous 50-ms tolerance incorrectly attached stance contact
            # chatter to the following swing in the proposed trial.
            and snapshot["liftoff_time"] <= contact_time
            and contact_time <= snapshot["touchdown_time"] + 0.20
        ]
        if candidates:
            # Use the policy solve closest to liftoff.  This retains almost the
            # whole swing while using the same lower-plane target in both runs.
            snapshot = min(
                candidates,
                key=lambda item: (
                    abs(item["solve_time"] - item["liftoff_time"]),
                    -item["solve_time"],
                ),
            )
            return contact, snapshot
    raise RuntimeError(
        f"No first lower-step FL contact with a matching pre-contact MPC snapshot in {trial_dir}"
    )


def _load_trial(trial_dir: Path, urdf_path: Path):
    config = _json(trial_dir / "run_config.json")
    result_path = trial_dir / "result.json"
    result = _json(result_path) if result_path.is_file() else {}
    terrain = _json(trial_dir / "terrain_descent_events.json")
    geometry = terrain["geometry"]
    true_surface = float(geometry["lower_surface_z_m"])
    perceived_surface = true_surface + float(config["terrain_z_offset"])
    guard_offset = float(
        config["effective_task_parameters"]["robustPhase"]["foot_frame_offset"]
    )
    contact, snapshot = _select_contact_and_snapshot(trial_dir)
    contact_time = float(contact["time_s"])
    touchdown_time = snapshot["touchdown_time"]
    planned_visible = snapshot["time"] >= snapshot["policy_start_time"] - 1.0e-9

    trajectory = _foot_z_for_trial(trial_dir, urdf_path)
    measured_time = np.asarray(trajectory["time"])
    measured_z = np.asarray(trajectory["measured_z"])
    snapshots = _snapshots(trial_dir / "foothold_plan_snapshots.csv")
    swing_reference_full = _online_swing_reference(
        snapshots, measured_time, guard_offset
    )
    visible = (
        (measured_time >= snapshot["liftoff_time"] - 0.03)
        & (measured_time <= max(contact_time, touchdown_time) + 0.05)
    )
    return {
        "trial": trial_dir.name,
        "snapshot_id": snapshot["snapshot_id"],
        "solve_time": snapshot["solve_time"],
        "planned_time": snapshot["time"][planned_visible] - touchdown_time,
        "optimized_z": _optimized_fl_z(
            snapshot["optimized_state"][planned_visible], urdf_path
        ),
        # SwingTrajectoryPlanner stores the contact-point reference at the
        # terrain surface, whereas FK and the measured trajectory are expressed
        # at the foot frame.  Put every plotted height in the foot-frame
        # convention before comparing plan and execution.
        "z_ref": snapshot["z_ref"][planned_visible] + guard_offset,
        "measured_time": measured_time[visible] - touchdown_time,
        "measured_z": measured_z[visible],
        "measured_time_full": measured_time - touchdown_time,
        "measured_z_full": measured_z,
        "swing_reference_time_full": measured_time - touchdown_time,
        "swing_reference_z_full": swing_reference_full,
        "actual_contact_time": contact_time - touchdown_time,
        "actual_contact_z": float(contact["foot_xyz_m"][2]),
        "actual_contact_x": float(contact["foot_xyz_m"][0]),
        "touchdown_normal_speed": (
            float(contact["touchdown_normal_speed_mps"])
            if contact.get("touchdown_normal_speed_mps") is not None else None
        ),
        "success": result.get("success"),
        "true_contact_level": true_surface + guard_offset,
        "perceived_lower_level": perceived_surface + guard_offset,
        "planned_touchdown_level": snapshot["touchdown_height"] + guard_offset,
        "terrain_offset": float(config["terrain_z_offset"]),
        "foot_frame_offset": guard_offset,
        "window_active": snapshot["window_active"],
        "window_ta": snapshot["window_ta"] - touchdown_time,
        "window_tb": snapshot["window_tb"] - touchdown_time,
        "window_lower": snapshot["plane_z"] + guard_offset - snapshot["d"],
        "window_upper": snapshot["plane_z"] + guard_offset + snapshot["d"],
    }


def plot(results_dir: Path, output: Path, urdf_path: Path, time_view: str = "swing"):
    if time_view not in {"swing", "full"}:
        raise ValueError(f"Unsupported time view: {time_view}")
    apply_paper_style()
    trials = _select_trials(results_dir)
    data = {mode: _load_trial(path, urdf_path) for mode, path in trials.items()}
    colors = {"nominal": NOMINAL_COLOR, "proposed": PROPOSED_COLOR}
    markers = {"nominal": NOMINAL_MARKER, "proposed": PROPOSED_MARKER}

    measured_time_key = "measured_time_full" if time_view == "full" else "measured_time"
    measured_z_key = "measured_z_full" if time_view == "full" else "measured_z"
    if time_view == "full":
        x_min, x_max = FULL_TIME_LIMITS_S
        z_parts = []
        for item in data.values():
            visible = (
                (item[measured_time_key] >= x_min)
                & (item[measured_time_key] <= x_max)
            )
            z_parts.append(item[measured_z_key][visible])
            reference_visible = item["swing_reference_z_full"][visible]
            z_parts.append(reference_visible[np.isfinite(reference_visible)])
            z_parts.append(
                np.asarray(
                    [item["true_contact_level"], item["perceived_lower_level"]]
                )
            )
        z_values = np.concatenate(z_parts)
    else:
        x_values = np.concatenate([
            np.concatenate((item["planned_time"], item[measured_time_key]))
            for item in data.values()
        ])
        z_values = np.concatenate(
            [
                np.concatenate(
                    (
                        item["optimized_z"], item["z_ref"], item[measured_z_key],
                        [item["true_contact_level"], item["perceived_lower_level"]],
                    )
                )
                for item in data.values()
            ]
        )
        x_pad = max(0.015, 0.04 * float(np.ptp(x_values)))
        x_min = float(np.min(x_values) - x_pad)
        x_max = float(np.max(x_values) + x_pad)
    y_pad = max(0.012, 0.06 * float(np.ptp(z_values)))

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2), sharex=True, sharey=True)
    for axis, mode, panel in zip(axes, ("nominal", "proposed"), ("(a) Baseline", "(b) Proposed")):
        item = data[mode]
        if time_view == "swing" and mode == "proposed" and item["window_active"]:
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
        if time_view == "full":
            reference_visible = (
                (item["swing_reference_time_full"] >= x_min)
                & (item["swing_reference_time_full"] <= x_max)
            )
            axis.plot(
                item["swing_reference_time_full"][reference_visible],
                item["swing_reference_z_full"][reference_visible],
                color=colors[mode], linestyle="--", linewidth=1.8, zorder=4,
            )
        else:
            axis.plot(
                item["planned_time"], item["z_ref"], color=PERCEIVED_COLOR,
                linestyle="--", linewidth=1.6, zorder=2,
            )
            axis.plot(
                item["planned_time"], item["optimized_z"], color=colors[mode],
                linestyle="--", linewidth=LINE_WIDTH, zorder=4,
            )
        axis.plot(
            item[measured_time_key], item[measured_z_key], color=colors[mode],
            linestyle="-", linewidth=LINE_WIDTH, zorder=5,
        )
        if time_view == "swing":
            outcome = (
                "Success" if item["success"] is True
                else "Failure" if item["success"] is False
                else "Outcome n/a"
            )
            speed = item["touchdown_normal_speed"]
            speed_text = "n/a" if speed is None else f"{speed:.3f} m/s"
            outcome_color = "#2E7D32" if item["success"] is True else "#C62828"
            annotation_offset = (8, 22) if mode == "nominal" else (-8, 22)
            annotation_alignment = "left" if mode == "nominal" else "right"
            axis.scatter(
                [item["actual_contact_time"]], [item["actual_contact_z"]],
                marker=markers[mode], s=80, facecolor=colors[mode],
                edgecolor=colors[mode], linewidth=1.4, zorder=8,
            )
            axis.annotate(
                f"Actual contact\n"
                f"$\\Delta t$={item['actual_contact_time'] * 1000:+.0f} ms\n"
                f"$v_n$={speed_text}  |  {outcome}",
                xy=(item["actual_contact_time"], item["actual_contact_z"]),
                xytext=annotation_offset, textcoords="offset points",
                ha=annotation_alignment, va="bottom",
                fontsize=10.5, color=outcome_color,
                arrowprops={"arrowstyle": "-", "color": outcome_color, "lw": 1.0},
                bbox={
                    "boxstyle": "square,pad=0.22", "facecolor": "white",
                    "edgecolor": outcome_color, "linewidth": 0.8, "alpha": 0.94,
                },
            )
        axis.text(0.025, 0.95, panel, transform=axis.transAxes, ha="left", va="top", fontsize=15)
        axis.set_xlabel("Time relative to scheduled touchdown [s]")
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(float(np.min(z_values) - y_pad), float(np.max(z_values) + y_pad))
        style_paper_axis(axis, minor=True)
    axes[0].set_ylabel("World-frame FL foot z [m]")

    if time_view == "full":
        handles = [
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle="-",
                   linewidth=LINE_WIDTH, label="Measured foot"),
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle="--",
                   linewidth=1.8, label="Online swing reference"),
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle=":",
                   linewidth=1.7, label="True foothold level"),
            Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="-.",
                   linewidth=1.5, label="Perceived foothold level"),
        ]
        legend = fig.legend(
            handles=handles, loc="upper center", ncol=4,
            bbox_to_anchor=(0.5, 0.995), fontsize=11,
            frameon=True, fancybox=False, framealpha=1.0,
            edgecolor="0.35", facecolor="white",
        )
        legend.get_frame().set_linewidth(0.8)
        apply_figure_font_sizes(fig, output)
        fig.tight_layout(rect=(0, 0, 1, 0.86), pad=0.45, w_pad=1.0)
    else:
        handles = [
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle="-", linewidth=LINE_WIDTH,
                   label="Measured execution"),
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle="--", linewidth=LINE_WIDTH,
                   label="Pre-contact optimized plan"),
            Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="--", linewidth=1.6,
                   label="Swing reference"),
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle=":", linewidth=1.7,
                   label="True contact level"),
            Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="-.", linewidth=1.5,
                   label="Perceived lower level"),
            Line2D([0], [0], color=REFERENCE_COLOR, linestyle=(0, (5, 2, 1, 2)),
                   linewidth=1.1, label="Scheduled touchdown ($t=0$)"),
            Patch(facecolor=PROPOSED_COLOR, alpha=0.13, edgecolor="none", label="Robust window"),
        ]
        paper_legend(
            axes[0], handles=handles, loc="upper center",
            bbox_to_anchor=(1.02, -0.18), ncol=4,
        )
        apply_figure_font_sizes(fig, output)
        fig.subplots_adjust(
            left=0.09, right=0.985, top=0.97, bottom=0.25, wspace=0.10
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)

    summary = {
        mode: {
            key: item[key]
            for key in (
                "trial", "snapshot_id", "solve_time", "actual_contact_time",
                "actual_contact_z", "actual_contact_x", "true_contact_level",
                "touchdown_normal_speed", "success", "terrain_offset",
                "foot_frame_offset", "perceived_lower_level",
                "planned_touchdown_level", "window_active", "window_ta",
                "window_tb", "window_lower", "window_upper",
            )
        }
        for mode, item in data.items()
    }
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--time-view", choices=("swing", "full"), default="swing",
        help=(
            "Plot the selected frozen swing only or the wider online-reference "
            "comparison through the pre-peak interval."
        ),
    )
    args = parser.parse_args()
    default_name = (
        "fig_foothold_policy_vs_actual_full.png"
        if args.time_view == "full"
        else "fig_foothold_policy_vs_actual.png"
    )
    output = args.output or args.results_dir / "all_visualizations" / default_name
    plot(
        args.results_dir.resolve(), output.resolve(), args.urdf.resolve(),
        time_view=args.time_view,
    )
    print(output.resolve())


if __name__ == "__main__":
    main()

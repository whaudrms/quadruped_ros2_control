#!/usr/bin/env python3
"""Plot FL foot-z trajectories for selected terrain-height-error trials."""

import argparse
import json
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from paper_plot_style import (
    LEGEND_SIZE,
    NOMINAL_COLOR,
    PERCEIVED_COLOR,
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    SAVE_DPI,
    apply_paper_style,
    paper_figsize,
    paper_legend,
    style_paper_axis,
)
from plot_robust_phase import (
    DEFAULT_URDF,
    build_pin_model,
    compute_foot_z_trajectory,
    euler_zyx_to_quat_xyzw,
    extract_unique_windows,
    load_tick,
    parse_robust_phase_log,
    slice_cols,
)
from terrain_descent_events import load_or_detect_events


TARGET_OFFSETS = (-0.05, 0.05)
OFFSET_TOLERANCE = 1e-9
OPT_COLOR = PROPOSED_COLOR
MEASURED_COLOR = NOMINAL_COLOR


apply_paper_style()


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _trial_metadata(trial_dir: Path):
    result = _load_json(trial_dir / "result.json")
    config = _load_json(trial_dir / "run_config.json")
    experiment = result.get("experiment", {})
    robust_parameters = experiment.get("effective_task_parameters", {}).get(
        "robustPhase", {}
    )
    if not robust_parameters:
        robust_parameters = config.get("effective_task_parameters", {}).get(
            "robustPhase", {}
        )
    offset = experiment.get("terrain_z_offset", config.get("terrain_z_offset"))
    enabled = str(robust_parameters.get("enabled", "false")).lower() == "true"
    run_match = re.search(r"_run(\d+)(?:_|$)", trial_dir.name)
    return {
        "trial_dir": trial_dir,
        "success": bool(result.get("success")),
        "robust_on": enabled,
        "offset_m": float(offset) if offset is not None else None,
        "run": int(run_match.group(1)) if run_match else None,
    }


def select_trials(
    results_dir: Path,
    *,
    robust_on: bool,
    allow_nearest: bool,
    require_success: bool = True,
):
    candidates = [_trial_metadata(path) for path in results_dir.iterdir() if path.is_dir()]
    selected = {}
    for target in TARGET_OFFSETS:
        eligible = [
            item for item in candidates
            if (item["success"] or not require_success)
            and item["robust_on"] == robust_on
            and item["offset_m"] is not None
            and (item["trial_dir"] / "tick.csv").is_file()
            and (item["trial_dir"] / "controller.log").is_file()
        ]
        matching = [
            item for item in eligible
            if abs(item["offset_m"] - target) <= OFFSET_TOLERANCE
        ]
        if not matching and allow_nearest and eligible:
            closest_distance = min(abs(item["offset_m"] - target) for item in eligible)
            matching = [
                item for item in eligible
                if abs(abs(item["offset_m"] - target) - closest_distance) <= OFFSET_TOLERANCE
            ]
        if not matching:
            raise RuntimeError(
                f"No {'successful ' if require_success else ''}"
                f"Robust-{'ON' if robust_on else 'OFF'} trial with "
                f"terrain_z_offset={target:+.2f} "
                f"was found in {results_dir}"
            )
        selected[target] = min(
            matching,
            key=lambda item: (
                item["run"] if item["run"] is not None else 10**9,
                item["trial_dir"].name,
            ),
        )
    return selected


def _foot_z_for_trial(trial_dir: Path, urdf_path: Path):
    header, tick = load_tick(trial_dir / "tick.csv")
    if tick.size == 0:
        raise RuntimeError(f"No valid tick rows in {trial_dir / 'tick.csv'}")
    time = tick[:, header.index("t")]
    opt_x = tick[:, slice_cols(header, "opt_x", 24)]
    measured = tick[:, slice_cols(header, "meas_rbd", 36)]
    modes = tick[:, header.index("planned_mode")].astype(np.int64)

    q_opt = np.zeros((len(time), 19))
    q_measured = np.zeros((len(time), 19))
    for index in range(len(time)):
        q_opt[index, 0:3] = opt_x[index, 6:9]
        q_opt[index, 3:7] = euler_zyx_to_quat_xyzw(opt_x[index, 9:12])
        q_opt[index, 7:19] = opt_x[index, 12:24]
        q_measured[index, 0:3] = measured[index, 3:6]
        q_measured[index, 3:7] = euler_zyx_to_quat_xyzw(measured[index, 0:3])
        q_measured[index, 7:19] = measured[index, 6:18]

    model, data, frame_ids = build_pin_model(urdf_path)
    opt_z = compute_foot_z_trajectory(model, data, frame_ids, q_opt)[:, 0]
    measured_z = compute_foot_z_trajectory(model, data, frame_ids, q_measured)[:, 0]

    events = load_or_detect_events(trial_dir)
    fl_events = events.get("feet", {}).get("FL", {})
    touchdown = fl_events.get("lower_touchdown")
    if not touchdown:
        raise RuntimeError(f"No FL lower touchdown detected in {trial_dir}")
    touchdown_time = float(touchdown["raw_time_sec"])
    touchdown_index = int(np.argmin(np.abs(time - touchdown_time)))

    fl_contact = ((modes >> 3) & 1).astype(bool)
    liftoff_indices = np.flatnonzero(fl_contact[:-1] & ~fl_contact[1:]) + 1
    prior_liftoffs = liftoff_indices[liftoff_indices < touchdown_index]
    swing_start_time = (
        float(time[prior_liftoffs[-1]]) if prior_liftoffs.size
        else touchdown_time - 0.8
    )

    windows = extract_unique_windows(parse_robust_phase_log(trial_dir / "controller.log"))[0]
    geometry = events.get("geometry", {})
    true_surface = float(geometry.get("lower_surface_z_m", 0.1))
    foot_frame_offset = float(geometry.get("foot_frame_offset_m", 0.06))
    return {
        "time": time,
        "time_relative": time - touchdown_time,
        "opt_z": opt_z,
        "measured_z": measured_z,
        "touchdown_time": touchdown_time,
        "touchdown_relative": 0.0,
        "touchdown_measured_z": float(measured_z[touchdown_index]),
        "swing_start_relative": swing_start_time - touchdown_time,
        "windows": windows,
        "true_foot_frame_z": true_surface + foot_frame_offset,
        "foot_frame_offset": foot_frame_offset,
    }


def plot(
    selected,
    output: Path,
    urdf_path: Path,
    *,
    robust_on: bool,
    outcome_unfiltered: bool = False,
    y_limits=None,
):
    datasets = {
        offset: _foot_z_for_trial(item["trial_dir"], urdf_path)
        for offset, item in selected.items()
    }
    x_limits = (-1.15, 0.65)
    visible_values = []
    for data in datasets.values():
        visible = (
            (data["time_relative"] >= x_limits[0])
            & (data["time_relative"] <= x_limits[1])
        )
        visible_values.extend(data["opt_z"][visible])
        visible_values.extend(data["measured_z"][visible])
    if y_limits is None:
        y_min = min(visible_values) - 0.025
        y_max = max(visible_values) + 0.035
    else:
        y_min, y_max = map(float, y_limits)
        if y_min >= y_max:
            raise ValueError("--y-limits requires MIN < MAX")

    fig, axes = plt.subplots(1, 2, figsize=paper_figsize(2, 1), sharex=True, sharey=True)
    selection_report = {}
    for axis, offset in zip(axes, TARGET_OFFSETS):
        item = selected[offset]
        data = datasets[offset]
        actual_offset = float(item["offset_m"])
        relative = data["time_relative"]
        visible = (relative >= x_limits[0]) & (relative <= x_limits[1])
        perceived_foot_frame_z = (
            data["true_foot_frame_z"] + actual_offset
        )

        axis.axvspan(
            max(x_limits[0], data["swing_start_relative"]), 0.0,
            color="#A8DADC", alpha=0.20, label="FL swing phase",
        )
        axis.axhline(
            data["true_foot_frame_z"], color=REFERENCE_COLOR, linestyle=":", linewidth=2.25,
            label="True surface + foot offset",
        )
        axis.axhline(
            perceived_foot_frame_z, color=PERCEIVED_COLOR, linestyle="--", linewidth=2.25,
            label="Perceived foothold",
        )
        axis.plot(
            relative[visible], data["opt_z"][visible], color=OPT_COLOR,
            linewidth=2.25, label="Optimized foot z",
        )
        axis.plot(
            relative[visible], data["measured_z"][visible], color=MEASURED_COLOR,
            linewidth=2.25, alpha=0.90, label="Measured foot z",
        )
        axis.scatter(
            [0.0], [data["touchdown_measured_z"]], color=MEASURED_COLOR,
            edgecolor="white", linewidth=0.7, s=52, zorder=6,
            label="Touchdown (measured z)",
        )

        if robust_on:
            marker_labels_added = False
            for ta, tb, pz, d, frame_offset, clamped in data["windows"]:
                ta_relative = ta - data["touchdown_time"]
                tb_relative = tb - data["touchdown_time"]
                if x_limits[0] <= ta_relative <= x_limits[1] and not clamped:
                    axis.scatter(
                        [ta_relative], [pz + frame_offset + d], marker="^", s=48,
                        color="#CC79A7", edgecolor="0.2", linewidth=0.5, zorder=7,
                        label="Robust boundary targets" if not marker_labels_added else None,
                    )
                    marker_labels_added = True
                if x_limits[0] <= tb_relative <= x_limits[1]:
                    axis.scatter(
                        [tb_relative], [pz + frame_offset - d], marker="v", s=48,
                        color="#CC79A7", edgecolor="0.2", linewidth=0.5, zorder=7,
            label="Robust boundary targets" if not marker_labels_added else None,
                    )
                    marker_labels_added = True

        direction = "underestimated" if actual_offset < 0.0 else "overestimated"
        nearest_note = (
            f" (nearest to {offset:+.2f} m)"
            if abs(actual_offset - offset) > OFFSET_TOLERANCE else ""
        )
        axis.set_title(
            rf"$\Delta z={actual_offset:+.5f}$ m ({direction}){nearest_note} "
            f"[{'PASS' if item['success'] else 'FAIL'}]",
        )
        axis.set_xlabel("Time relative to FL touchdown [s]")
        axis.set_xlim(*x_limits)
        axis.set_ylim(y_min, y_max)
        style_paper_axis(axis)
        axis.text(
            0.03, 0.04,
            f"true level: {data['true_foot_frame_z']:.3f} m\n"
            f"perceived level: {perceived_foot_frame_z:.3f} m\n"
            f"measured touchdown: {data['touchdown_measured_z']:.3f} m",
            transform=axis.transAxes, fontsize=11, va="bottom",
            bbox={"boxstyle": "square,pad=0.3", "facecolor": "white",
                  "edgecolor": "0.8", "alpha": 0.90},
        )
        selection_report[f"{offset:+.2f}"] = {
            "trial": item["trial_dir"].name,
            "run": item["run"],
            "requested_terrain_z_offset_m": offset,
            "actual_terrain_z_offset_m": actual_offset,
            "robust_enabled": robust_on,
            "success": item["success"],
            "true_foot_frame_level_m": data["true_foot_frame_z"],
            "perceived_foot_frame_level_m": perceived_foot_frame_z,
            "measured_touchdown_z_m": data["touchdown_measured_z"],
        }

    axes[0].set_ylabel("FL foot-frame z [m]")
    handles, labels = axes[1].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    paper_legend(
        fig,
        unique.values(), unique.keys(), loc="upper center", ncol=4,
        bbox_to_anchor=(0.5, 0.94), fontsize=12,
    )
    fig.suptitle(
        (
            "Outcome-unfiltered " if outcome_unfiltered else "Successful "
        )
        + f"Robust-{'ON' if robust_on else 'OFF'} FL foothold response "
        + (
            "near Monte Carlo terrain-height bounds"
            if outcome_unfiltered else "at extreme terrain-height errors"
        ),
        y=0.995,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.82), w_pad=2.2)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=SAVE_DPI)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)
    output.with_name(f"{output.stem}_selection.json").write_text(
        json.dumps(selection_report, indent=2), encoding="utf-8"
    )


def plot_single_trial(
    trial_dir: Path,
    output: Path,
    urdf_path: Path,
    *,
    y_limits=None,
):
    """Create the same touchdown-aligned visualization inside one trial directory."""
    item = _trial_metadata(trial_dir)
    if not item["success"]:
        raise RuntimeError(f"Selected trial is not successful: {trial_dir}")
    if item["offset_m"] is None:
        raise RuntimeError(f"Selected trial has no terrain_z_offset: {trial_dir}")

    data = _foot_z_for_trial(trial_dir, urdf_path)
    robust_on = bool(item["robust_on"])
    actual_offset = float(item["offset_m"])
    perceived_foot_frame_z = data["true_foot_frame_z"] + actual_offset
    x_limits = (-1.15, 0.65)
    relative = data["time_relative"]
    visible = (relative >= x_limits[0]) & (relative <= x_limits[1])
    visible_values = np.concatenate(
        (
            data["opt_z"][visible],
            data["measured_z"][visible],
            np.asarray([data["true_foot_frame_z"], perceived_foot_frame_z]),
        )
    )
    if y_limits is None:
        y_min = float(np.min(visible_values)) - 0.025
        y_max = float(np.max(visible_values)) + 0.035
    else:
        y_min, y_max = map(float, y_limits)
        if y_min >= y_max:
            raise ValueError("--y-limits requires MIN < MAX")

    fig, axis = plt.subplots(figsize=paper_figsize(1, 1, width_scale=1.6, height_scale=1.6))
    axis.axvspan(
        max(x_limits[0], data["swing_start_relative"]), 0.0,
        color="#A8DADC", alpha=0.20, label="FL swing phase",
    )
    axis.axhline(
        data["true_foot_frame_z"], color=REFERENCE_COLOR, linestyle=":", linewidth=2.25,
        label="True surface + foot offset",
    )
    axis.axhline(
        perceived_foot_frame_z, color=PERCEIVED_COLOR, linestyle="--", linewidth=2.25,
        label="Perceived foothold",
    )
    axis.plot(
        relative[visible], data["opt_z"][visible], color=OPT_COLOR,
        linewidth=2.25, label="MPC-optimized foot z",
    )
    axis.plot(
        relative[visible], data["measured_z"][visible], color=MEASURED_COLOR,
        linewidth=2.25, alpha=0.90, label="Measured foot z",
    )
    axis.scatter(
        [0.0], [data["touchdown_measured_z"]], color=MEASURED_COLOR,
        edgecolor="black", linewidth=0.8, s=58, zorder=8,
        label="Touchdown (measured z)",
    )

    if robust_on:
        marker_labels_added = False
        for ta, tb, pz, d, frame_offset, clamped in data["windows"]:
            ta_relative = ta - data["touchdown_time"]
            tb_relative = tb - data["touchdown_time"]
            if x_limits[0] <= ta_relative <= x_limits[1] and not clamped:
                axis.scatter(
                    [ta_relative], [pz + frame_offset + d], marker="^", s=52,
                    color="#CC79A7", edgecolor="0.2", linewidth=0.5, zorder=7,
                    label=(
                        "Robust boundary targets"
                        if not marker_labels_added else None
                    ),
                )
                marker_labels_added = True
            if x_limits[0] <= tb_relative <= x_limits[1]:
                axis.scatter(
                    [tb_relative], [pz + frame_offset - d], marker="v", s=52,
                    color="#CC79A7", edgecolor="0.2", linewidth=0.5, zorder=7,
                    label=(
                        "Robust boundary targets"
                        if not marker_labels_added else None
                    ),
                )
                marker_labels_added = True

    direction = "underestimated" if actual_offset < 0.0 else "overestimated"
    axis.set_title(
        rf"Robust-{'ON' if robust_on else 'OFF'}, "
        rf"$\Delta z={actual_offset:+.3f}$ m ({direction}), run {item['run']}",
    )
    axis.set_xlabel("Time relative to FL touchdown [s]")
    axis.set_ylabel("FL foot-frame z [m]")
    axis.set_xlim(*x_limits)
    axis.set_ylim(y_min, y_max)
    style_paper_axis(axis)
    axis.text(
        0.03, 0.04,
        f"true level: {data['true_foot_frame_z']:.3f} m\n"
        f"perceived level: {perceived_foot_frame_z:.3f} m\n"
        f"measured touchdown: {data['touchdown_measured_z']:.3f} m",
        transform=axis.transAxes, fontsize=11, va="bottom",
        bbox={"boxstyle": "square,pad=0.3", "facecolor": "white",
              "edgecolor": "0.8", "alpha": 0.90},
    )
    handles, labels = axis.get_legend_handles_labels()
    paper_legend(
        fig, handles=handles, labels=labels, loc="lower center",
        bbox_to_anchor=(0.5, 0.02), ncol=2, fontsize=min(12, LEGEND_SIZE),
    )
    fig.subplots_adjust(left=0.14, right=0.98, top=0.88, bottom=0.33)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=SAVE_DPI)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)
    report = {
        "trial": trial_dir.name,
        "run": item["run"],
        "terrain_z_offset_m": actual_offset,
        "robust_enabled": robust_on,
        "success": item["success"],
        "true_foot_frame_level_m": data["true_foot_frame_z"],
        "perceived_foot_frame_level_m": perceived_foot_frame_z,
        "measured_touchdown_z_m": data["touchdown_measured_z"],
    }
    output.with_name(f"{output.stem}_selection.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--results-dir", type=Path)
    source.add_argument(
        "--trial-dir", type=Path,
        help="Plot one successful trial and save into that trial directory by default.",
    )
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--robust", choices=("on", "off"), default="on")
    parser.add_argument(
        "--allow-nearest", action="store_true",
        help="Use the closest eligible offset when an exact +/-0.05 trial is unavailable.",
    )
    parser.add_argument(
        "--any-outcome", action="store_true",
        help="Select PASS or FAIL trials without filtering on success.",
    )
    parser.add_argument("--output-name", default="fl_foothold_extremes.png")
    parser.add_argument(
        "--y-limits", nargs=2, type=float, metavar=("MIN", "MAX"),
        help="Optional shared y-axis limits for comparable single-trial plots.",
    )
    args = parser.parse_args(argv)

    if args.trial_dir is not None:
        trial_dir = args.trial_dir.resolve()
        out_dir = args.out_dir.resolve() if args.out_dir else trial_dir
        output_name = (
            "fl_foothold_response.png"
            if args.output_name == "fl_foothold_extremes.png"
            else args.output_name
        )
        output = out_dir / output_name
        plot_single_trial(
            trial_dir, output, args.urdf.resolve(), y_limits=args.y_limits
        )
        print(f"saved {output}")
        return 0

    results_dir = args.results_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else results_dir / "all_visualizations"
    output = out_dir / args.output_name
    robust_on = args.robust == "on"
    selected = select_trials(
        results_dir,
        robust_on=robust_on,
        allow_nearest=args.allow_nearest,
        require_success=not args.any_outcome,
    )
    plot(
        selected,
        output,
        args.urdf.resolve(),
        robust_on=robust_on,
        outcome_unfiltered=args.any_outcome,
        y_limits=args.y_limits,
    )
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

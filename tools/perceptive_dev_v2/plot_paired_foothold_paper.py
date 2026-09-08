#!/usr/bin/env python3
"""Create a paper-ready paired ON/OFF FL touchdown comparison near dz=-0.048 m."""

import argparse
import json
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from paper_plot_style import (
    LINE_WIDTH,
    NOMINAL_COLOR,
    PERCEIVED_COLOR,
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    apply_paper_style,
    style_paper_axis,
)
from plot_fl_foothold_extremes import _foot_z_for_trial, _trial_metadata
from plot_robust_phase import DEFAULT_URDF, load_tick
from trial_metrics import analyze_trial


TARGET_OFFSET_M = -0.048
PLOT_TIME_LIMITS_S = (-0.26, 0.20)
FULL_TIME_LIMITS_S = (-2.0, 2.2)


def _load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _paired_identity(trial_name: str):
    match = re.search(r"_mc_(s\d+_seed\d+)_(?:on|off)$", trial_name)
    return match.group(1) if match else None


def _select_paired_trials(results_dir: Path, target_offset_m: float):
    paired = {}
    for trial_dir in results_dir.iterdir():
        if not trial_dir.is_dir() or trial_dir.name == "all_visualizations":
            continue
        identity = _paired_identity(trial_dir.name)
        if identity is None:
            continue
        item = _trial_metadata(trial_dir)
        if item["offset_m"] is None:
            continue
        mode = "ON" if item["robust_on"] else "OFF"
        paired.setdefault(identity, {})[mode] = item

    candidates = []
    for identity, pair in paired.items():
        if set(pair) != {"ON", "OFF"}:
            continue
        on_offset = float(pair["ON"]["offset_m"])
        off_offset = float(pair["OFF"]["offset_m"])
        if abs(on_offset - off_offset) > 1e-12:
            continue
        candidates.append((abs(on_offset - target_offset_m), identity, pair))
    if not candidates:
        raise RuntimeError(f"No paired Monte Carlo sample in {results_dir}")
    return min(candidates, key=lambda item: (item[0], item[1]))[2]


def _edge_crossing_swing(trial_dir: Path, edge_time: float):
    header, tick = load_tick(trial_dir / "tick.csv")
    time = tick[:, header.index("t")]
    planned_modes = tick[:, header.index("planned_mode")].astype(np.int64)
    fl_contact = ((planned_modes >> 3) & 1).astype(bool)
    liftoff_indices = np.flatnonzero(fl_contact[:-1] & ~fl_contact[1:]) + 1
    touchdown_indices = np.flatnonzero(~fl_contact[:-1] & fl_contact[1:]) + 1

    for liftoff_index in liftoff_indices:
        following_touchdowns = touchdown_indices[touchdown_indices > liftoff_index]
        if not following_touchdowns.size:
            continue
        touchdown_index = int(following_touchdowns[0])
        liftoff_time = float(time[liftoff_index])
        touchdown_time = float(time[touchdown_index])
        if liftoff_time <= edge_time <= touchdown_time:
            following_liftoffs = liftoff_indices[liftoff_indices > touchdown_index]
            next_liftoff_time = (
                float(time[int(following_liftoffs[0])])
                if following_liftoffs.size else touchdown_time + 0.5
            )
            return {
                "liftoff_time_s": liftoff_time,
                "planned_touchdown_time_s": touchdown_time,
                "next_liftoff_time_s": next_liftoff_time,
                "planned_touchdown_mode": int(planned_modes[touchdown_index]),
            }
    raise RuntimeError(
        f"No FL swing phase contains edge crossing t={edge_time:.3f} in {trial_dir}"
    )


def _measured_lower_contact(
    trial_dir: Path,
    *,
    edge_time: float,
    next_liftoff_time: float,
    edge_x_min: float,
    lower_z_max: float,
):
    runtime_path = trial_dir / "runtime_metrics.json"
    if runtime_path.is_file():
        runtime = _load_json(runtime_path)
    else:
        runtime, _ = analyze_trial(trial_dir)
    candidates = [
        event for event in runtime.get("touchdown_events", [])
        if event.get("leg") == "FL"
        and edge_time <= float(event["time_s"]) <= next_liftoff_time
        and float(event["foot_xyz_m"][0]) >= edge_x_min
        and float(event["foot_xyz_m"][2]) <= lower_z_max
    ]
    if not candidates:
        raise RuntimeError(
            f"No measured FL lower-zone contact in edge-crossing swing for {trial_dir}"
        )
    event = min(candidates, key=lambda item: float(item["time_s"]))
    return event, runtime.get("contact_source", "unknown")


def _load_paired_trial(item: dict, urdf_path: Path):
    trial_dir = item["trial_dir"]
    trajectory = _foot_z_for_trial(trial_dir, urdf_path)
    terrain = _load_json(trial_dir / "terrain_descent_events.json")
    fl_events = terrain.get("feet", {}).get("FL", {})
    edge = fl_events.get("edge_crossing")
    if not edge:
        raise RuntimeError(f"Incomplete FL descent events in {trial_dir}")

    edge_time = float(edge["raw_time_sec"])
    swing = _edge_crossing_swing(trial_dir, edge_time)
    planned_time = swing["planned_touchdown_time_s"]
    geometry = terrain.get("geometry", {})
    edge_x = float(geometry.get("edge_x_m", 0.60))
    edge_tolerance = float(geometry.get("edge_x_tolerance_m", 0.05))
    lower_z_max = float(geometry.get("lower_touchdown_z_max_m", 0.21))
    contact, contact_source = _measured_lower_contact(
        trial_dir,
        edge_time=edge_time,
        next_liftoff_time=swing["next_liftoff_time_s"],
        edge_x_min=edge_x - edge_tolerance,
        lower_z_max=lower_z_max,
    )
    contact_time = float(contact["time_s"])
    time = trajectory["time"]
    planned_index = int(np.argmin(np.abs(time - planned_time)))
    contact_index = int(np.argmin(np.abs(time - contact_time)))
    normal_speed = contact.get("touchdown_normal_speed_mps")
    normal_speed = float(normal_speed) if normal_speed is not None else None
    contact_z = float(contact["foot_xyz_m"][2])

    return {
        "trial": trial_dir.name,
        "success": bool(item["success"]),
        "offset_m": float(item["offset_m"]),
        "time_relative_to_planned_touchdown": time - planned_time,
        "opt_z": trajectory["opt_z"],
        "measured_z": trajectory["measured_z"],
        "true_z": float(trajectory["true_foot_frame_z"]),
        "perceived_z": float(trajectory["true_foot_frame_z"] + item["offset_m"]),
        # The terrain perception error is restricted to the lower step.  The
        # upper surface therefore has one shared true/perceived level.
        "upper_z": (
            float(geometry.get("upper_surface_z_m", 0.20))
            + float(trajectory["foot_frame_offset"])
        ),
        "liftoff_time_relative_s": swing["liftoff_time_s"] - planned_time,
        "edge_time_relative_to_planned_s": edge_time - planned_time,
        "planned_time_since_edge_s": planned_time - edge_time,
        "planned_time_relative_s": 0.0,
        "planned_opt_z_m": float(trajectory["opt_z"][planned_index]),
        "planned_measured_z_m": float(trajectory["measured_z"][planned_index]),
        "contact_time_since_edge_s": contact_time - edge_time,
        "contact_time_relative_s": contact_time - planned_time,
        "contact_opt_z_m": float(trajectory["opt_z"][contact_index]),
        "contact_measured_z_m": contact_z,
        "touchdown_normal_speed_mps": normal_speed,
        "planned_to_contact_delay_s": contact_time - planned_time,
        "next_liftoff_time_relative_s": swing["next_liftoff_time_s"] - planned_time,
        "planned_touchdown_mode": swing["planned_touchdown_mode"],
        "contact_source": contact_source,
    }


def plot_paired(
    results_dir: Path,
    output: Path,
    urdf_path: Path,
    time_view: str = "swing",
):
    if time_view not in {"swing", "full"}:
        raise ValueError(f"Unsupported time view: {time_view}")
    apply_paper_style()
    selected = _select_paired_trials(results_dir, TARGET_OFFSET_M)
    on_identity = _paired_identity(selected["ON"]["trial_dir"].name)
    off_identity = _paired_identity(selected["OFF"]["trial_dir"].name)
    if on_identity is None or on_identity != off_identity:
        raise RuntimeError("Nearest Robust ON/OFF trials are not a paired MC sample")
    if abs(selected["ON"]["offset_m"] - selected["OFF"]["offset_m"]) > 1e-12:
        raise RuntimeError("Nearest Robust ON/OFF trials do not share the same dz")

    datasets = {
        mode: _load_paired_trial(selected[mode], urdf_path)
        for mode in ("ON", "OFF")
    }
    colors = {"ON": PROPOSED_COLOR, "OFF": NOMINAL_COLOR}

    x_min, x_max = (
        FULL_TIME_LIMITS_S if time_view == "full" else PLOT_TIME_LIMITS_S
    )
    visible_values = []
    for data in datasets.values():
        visible = (
            (data["time_relative_to_planned_touchdown"] >= x_min)
            & (data["time_relative_to_planned_touchdown"] <= x_max)
        )
        visible_values.extend(data["measured_z"][visible])
    visible_values.extend(
        [
            datasets["ON"]["true_z"], datasets["ON"]["perceived_z"],
            datasets["ON"]["upper_z"],
        ]
    )
    y_min = min(visible_values) - 0.012
    data_y_max = max(visible_values)
    y_max = data_y_max + 0.014
    if time_view == "full":
        # Reserve the upper 30% of the axis for the in-axis legend so it does
        # not cover either measured foot trajectory.
        y_max = y_min + (data_y_max - y_min) / 0.70

    if time_view == "full":
        fig, axis = plt.subplots(figsize=(9.6, 5.4))
        plot_groups = (
            (fig, axis, ("OFF", "ON"), None, output),
        )
    else:
        fig, axis = plt.subplots(figsize=(9.6, 5.4))
        plot_groups = ((fig, axis, ("ON", "OFF"), None, output),)

    output.parent.mkdir(parents=True, exist_ok=True)
    saved_outputs = []
    for fig, axis, modes, figure_title, figure_output in plot_groups:
        axis.axvline(
            0.0, color=REFERENCE_COLOR, linestyle="-.", linewidth=1.1,
            alpha=0.75, zorder=1,
        )
        axis.axhline(
            datasets["ON"]["upper_z"], color=REFERENCE_COLOR,
            linestyle="--", linewidth=1.5, zorder=1,
        )
        axis.axhline(
            datasets["ON"]["true_z"], color=REFERENCE_COLOR, linestyle=":",
            linewidth=1.5, zorder=1,
        )
        axis.axhline(
            datasets["ON"]["perceived_z"], color=PERCEIVED_COLOR,
            linestyle="--", linewidth=1.5, zorder=1,
        )
        for mode in modes:
            data = datasets[mode]
            visible = (
                (data["time_relative_to_planned_touchdown"] >= x_min)
                & (data["time_relative_to_planned_touchdown"] <= x_max)
            )
            axis.plot(
                data["time_relative_to_planned_touchdown"][visible],
                data["measured_z"][visible],
                color=colors[mode], linewidth=LINE_WIDTH,
            )

        axis.set_xlabel("Time relative to planned FL contact-mode transition [s]")
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min, y_max)
        style_paper_axis(axis, minor=True)
        if figure_title is not None:
            axis.text(
                0.02, 0.97, figure_title, transform=axis.transAxes,
                ha="left", va="top", fontsize=12, fontweight="bold",
            )
        axis.set_ylabel("Measured FL foot-frame z [m]")

        legend_handles = [
            Line2D(
                [0], [0], color=colors[mode], lw=LINE_WIDTH,
                label="Proposed" if mode == "ON" else "Baseline",
            )
            for mode in modes
        ]
        legend_handles.extend(
            [
                Line2D(
                    [0], [0], color=REFERENCE_COLOR, lw=1.5, ls="--",
                    label="Upper foothold level",
                ),
                Line2D(
                    [0], [0], color=REFERENCE_COLOR, lw=1.5, ls=":",
                    label="Lower foothold level",
                ),
                Line2D(
                    [0], [0], color=PERCEIVED_COLOR, lw=1.5, ls="--",
                    label="Perceived lower foothold level",
                ),
            ]
        )
        legend = axis.legend(
            handles=legend_handles, loc="upper right", ncol=1,
            fontsize=11, frameon=True, fancybox=False, framealpha=1.0,
            edgecolor="0.35", facecolor="white",
        )
        legend.get_frame().set_linewidth(0.8)
        fig.tight_layout(pad=0.45)
        fig.savefig(figure_output, dpi=300)
        fig.savefig(figure_output.with_suffix(".pdf"))
        plt.close(fig)
        saved_outputs.append((figure_output, modes[0] if len(modes) == 1 else None))

    actual_offset = datasets["ON"]["offset_m"]

    report = {
        "selection_rule": "paired Monte Carlo sample nearest to dz=-0.048 m; outcome unfiltered",
        "paired_sample": on_identity,
        "target_terrain_z_offset_m": TARGET_OFFSET_M,
        "actual_terrain_z_offset_m": actual_offset,
        "true_foot_frame_level_m": datasets["ON"]["true_z"],
        "perceived_foot_frame_level_m": datasets["ON"]["perceived_z"],
        "upper_foot_frame_level_m": datasets["ON"]["upper_z"],
        "upper_terrain_perception_error_m": 0.0,
        "time_alignment": "planned FL contact-mode transition (t=0)",
        "time_view": time_view,
        "plot_time_limits_s": [x_min, x_max],
        "contact_selection_rule": (
            "first measured FL contact after edge crossing and before the next "
            "planned liftoff, with x >= edge-tolerance and z <= 0.21 m"
        ),
        "swing_selection_rule": (
            "FL planned swing whose liftoff-to-touchdown interval contains the "
            "first edge crossing"
        ),
        "figure_content": (
            "Baseline and Proposed FL height overlaid in one Fig. 3, cropped "
            "to show the first liftoff through the pre-peak interval"
            if time_view == "full"
            else "first step-down FL swing height, phase-aligned to its planned touchdown"
        ),
        "figure_outputs": {
            ("nominal" if mode == "OFF" else "proposed" if mode == "ON" else "combined"):
            str(path)
            for path, mode in saved_outputs
        },
        "methods": datasets,
    }
    for data in report["methods"].values():
        data.pop("time_relative_to_planned_touchdown", None)
        data.pop("opt_z", None)
        data.pop("measured_z", None)
    for figure_output, mode in saved_outputs:
        figure_report = dict(report)
        figure_report["figure_method"] = (
            "Baseline" if mode == "OFF" else "Proposed" if mode == "ON" else "Combined"
        )
        figure_output.with_name(f"{figure_output.stem}_selection.json").write_text(
            json.dumps(figure_report, indent=2), encoding="utf-8"
        )
    if time_view == "full":
        for stem in (
            "fig3_paired_fl_touchdown_dz_m048_full",
            "fig4_paired_fl_touchdown_dz_m048_full",
        ):
            for path in (
                output.parent / f"{stem}.png",
                output.parent / f"{stem}.pdf",
                output.parent / f"{stem}_selection.json",
            ):
                if path != output and path != output.with_suffix(".pdf") and path.is_file():
                    path.unlink()
    return [path for path, _mode in saved_outputs]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--time-view", choices=("swing", "full"), default="swing",
        help=(
            "Plot the selected swing window or the wider -2.0 to 2.2 s "
            "Baseline/Proposed comparison."
        ),
    )
    parser.add_argument(
        "--output-name"
    )
    args = parser.parse_args(argv)
    results_dir = args.results_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else results_dir / "all_visualizations"
    output_name = args.output_name or (
        "fig3_nominal_proposed_fl_touchdown_dz_m048_full.png"
        if args.time_view == "full"
        else "fig3_paired_fl_touchdown_dz_m048.png"
    )
    output = out_dir / output_name
    outputs = plot_paired(
        results_dir, output, args.urdf.resolve(), time_view=args.time_view
    )
    for saved_output in outputs:
        print(f"saved {saved_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

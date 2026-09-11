#!/usr/bin/env python3
"""Create a paper-ready paired FL foothold x-z comparison near dz=-0.048 m."""

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
import pinocchio as pin

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from paper_plot_style import apply_figure_font_sizes
from matplotlib.lines import Line2D

from paper_plot_style import (
    LINE_WIDTH,
    NOMINAL_COLOR,
    NOMINAL_MARKER,
    PERCEIVED_COLOR,
    PROPOSED_COLOR,
    PROPOSED_MARKER,
    REFERENCE_COLOR,
    apply_paper_style,
    style_paper_axis,
)
from plot_paired_foothold_paper import (
    TARGET_OFFSET_M,
    _edge_crossing_swing,
    _load_json,
    _measured_lower_contact,
    _paired_identity,
    _select_paired_trials,
)
from plot_robust_phase import (
    DEFAULT_URDF,
    build_pin_model,
    euler_zyx_to_quat_xyzw,
    load_tick,
    slice_cols,
)


def _fl_xyz_trajectory(model, data, fl_frame_id, q_trajectory):
    xyz = np.zeros((len(q_trajectory), 3))
    for index, q_pin in enumerate(q_trajectory):
        pin.forwardKinematics(model, data, q_pin)
        pin.updateFramePlacements(model, data)
        xyz[index] = data.oMf[fl_frame_id].translation
    return xyz


def _load_trial(item: dict, urdf_path: Path):
    trial_dir = item["trial_dir"]
    header, tick = load_tick(trial_dir / "tick.csv")
    if tick.size == 0:
        raise RuntimeError(f"No valid tick rows in {trial_dir / 'tick.csv'}")

    time = tick[:, header.index("t")]
    opt_x = tick[:, slice_cols(header, "opt_x", 24)]
    measured = tick[:, slice_cols(header, "meas_rbd", 36)]
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
    opt_xyz = _fl_xyz_trajectory(model, data, frame_ids[0], q_opt)
    measured_xyz = _fl_xyz_trajectory(model, data, frame_ids[0], q_measured)

    terrain = _load_json(trial_dir / "terrain_descent_events.json")
    edge = terrain.get("feet", {}).get("FL", {}).get("edge_crossing")
    if not edge:
        raise RuntimeError(f"No FL edge crossing in {trial_dir}")
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
    planned_index = int(np.argmin(np.abs(time - planned_time)))
    contact_index = int(np.argmin(np.abs(time - contact_time)))
    relative_time = time - planned_time
    liftoff_relative = swing["liftoff_time_s"] - planned_time
    contact_relative = contact_time - planned_time
    visible = (
        (relative_time >= liftoff_relative - 1e-12)
        & (relative_time <= contact_relative + 1e-12)
    )

    foot_frame_offset = float(geometry.get("foot_frame_offset_m", 0.06))
    return {
        "trial": trial_dir.name,
        "success": bool(item["success"]),
        "offset_m": float(item["offset_m"]),
        "time_relative_s": relative_time[visible],
        "optimized_xyz_m": opt_xyz[visible],
        "measured_xyz_m": measured_xyz[visible],
        "planned_optimized_xyz_m": opt_xyz[planned_index],
        "planned_measured_xyz_m": measured_xyz[planned_index],
        "contact_xyz_m": np.asarray(contact["foot_xyz_m"], dtype=float),
        "contact_fk_xyz_m": measured_xyz[contact_index],
        "liftoff_time_relative_s": liftoff_relative,
        "edge_time_relative_s": edge_time - planned_time,
        "contact_time_relative_s": contact_relative,
        "next_liftoff_time_relative_s": swing["next_liftoff_time_s"] - planned_time,
        "planned_touchdown_mode": swing["planned_touchdown_mode"],
        "touchdown_normal_speed_mps": contact.get("touchdown_normal_speed_mps"),
        "contact_source": contact_source,
        "edge_x_m": edge_x,
        "upper_foot_frame_level_m": (
            float(geometry.get("upper_surface_z_m", 0.20)) + foot_frame_offset
        ),
        "lower_foot_frame_level_m": (
            float(geometry.get("lower_surface_z_m", 0.10)) + foot_frame_offset
        ),
    }


def _trajectory_arrow(axis, xyz, color):
    if len(xyz) < 3:
        return
    index = min(max(int(0.70 * len(xyz)), 1), len(xyz) - 1)
    start = xyz[index - 1]
    end = xyz[index]
    axis.annotate(
        "", xy=(end[0], end[2]), xytext=(start[0], start[2]),
        arrowprops={"arrowstyle": "-|>", "color": color, "lw": 1.4,
                    "mutation_scale": 12},
        zorder=6,
    )


def plot_paired_xz(results_dir: Path, output: Path, urdf_path: Path):
    apply_paper_style()
    selected = _select_paired_trials(results_dir, TARGET_OFFSET_M)
    identities = {
        mode: _paired_identity(selected[mode]["trial_dir"].name)
        for mode in ("ON", "OFF")
    }
    if identities["ON"] is None or identities["ON"] != identities["OFF"]:
        raise RuntimeError("Nearest Robust ON/OFF trials are not a paired MC sample")
    if abs(selected["ON"]["offset_m"] - selected["OFF"]["offset_m"]) > 1e-12:
        raise RuntimeError("Nearest Robust ON/OFF trials do not share the same dz")

    datasets = {
        mode: _load_trial(selected[mode], urdf_path)
        for mode in ("ON", "OFF")
    }
    colors = {"ON": PROPOSED_COLOR, "OFF": NOMINAL_COLOR}
    markers = {"ON": PROPOSED_MARKER, "OFF": NOMINAL_MARKER}

    all_x = np.concatenate([
        np.concatenate(
            (
                data["measured_xyz_m"][:, 0],
                [data["planned_optimized_xyz_m"][0], data["contact_xyz_m"][0]],
            )
        )
        for data in datasets.values()
    ])
    x_padding = max(0.015, 0.06 * float(np.ptp(all_x)))
    x_limits = (float(np.min(all_x) - x_padding), float(np.max(all_x) + x_padding))
    lower_level = datasets["ON"]["lower_foot_frame_level_m"]
    upper_level = datasets["ON"]["upper_foot_frame_level_m"]
    perceived_level = lower_level + datasets["ON"]["offset_m"]

    visible_z = np.concatenate([
        np.concatenate(
            (
                data["measured_xyz_m"][:, 2],
                [data["planned_optimized_xyz_m"][2], data["contact_xyz_m"][2]],
            )
        )
        for data in datasets.values()
    ])
    y_min = min(float(np.min(visible_z)), lower_level, perceived_level) - 0.012
    y_max = max(float(np.max(visible_z)), upper_level, perceived_level) + 0.015

    fig, axis = plt.subplots(figsize=(9.6, 5.4))
    edge_x = datasets["ON"]["edge_x_m"]
    axis.plot(
        [x_limits[0], edge_x, edge_x, x_limits[1]],
        [upper_level, upper_level, lower_level, lower_level],
        color=REFERENCE_COLOR, linestyle=":", linewidth=1.7, zorder=1,
    )
    axis.plot(
        [edge_x, x_limits[1]], [perceived_level, perceived_level],
        color=PERCEIVED_COLOR, linestyle="--", linewidth=1.7, zorder=1,
    )

    for mode in ("OFF", "ON"):
        data = datasets[mode]
        xyz = data["measured_xyz_m"]
        planned = data["planned_optimized_xyz_m"]
        contact = data["contact_xyz_m"]
        axis.plot(
            xyz[:, 0], xyz[:, 2], color=colors[mode], linewidth=LINE_WIDTH,
            zorder=3,
        )
        _trajectory_arrow(axis, xyz, colors[mode])
        axis.scatter(
            [planned[0]], [planned[2]], marker=markers[mode], s=72,
            facecolor="white", edgecolor=colors[mode], linewidth=1.5, zorder=7,
        )
        axis.scatter(
            [contact[0]], [contact[2]], marker=markers[mode], s=72,
            facecolor=colors[mode], edgecolor=colors[mode], linewidth=1.5,
            zorder=8,
        )

    axis.set_xlabel("World-frame FL foot x [m]")
    axis.set_ylabel("World-frame FL foot z [m]")
    axis.set_xlim(*x_limits)
    axis.set_ylim(y_min, y_max)
    style_paper_axis(axis, minor=True)

    legend_handles = [
        Line2D([0], [0], color=NOMINAL_COLOR, marker=NOMINAL_MARKER,
               lw=LINE_WIDTH, markersize=6, label="Baseline trajectory"),
        Line2D([0], [0], color=PROPOSED_COLOR, marker=PROPOSED_MARKER,
               lw=LINE_WIDTH, markersize=6, label="Proposed trajectory"),
        Line2D([0], [0], color=REFERENCE_COLOR, lw=1.7, ls=":",
               label="True step contact level"),
        Line2D([0], [0], color=PERCEIVED_COLOR, lw=1.7, ls="--",
               label="Perceived lower level"),
        Line2D([0], [0], marker="o", color=REFERENCE_COLOR,
               markerfacecolor="white", lw=0, markersize=6,
               label="Optimized state at contact-mode transition"),
        Line2D([0], [0], marker="o", color=REFERENCE_COLOR,
               markerfacecolor=REFERENCE_COLOR, lw=0, markersize=6,
               label="First lower-zone contact"),
    ]
    legend = fig.legend(
        handles=legend_handles, loc="upper center", ncol=3,
        bbox_to_anchor=(0.5, 0.995), fontsize=10.8,
        frameon=True, fancybox=False, framealpha=1.0,
        edgecolor="0.35", facecolor="white",
    )
    legend.get_frame().set_linewidth(0.8)
    apply_figure_font_sizes(fig, output)
    fig.tight_layout(rect=(0, 0, 1, 0.82), pad=0.35)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)

    report = {
        "selection_rule": "paired Monte Carlo sample nearest to dz=-0.048 m; outcome unfiltered",
        "paired_sample": identities["ON"],
        "target_terrain_z_offset_m": TARGET_OFFSET_M,
        "actual_terrain_z_offset_m": datasets["ON"]["offset_m"],
        "time_alignment": "planned FL contact-mode transition (t=0)",
        "trajectory_interval": "planned FL liftoff to first lower-zone contact",
        "swing_selection_rule": (
            "FL planned swing whose liftoff-to-touchdown interval contains the "
            "first edge crossing"
        ),
        "contact_selection_rule": (
            "first measured FL contact after edge crossing and before the next "
            "planned liftoff, with x >= edge-tolerance and z <= 0.21 m"
        ),
        "figure_content": (
            "first step-down FL x-z trajectory from planned liftoff to first "
            "lower-zone contact, optimized state at contact-mode transition, "
            "and true/perceived terrain levels"
        ),
        "methods": {},
    }
    array_fields = {
        "time_relative_s", "optimized_xyz_m", "measured_xyz_m"
    }
    for mode, data in datasets.items():
        report["methods"][mode] = {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in data.items()
            if key not in array_fields
        }
    output.with_name(f"{output.stem}_selection.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--output-name", default="fig4_paired_fl_foothold_xz_dz_m048.png"
    )
    args = parser.parse_args(argv)
    results_dir = args.results_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else results_dir / "all_visualizations"
    output = out_dir / args.output_name
    plot_paired_xz(results_dir, output, args.urdf.resolve())
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

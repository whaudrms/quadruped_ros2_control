#!/usr/bin/env python3
"""Plot recommended paired Monte Carlo touchdown candidates on one test page."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np

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
from plot_fl_foothold_extremes import _trial_metadata
from plot_paired_foothold_paper import (
    _load_paired_trial,
    _paired_identity,
)
from plot_paired_foothold_xz_paper import _load_trial, _trajectory_arrow
from plot_robust_phase import DEFAULT_URDF


CANDIDATE_IDS = (
    "s0038_seed341658427",
    "s0026_seed2592233787",
    "s0036_seed3887093221",
)
TIME_LIMITS_S = (-0.26, 0.20)


def _find_candidates(results_dir: Path):
    selected = {identity: {} for identity in CANDIDATE_IDS}
    for trial_dir in results_dir.iterdir():
        if not trial_dir.is_dir() or trial_dir.name == "all_visualizations":
            continue
        identity = _paired_identity(trial_dir.name)
        if identity not in selected:
            continue
        item = _trial_metadata(trial_dir)
        selected[identity]["ON" if item["robust_on"] else "OFF"] = item
    for identity, pair in selected.items():
        if set(pair) != {"ON", "OFF"}:
            raise RuntimeError(f"Incomplete paired sample: {identity}")
    return selected


def _marker(axis, x, y, *, color, marker, filled):
    axis.scatter(
        [x], [y], marker=marker, s=42,
        facecolor=color if filled else "white",
        edgecolor=color, linewidth=1.2, zorder=8,
    )


def plot_overview(results_dir: Path, output: Path, urdf_path: Path):
    apply_paper_style()
    selected = _find_candidates(results_dir)
    rows = []
    for identity in CANDIDATE_IDS:
        temporal = {
            mode: _load_paired_trial(selected[identity][mode], urdf_path)
            for mode in ("ON", "OFF")
        }
        spatial = {
            mode: _load_trial(selected[identity][mode], urdf_path)
            for mode in ("ON", "OFF")
        }
        rows.append((identity, temporal, spatial))

    all_x = []
    all_z = []
    for _identity, temporal, spatial in rows:
        for mode in ("ON", "OFF"):
            time = temporal[mode]["time_relative_to_planned_touchdown"]
            visible = (
                (time >= temporal[mode]["liftoff_time_relative_s"] - 1e-12)
                & (time <= temporal[mode]["contact_time_relative_s"] + 1e-12)
            )
            all_z.extend(temporal[mode]["measured_z"][visible])
            xyz = spatial[mode]["measured_xyz_m"]
            all_x.extend(xyz[:, 0])
            all_z.extend(xyz[:, 2])
            all_x.extend([
                spatial[mode]["planned_optimized_xyz_m"][0],
                spatial[mode]["contact_xyz_m"][0],
            ])
            all_z.extend([
                spatial[mode]["planned_optimized_xyz_m"][2],
                spatial[mode]["contact_xyz_m"][2],
                spatial[mode]["upper_foot_frame_level_m"],
                spatial[mode]["lower_foot_frame_level_m"],
                spatial[mode]["lower_foot_frame_level_m"] + spatial[mode]["offset_m"],
            ])
    x_padding = max(0.015, 0.04 * float(np.ptp(all_x)))
    spatial_x_limits = (float(np.min(all_x) - x_padding), float(np.max(all_x) + x_padding))
    y_limits = (float(np.min(all_z) - 0.012), float(np.max(all_z) + 0.014))

    fig, axes = plt.subplots(3, 2, figsize=(15.5, 12.0), sharey=True)
    colors = {"ON": PROPOSED_COLOR, "OFF": NOMINAL_COLOR}
    markers = {"ON": PROPOSED_MARKER, "OFF": NOMINAL_MARKER}

    for row_index, (identity, temporal, spatial) in enumerate(rows):
        time_axis, spatial_axis = axes[row_index]
        dz = temporal["ON"]["offset_m"]
        true_level = temporal["ON"]["true_z"]
        perceived_level = temporal["ON"]["perceived_z"]

        time_axis.axvline(
            0.0, color=REFERENCE_COLOR, linestyle="-.", linewidth=0.9,
            alpha=0.75, zorder=1,
        )
        time_axis.axhline(
            true_level, color=REFERENCE_COLOR, linestyle=":", linewidth=1.2,
        )
        time_axis.axhline(
            perceived_level, color=PERCEIVED_COLOR, linestyle="--", linewidth=1.2,
        )
        for mode in ("OFF", "ON"):
            data = temporal[mode]
            time = data["time_relative_to_planned_touchdown"]
            visible = (
                (time >= data["liftoff_time_relative_s"] - 1e-12)
                & (time <= data["contact_time_relative_s"] + 1e-12)
            )
            time_axis.plot(
                time[visible], data["measured_z"][visible],
                color=colors[mode], linewidth=1.8,
            )
            _marker(
                time_axis, 0.0, data["planned_measured_z_m"],
                color=colors[mode], marker=markers[mode], filled=False,
            )
            _marker(
                time_axis, data["contact_time_relative_s"],
                data["contact_measured_z_m"], color=colors[mode],
                marker=markers[mode], filled=True,
            )

        nominal_speed = temporal["OFF"]["touchdown_normal_speed_mps"]
        proposed_speed = temporal["ON"]["touchdown_normal_speed_mps"]
        time_axis.set_title(
            f"{identity}   $\\Delta z={dz:+.4f}$ m   "
            f"$v_n$: {nominal_speed:.3f} $\\rightarrow$ {proposed_speed:.3f} m/s",
            fontsize=12.5, pad=5,
        )
        time_axis.set_xlim(*TIME_LIMITS_S)
        time_axis.set_ylim(*y_limits)
        time_axis.set_ylabel("FL foot z [m]", fontsize=13)
        if row_index == len(rows) - 1:
            time_axis.set_xlabel(
                "Time from planned contact-mode transition [s]", fontsize=13,
            )
        style_paper_axis(time_axis, minor=True)
        time_axis.tick_params(labelsize=10)

        edge_x = spatial["ON"]["edge_x_m"]
        upper_level = spatial["ON"]["upper_foot_frame_level_m"]
        lower_level = spatial["ON"]["lower_foot_frame_level_m"]
        spatial_axis.plot(
            [spatial_x_limits[0], edge_x, edge_x, spatial_x_limits[1]],
            [upper_level, upper_level, lower_level, lower_level],
            color=REFERENCE_COLOR, linestyle=":", linewidth=1.2,
        )
        spatial_axis.plot(
            [edge_x, spatial_x_limits[1]],
            [lower_level + dz, lower_level + dz],
            color=PERCEIVED_COLOR, linestyle="--", linewidth=1.2,
        )
        for mode in ("OFF", "ON"):
            data = spatial[mode]
            xyz = data["measured_xyz_m"]
            planned = data["planned_optimized_xyz_m"]
            contact = data["contact_xyz_m"]
            spatial_axis.plot(
                xyz[:, 0], xyz[:, 2], color=colors[mode], linewidth=1.8,
            )
            _trajectory_arrow(spatial_axis, xyz, colors[mode])
            _marker(
                spatial_axis, planned[0], planned[2], color=colors[mode],
                marker=markers[mode], filled=False,
            )
            _marker(
                spatial_axis, contact[0], contact[2], color=colors[mode],
                marker=markers[mode], filled=True,
            )
        nominal_clearance = 1000.0 * (spatial["OFF"]["contact_xyz_m"][0] - edge_x)
        proposed_clearance = 1000.0 * (spatial["ON"]["contact_xyz_m"][0] - edge_x)
        spatial_axis.set_title(
            f"Edge clearance: {nominal_clearance:.0f} $\\rightarrow$ "
            f"{proposed_clearance:.0f} mm",
            fontsize=12.5, pad=5,
        )
        spatial_axis.set_xlim(*spatial_x_limits)
        spatial_axis.set_ylim(*y_limits)
        if row_index == len(rows) - 1:
            spatial_axis.set_xlabel("World-frame FL foot x [m]", fontsize=13)
        style_paper_axis(spatial_axis, minor=True)
        spatial_axis.tick_params(labelsize=10)

    legend_handles = [
        Line2D([0], [0], color=NOMINAL_COLOR, marker=NOMINAL_MARKER,
               lw=LINE_WIDTH, markersize=6, label="Baseline"),
        Line2D([0], [0], color=PROPOSED_COLOR, marker=PROPOSED_MARKER,
               lw=LINE_WIDTH, markersize=6, label="Proposed"),
        Line2D([0], [0], color=REFERENCE_COLOR, lw=1.4, ls=":",
               label="True terrain"),
        Line2D([0], [0], color=PERCEIVED_COLOR, lw=1.4, ls="--",
               label="Perceived lower level"),
        Line2D([0], [0], marker="o", color=REFERENCE_COLOR,
               markerfacecolor="white", lw=0, markersize=6,
               label="Contact-mode transition state"),
        Line2D([0], [0], marker="o", color=REFERENCE_COLOR,
               markerfacecolor=REFERENCE_COLOR, lw=0, markersize=6,
               label="First lower-zone contact"),
    ]
    legend = fig.legend(
        handles=legend_handles, loc="upper center", ncol=3,
        bbox_to_anchor=(0.5, 0.992), fontsize=11,
        frameon=True, fancybox=False, framealpha=1.0,
        edgecolor="0.35", facecolor="white",
    )
    legend.get_frame().set_linewidth(0.8)
    apply_figure_font_sizes(fig, output)
    fig.tight_layout(rect=(0, 0, 1, 0.92), h_pad=1.35, w_pad=1.0)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=220)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args(argv)
    results_dir = args.results_dir.resolve()
    output = (
        args.output.resolve() if args.output else
        results_dir / "all_visualizations" / "test_candidates" /
        "recommended_candidate_overview.png"
    )
    plot_overview(results_dir, output, args.urdf.resolve())
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

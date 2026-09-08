#!/usr/bin/env python3
"""Overlay paired FL-foot and body-z trajectories in independent figures."""

import argparse
import json
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
from plot_paired_body_z_paper import _load_body_z
from plot_paired_foothold_paper import (
    FULL_TIME_LIMITS_S,
    TARGET_OFFSET_M,
    _load_paired_trial,
    _paired_identity,
    _select_paired_trials,
)
from plot_robust_phase import DEFAULT_URDF


BODY_COLOR = (0.0, 0.6196, 0.4510)  # Okabe-Ito bluish green


def plot(results_dir: Path, out_dir: Path, urdf_path: Path):
    apply_paper_style()
    selected = _select_paired_trials(results_dir, TARGET_OFFSET_M)
    identities = {
        _paired_identity(selected[mode]["trial_dir"].name) for mode in ("ON", "OFF")
    }
    if len(identities) != 1 or None in identities:
        raise RuntimeError("Selected Baseline/Proposed trials are not one paired sample")
    paired_sample = identities.pop()

    foot = {
        mode: _load_paired_trial(selected[mode], urdf_path) for mode in ("OFF", "ON")
    }
    body = {
        mode: _load_body_z(selected[mode], urdf_path) for mode in ("OFF", "ON")
    }
    x_min, x_max = FULL_TIME_LIMITS_S
    visible_values = [
        foot["ON"]["upper_z"], foot["ON"]["true_z"], foot["ON"]["perceived_z"]
    ]
    for mode in ("OFF", "ON"):
        foot_visible = (
            (foot[mode]["time_relative_to_planned_touchdown"] >= x_min)
            & (foot[mode]["time_relative_to_planned_touchdown"] <= x_max)
        )
        body_visible = (body[mode]["time_s"] >= x_min) & (body[mode]["time_s"] <= x_max)
        visible_values.extend(foot[mode]["measured_z"][foot_visible])
        visible_values.extend(body[mode]["measured_z_m"][body_visible])
        visible_values.extend(body[mode]["reference_z_m"][body_visible])
    y_span = float(np.ptp(visible_values))
    y_pad = max(0.012, 0.045 * y_span)
    y_min = float(np.min(visible_values) - y_pad)
    y_max = float(np.max(visible_values) + y_pad)

    out_dir.mkdir(parents=True, exist_ok=True)
    specs = (
        ("OFF", "Baseline", NOMINAL_COLOR, out_dir / "fig7_nominal_body_foot_z.png", 7),
        ("ON", "Proposed", PROPOSED_COLOR, out_dir / "fig8_proposed_body_foot_z.png", 8),
    )
    outputs = []
    for mode, label, foot_color, output, figure_number in specs:
        foot_data = foot[mode]
        body_data = body[mode]
        foot_visible = (
            (foot_data["time_relative_to_planned_touchdown"] >= x_min)
            & (foot_data["time_relative_to_planned_touchdown"] <= x_max)
        )
        body_visible = (
            (body_data["time_s"] >= x_min) & (body_data["time_s"] <= x_max)
        )

        fig, axis = plt.subplots(figsize=(9.6, 5.4))
        axis.axvline(
            0.0, color=REFERENCE_COLOR, linestyle="-.", linewidth=1.1,
            alpha=0.75, zorder=1,
        )
        axis.axhline(
            foot_data["upper_z"], color=REFERENCE_COLOR,
            linestyle="--", linewidth=1.4, zorder=1,
        )
        axis.axhline(
            foot_data["true_z"], color=REFERENCE_COLOR,
            linestyle=":", linewidth=1.4, zorder=1,
        )
        axis.axhline(
            foot_data["perceived_z"], color=PERCEIVED_COLOR,
            linestyle="--", linewidth=1.4, zorder=1,
        )
        axis.plot(
            foot_data["time_relative_to_planned_touchdown"][foot_visible],
            foot_data["measured_z"][foot_visible], color=foot_color,
            linestyle="-", linewidth=LINE_WIDTH, zorder=4,
        )
        axis.plot(
            body_data["time_s"][body_visible],
            body_data["measured_z_m"][body_visible], color=BODY_COLOR,
            linestyle="-", linewidth=LINE_WIDTH, zorder=4,
        )
        axis.plot(
            body_data["time_s"][body_visible],
            body_data["reference_z_m"][body_visible], color=REFERENCE_COLOR,
            linestyle=(0, (6, 2, 1, 2)), linewidth=1.7, zorder=5,
        )
        axis.text(
            0.02, 0.97, label, transform=axis.transAxes,
            ha="left", va="top", fontsize=12, fontweight="bold",
        )
        axis.set_xlabel("Time relative to planned FL contact-mode transition [s]")
        axis.set_ylabel("World-frame z [m]")
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min, y_max)
        style_paper_axis(axis, minor=True)
        legend = axis.legend(
            handles=[
                Line2D([0], [0], color=foot_color, lw=LINE_WIDTH,
                       label="Measured FL foot z"),
                Line2D([0], [0], color=BODY_COLOR, lw=LINE_WIDTH,
                       label="Measured body z"),
                Line2D([0], [0], color=REFERENCE_COLOR, lw=1.7,
                       ls=(0, (6, 2, 1, 2)), label="MPC body-z reference"),
                Line2D([0], [0], color=REFERENCE_COLOR, lw=1.4, ls="--",
                       label="Upper foothold level (no perception error)"),
                Line2D([0], [0], color=REFERENCE_COLOR, lw=1.4, ls=":",
                       label="True lower foothold level"),
                Line2D([0], [0], color=PERCEIVED_COLOR, lw=1.4, ls="--",
                       label="Perceived lower foothold level"),
            ],
            loc="upper right", ncol=2, fontsize=9.5,
            frameon=True, fancybox=False, framealpha=1.0,
            edgecolor="0.35", facecolor="white",
        )
        legend.get_frame().set_linewidth(0.8)
        fig.tight_layout(pad=0.45)
        fig.savefig(output, dpi=300)
        fig.savefig(output.with_suffix(".pdf"))
        plt.close(fig)

        report = {
            "figure": figure_number,
            "method": label,
            "paired_sample": paired_sample,
            "trial": foot_data["trial"],
            "success": foot_data["success"],
            "terrain_z_offset_m": foot_data["offset_m"],
            "time_alignment": "planned FL contact-mode transition (t=0)",
            "plot_time_limits_s": [x_min, x_max],
            "shared_y_limits_m": [y_min, y_max],
            "measured_foot_z_source": "tick.csv meas_rbd + FL forward kinematics",
            "measured_body_z_source": "tick.csv meas_rbd5",
            "body_reference_source": "tick.csv opt_x8",
            "body_reference_definition": (
                "policy-evaluated MPC optimized base z passed to WBC"
            ),
            "upper_foot_frame_level_m": foot_data["upper_z"],
            "upper_terrain_perception_error_m": 0.0,
            "true_lower_foot_frame_level_m": foot_data["true_z"],
            "perceived_lower_foot_frame_level_m": foot_data["perceived_z"],
        }
        output.with_name(f"{output.stem}_selection.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        outputs.append(output)
    return outputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args(argv)
    results_dir = args.results_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else results_dir / "all_visualizations"
    for output in plot(results_dir, out_dir, args.urdf.resolve()):
        print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

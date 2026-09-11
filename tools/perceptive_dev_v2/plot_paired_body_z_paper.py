#!/usr/bin/env python3
"""Create independent Baseline/Proposed body-z tracking figures."""

import argparse
import json
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
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    apply_paper_style,
    style_paper_axis,
)
from plot_paired_foothold_paper import (
    FULL_TIME_LIMITS_S,
    TARGET_OFFSET_M,
    _load_paired_trial,
    _paired_identity,
    _select_paired_trials,
)
from plot_robust_phase import DEFAULT_URDF, load_tick


def _load_body_z(item: dict, urdf_path: Path):
    alignment = _load_paired_trial(item, urdf_path)
    header, tick = load_tick(item["trial_dir"] / "tick.csv")
    if len(tick) != len(alignment["time_relative_to_planned_touchdown"]):
        raise RuntimeError(f"Inconsistent tick rows in {item['trial_dir']}")
    return {
        "trial": item["trial_dir"].name,
        "success": bool(item["success"]),
        "offset_m": float(item["offset_m"]),
        "time_s": alignment["time_relative_to_planned_touchdown"],
        # OCS2 centroidal state: r_b = opt_x[6:9].  The evaluated policy's
        # base z is the command passed to WBC, so opt_x8 is the available
        # closed-loop body-z reference in these legacy Monte Carlo logs.
        "reference_z_m": tick[:, header.index("opt_x8")],
        # Measured RBD state: r_b = meas_rbd[3:6].
        "measured_z_m": tick[:, header.index("meas_rbd5")],
        "planned_touchdown_mode": alignment["planned_touchdown_mode"],
    }


def plot(results_dir: Path, out_dir: Path, urdf_path: Path):
    apply_paper_style()
    selected = _select_paired_trials(results_dir, TARGET_OFFSET_M)
    identities = {
        _paired_identity(selected[mode]["trial_dir"].name) for mode in ("ON", "OFF")
    }
    if len(identities) != 1 or None in identities:
        raise RuntimeError("Selected Baseline/Proposed trials are not one paired sample")
    paired_sample = identities.pop()

    datasets = {
        mode: _load_body_z(selected[mode], urdf_path) for mode in ("OFF", "ON")
    }
    x_min, x_max = FULL_TIME_LIMITS_S
    visible_values = []
    for data in datasets.values():
        visible = (data["time_s"] >= x_min) & (data["time_s"] <= x_max)
        visible_values.extend(data["measured_z_m"][visible])
        visible_values.extend(data["reference_z_m"][visible])
    y_span = float(np.ptp(visible_values))
    y_pad = max(0.008, 0.06 * y_span)
    y_min = float(np.min(visible_values) - y_pad)
    y_max = float(np.max(visible_values) + y_pad)

    out_dir.mkdir(parents=True, exist_ok=True)
    specs = (
        ("OFF", "Baseline", NOMINAL_COLOR, out_dir / "fig5_nominal_body_z_vs_reference.png"),
        ("ON", "Proposed", PROPOSED_COLOR, out_dir / "fig6_proposed_body_z_vs_reference.png"),
    )
    outputs = []
    for mode, label, color, output in specs:
        data = datasets[mode]
        visible = (data["time_s"] >= x_min) & (data["time_s"] <= x_max)
        fig, axis = plt.subplots(figsize=(9.6, 5.4))
        axis.axvline(
            0.0, color=REFERENCE_COLOR, linestyle="-.", linewidth=1.1,
            alpha=0.75, zorder=1,
        )
        axis.plot(
            data["time_s"][visible], data["measured_z_m"][visible],
            color=color, linestyle="-", linewidth=LINE_WIDTH, zorder=3,
        )
        axis.plot(
            data["time_s"][visible], data["reference_z_m"][visible],
            color=REFERENCE_COLOR, linestyle="--", linewidth=1.7, zorder=4,
        )
        axis.text(
            0.02, 0.97, label, transform=axis.transAxes,
            ha="left", va="top", fontsize=12, fontweight="bold",
        )
        axis.set_xlabel("Time relative to planned FL contact-mode transition [s]")
        axis.set_ylabel("Body z [m]")
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min, y_max)
        style_paper_axis(axis, minor=True)
        legend = axis.legend(
            handles=[
                Line2D(
                    [0], [0], color=color, lw=LINE_WIDTH,
                    label=f"{label} measured body z",
                ),
                Line2D(
                    [0], [0], color=REFERENCE_COLOR, lw=1.7, ls="--",
                    label="MPC body-z reference",
                ),
            ],
            loc="upper right", ncol=1, fontsize=11,
            frameon=True, fancybox=False, framealpha=1.0,
            edgecolor="0.35", facecolor="white",
        )
        legend.get_frame().set_linewidth(0.8)
        apply_figure_font_sizes(fig, output)
        fig.tight_layout(pad=0.45)
        fig.savefig(output, dpi=300)
        fig.savefig(output.with_suffix(".pdf"))
        plt.close(fig)

        error = data["measured_z_m"][visible] - data["reference_z_m"][visible]
        report = {
            "figure": 5 if mode == "OFF" else 6,
            "method": label,
            "paired_sample": paired_sample,
            "trial": data["trial"],
            "success": data["success"],
            "terrain_z_offset_m": data["offset_m"],
            "time_alignment": "planned FL contact-mode transition (t=0)",
            "plot_time_limits_s": [x_min, x_max],
            "shared_y_limits_m": [y_min, y_max],
            "measured_body_z_source": "tick.csv meas_rbd5",
            "body_reference_source": "tick.csv opt_x8",
            "body_reference_definition": (
                "policy-evaluated MPC optimized base z passed to WBC; "
                "not the unlogged raw TargetTrajectories body-height reference"
            ),
            "tracking_rmse_m": float(np.sqrt(np.mean(error**2))),
            "tracking_max_abs_error_m": float(np.max(np.abs(error))),
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

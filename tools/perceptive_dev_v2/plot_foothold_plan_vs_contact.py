#!/usr/bin/env python3
"""Plot saved pre-swing terrain projection against the physical FL contact."""

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
from plot_robust_phase import DEFAULT_URDF


FOOT_DEBUG_RE = re.compile(
    r"\[FootPlacementDebug\] t=(?P<t>[-+0-9.eE]+).*?"
    r"proj_z=\[(?P<values>[^]]+)\]"
)
ROBUST_RE = re.compile(
    r"\[robust_phase\] t=(?P<t>[-+0-9.eE]+) leg=(?P<leg>\d+) "
    r"active=(?P<active>[01]) ta=(?P<ta>[-+0-9.eE]+) "
    r"tb=(?P<tb>[-+0-9.eE]+) pz=(?P<pz>[-+0-9.eE]+) "
    r"d=(?P<d>[-+0-9.eE]+) offset=(?P<offset>[-+0-9.eE]+)"
)


def _json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _trial_mode(trial_dir: Path):
    config = _json(trial_dir / "run_config.json")
    enabled = config["effective_task_parameters"]["robustPhase"]["enabled"]
    return "proposed" if str(enabled).lower() == "true" else "nominal"


def _select_trials(results_dir: Path):
    trials = {}
    for trial_dir in sorted(results_dir.iterdir()):
        if not trial_dir.is_dir() or not (trial_dir / "run_config.json").is_file():
            continue
        mode = _trial_mode(trial_dir)
        if mode in trials:
            raise RuntimeError(f"Multiple {mode} trials in {results_dir}")
        trials[mode] = trial_dir
    if set(trials) != {"nominal", "proposed"}:
        raise RuntimeError(f"Expected one nominal and one proposed trial in {results_dir}")
    return trials


def _load_modes(tick_path: Path):
    with tick_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    time = np.asarray([float(row["t"]) for row in rows])
    planned = np.asarray([int(float(row["planned_mode"])) for row in rows])
    return time, planned


def _fl_transitions(time: np.ndarray, modes: np.ndarray):
    contact = ((modes >> 3) & 1).astype(bool)
    liftoff = np.flatnonzero(contact[:-1] & ~contact[1:]) + 1
    touchdown = np.flatnonzero(~contact[:-1] & contact[1:]) + 1
    return time[liftoff], time[touchdown]


def _foot_projection_samples(controller_log: Path):
    samples = []
    for match in FOOT_DEBUG_RE.finditer(controller_log.read_text(errors="replace")):
        values = match.group("values").split(",")
        try:
            fl_projection = float(values[0])
        except (ValueError, IndexError):
            continue
        if np.isfinite(fl_projection):
            samples.append((float(match.group("t")), fl_projection))
    return samples


def _robust_samples(controller_log: Path):
    samples = []
    for match in ROBUST_RE.finditer(controller_log.read_text(errors="replace")):
        if int(match.group("leg")) != 0 or int(match.group("active")) != 1:
            continue
        samples.append({key: float(match.group(key)) for key in ("t", "ta", "tb", "pz", "d", "offset")})
    return samples


def _lower_contact(trial_dir: Path, liftoff: float, next_liftoff: float, z_limit: float):
    runtime = _json(trial_dir / "runtime_metrics.json")
    candidates = [
        event for event in runtime.get("touchdown_events", [])
        if event.get("leg") == "FL"
        and liftoff <= float(event["time_s"]) < next_liftoff
        and float(event["foot_xyz_m"][0]) >= 0.55
        and float(event["foot_xyz_m"][2]) <= z_limit
    ]
    if not candidates:
        raise RuntimeError(f"No FL lower-plane contact after t={liftoff:.3f} in {trial_dir}")
    return min(candidates, key=lambda event: float(event["time_s"]))


def _load_trial(trial_dir: Path, urdf_path: Path):
    config = _json(trial_dir / "run_config.json")
    terrain = _json(trial_dir / "terrain_descent_events.json")
    geometry = terrain["geometry"]
    true_surface = float(geometry["lower_surface_z_m"])
    foot_offset = float(geometry["foot_frame_offset_m"])
    perceived_surface = true_surface + float(config["terrain_z_offset"])

    trajectory = _foot_z_for_trial(trial_dir, urdf_path)
    time = np.asarray(trajectory["time"])
    contact_point_z = np.asarray(trajectory["measured_z"]) - foot_offset
    planned_time, planned_modes = _load_modes(trial_dir / "tick.csv")
    liftoffs, touchdowns = _fl_transitions(planned_time, planned_modes)

    projection_candidates = [
        sample for sample in _foot_projection_samples(trial_dir / "controller.log")
        if abs(sample[1] - perceived_surface) <= 0.005
    ]
    if not projection_candidates:
        raise RuntimeError(f"No saved FL lower-plane projection in {trial_dir}")

    edge_time = float(terrain["feet"]["FL"]["edge_crossing"]["raw_time_sec"])
    projection_time, projection_z = min(
        (sample for sample in projection_candidates if sample[0] >= edge_time),
        key=lambda sample: sample[0],
    )
    following_liftoffs = liftoffs[liftoffs > projection_time]
    if not following_liftoffs.size:
        raise RuntimeError(f"No FL liftoff after saved projection in {trial_dir}")
    liftoff = float(following_liftoffs[0])
    following_touchdowns = touchdowns[touchdowns > liftoff]
    if not following_touchdowns.size:
        raise RuntimeError(f"No planned FL touchdown after t={liftoff:.3f}")
    planned_touchdown = float(following_touchdowns[0])
    later_liftoffs = liftoffs[liftoffs > planned_touchdown]
    next_liftoff = float(later_liftoffs[0]) if later_liftoffs.size else planned_touchdown + 0.5
    contact = _lower_contact(
        trial_dir, liftoff, next_liftoff,
        float(geometry.get("lower_touchdown_z_max_m", 0.21)),
    )
    actual_touchdown = float(contact["time_s"])

    relative_time = time - planned_touchdown
    visible = (
        (time >= projection_time - 0.02)
        & (time <= max(actual_touchdown, planned_touchdown) + 0.06)
    )
    robust_window = None
    if _trial_mode(trial_dir) == "proposed":
        robust_candidates = [
            sample for sample in _robust_samples(trial_dir / "controller.log")
            if sample["t"] <= liftoff + 1e-9
            and sample["ta"] >= liftoff - 0.05
            and abs(sample["tb"] - planned_touchdown) <= 0.06
            and abs(sample["pz"] - perceived_surface) <= 0.005
        ]
        if robust_candidates:
            robust_window = dict(max(robust_candidates, key=lambda sample: sample["t"]))
            for key in ("t", "ta", "tb"):
                robust_window[key] -= planned_touchdown

    return {
        "trial": trial_dir.name,
        "time": relative_time[visible],
        "contact_point_z": contact_point_z[visible],
        "projection_time": projection_time - planned_touchdown,
        "projection_z": projection_z,
        "liftoff_time": liftoff - planned_touchdown,
        "planned_touchdown": 0.0,
        "actual_touchdown": actual_touchdown - planned_touchdown,
        "actual_contact_z": float(contact["foot_xyz_m"][2]) - foot_offset,
        "actual_contact_x": float(contact["foot_xyz_m"][0]),
        "true_surface": true_surface,
        "perceived_surface": perceived_surface,
        "robust_window": robust_window,
        "robust_d": float(config["effective_task_parameters"]["robustPhase"]["d"]),
    }


def _plot(results_dir: Path, output: Path, urdf_path: Path):
    apply_paper_style()
    trials = _select_trials(results_dir)
    data = {mode: _load_trial(path, urdf_path) for mode, path in trials.items()}
    colors = {"nominal": NOMINAL_COLOR, "proposed": PROPOSED_COLOR}
    markers = {"nominal": NOMINAL_MARKER, "proposed": PROPOSED_MARKER}

    all_time = np.concatenate([item["time"] for item in data.values()])
    x_min = float(np.min(all_time) - 0.015)
    x_max = float(np.max(all_time) + 0.015)
    all_z = np.concatenate([item["contact_point_z"] for item in data.values()])
    y_min = min(float(np.min(all_z)), data["nominal"]["perceived_surface"] - data["nominal"]["robust_d"]) - 0.01
    y_max = max(float(np.max(all_z)), data["nominal"]["true_surface"]) + 0.012

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2), sharex=True, sharey=True)
    for axis, mode, panel in zip(axes, ("nominal", "proposed"), ("(a) Baseline", "(b) Proposed")):
        item = data[mode]
        if mode == "proposed" and item["robust_window"] is not None:
            window = item["robust_window"]
            axis.fill_between(
                [window["ta"], window["tb"]],
                window["pz"] - window["d"],
                window["pz"] + window["d"],
                color=PROPOSED_COLOR, alpha=0.13, linewidth=0, zorder=0,
            )
        axis.axhline(
            item["true_surface"], color=REFERENCE_COLOR, linestyle=":",
            linewidth=1.7, zorder=1,
        )
        axis.axhline(
            item["perceived_surface"], color=PERCEIVED_COLOR, linestyle="--",
            linewidth=1.7, zorder=1,
        )
        axis.axvline(0.0, color=REFERENCE_COLOR, linestyle="-.", linewidth=1.1, zorder=1)
        axis.plot(
            item["time"], item["contact_point_z"], color=colors[mode],
            linewidth=LINE_WIDTH, zorder=4,
        )
        axis.scatter(
            [item["projection_time"]], [item["projection_z"]], marker="D", s=64,
            facecolor="white", edgecolor=PERCEIVED_COLOR, linewidth=1.5, zorder=7,
        )
        axis.scatter(
            [item["actual_touchdown"]], [item["actual_contact_z"]],
            marker=markers[mode], s=80, facecolor=colors[mode],
            edgecolor=colors[mode], linewidth=1.4, zorder=8,
        )
        axis.annotate(
            f"actual contact\n{item['actual_touchdown'] * 1000:+.0f} ms",
            xy=(item["actual_touchdown"], item["actual_contact_z"]),
            xytext=(7, 20), textcoords="offset points", ha="left", va="bottom",
            fontsize=12, color=colors[mode],
            arrowprops={"arrowstyle": "-", "color": colors[mode], "lw": 1.0},
        )
        axis.text(
            0.025, 0.95, panel, transform=axis.transAxes,
            ha="left", va="top", fontsize=15,
        )
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min, y_max)
        axis.set_xlabel("Time relative to scheduled touchdown [s]")
        style_paper_axis(axis, minor=True)
    axes[0].set_ylabel("FL contact-point height [m]")

    handles = [
        Line2D([0], [0], color=NOMINAL_COLOR, linewidth=LINE_WIDTH, label="Baseline measured"),
        Line2D([0], [0], color=PROPOSED_COLOR, linewidth=LINE_WIDTH, label="Proposed measured"),
        Line2D([0], [0], color=REFERENCE_COLOR, linestyle=":", linewidth=1.7, label="True surface"),
        Line2D([0], [0], color=PERCEIVED_COLOR, linestyle="--", linewidth=1.7, label="Perceived target"),
        Line2D([0], [0], marker="D", linestyle="none", markerfacecolor="white",
               markeredgecolor=PERCEIVED_COLOR, markersize=7, label="Saved projection"),
        Line2D([0], [0], color=REFERENCE_COLOR, linestyle="-.", linewidth=1.1,
               label="Scheduled touchdown"),
        Patch(facecolor=PROPOSED_COLOR, alpha=0.13, edgecolor="none", label="Robust window"),
    ]
    paper_legend(
        axes[0], handles=handles, loc="upper center", bbox_to_anchor=(1.02, -0.18),
        ncol=4,
    )
    fig.subplots_adjust(left=0.09, right=0.985, top=0.97, bottom=0.25, wspace=0.10)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)

    summary = {
        mode: {
            "trial": item["trial"],
            "saved_projection_time_relative_s": item["projection_time"],
            "perceived_surface_z_m": item["projection_z"],
            "true_surface_z_m": item["true_surface"],
            "planned_touchdown_time_relative_s": 0.0,
            "actual_touchdown_time_relative_s": item["actual_touchdown"],
            "actual_contact_point_z_m": item["actual_contact_z"],
            "actual_contact_x_m": item["actual_contact_x"],
            "robust_window": item["robust_window"],
        }
        for mode, item in data.items()
    }
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args()
    output = args.output or args.results_dir / "all_visualizations" / "fig_foothold_plan_vs_contact.png"
    _plot(args.results_dir.resolve(), output.resolve(), args.urdf.resolve())
    print(output.resolve())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Plot full Monte Carlo body-frame velocity means with timewise 95% CIs."""

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from paper_plot_style import (
    LINE_WIDTH,
    NOMINAL_COLOR,
    PROPOSED_COLOR,
    REFERENCE_COLOR,
    apply_paper_style,
    style_paper_axis,
)
from plot_robust_phase import DEFAULT_URDF, load_tick
from terrain_descent_events import descent_times, load_or_detect_events
from wbc_plot_utils import discover_trials


TIME_STEP_S = 0.005
CONFIDENCE_Z = 1.96


def _forward_command_profile(trial_dir: Path, sample_time: np.ndarray):
    scenario_path = trial_dir / "scenario.yaml"
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    elapsed = 0.0
    enter_ocs2_start = None
    scheduled_steps = []
    for step in scenario.get("steps", []):
        duration = float(step["duration"])
        if step.get("name") == "enter_ocs2":
            enter_ocs2_start = elapsed
        command_x = float(step.get("twist", {}).get("linear", {}).get("x", 0.0))
        scheduled_steps.append((elapsed, elapsed + duration, command_x, step.get("name")))
        elapsed += duration
    if enter_ocs2_start is None:
        raise RuntimeError(f"No enter_ocs2 step in {scenario_path}")

    scenario_elapsed = enter_ocs2_start + (sample_time - sample_time[0])
    command = np.zeros(sample_time.shape, dtype=float)
    for start, end, command_x, _name in scheduled_steps:
        active = (scenario_elapsed >= start) & (scenario_elapsed < end)
        command[active] = command_x
    return command, scenario_elapsed


def _world_to_body_vx(measured_rbd: np.ndarray):
    yaw = measured_rbd[:, 0]
    pitch = measured_rbd[:, 1]
    velocity_world = measured_rbd[:, 21:24]
    cos_pitch = np.cos(pitch)
    return (
        np.cos(yaw) * cos_pitch * velocity_world[:, 0]
        + np.sin(yaw) * cos_pitch * velocity_world[:, 1]
        - np.sin(pitch) * velocity_world[:, 2]
    )


def _load_velocity(trial: dict):
    header, tick = load_tick(trial["tick_path"])
    measured_columns = [header.index(f"meas_rbd{index}") for index in range(36)]
    measured_rbd = tick[:, measured_columns]
    tick_time = tick[:, header.index("t")]
    command, scenario_time = _forward_command_profile(trial["trial_dir"], tick_time)
    touchdown, _descent_complete = descent_times(
        load_or_detect_events(trial["trial_dir"])
    )
    if touchdown is None:
        raise RuntimeError(f"No first lower touchdown in {trial['trial_dir']}")
    return {
        "trial": trial["trial_dir"].name,
        "success": bool(trial.get("result", {}).get("success")),
        "robust": trial["robust"],
        "offset_m": float(trial["offset_m"]),
        "time_s": scenario_time,
        "first_lower_touchdown_scenario_time_s": float(
            scenario_time[0] + touchdown
        ),
        "command_x_mps": command,
        "measured_body_vx_mps": _world_to_body_vx(measured_rbd),
    }


def _mean_ci(values: np.ndarray):
    count = np.sum(np.isfinite(values), axis=0)
    mean = np.divide(
        np.nansum(values, axis=0),
        count,
        out=np.full(values.shape[1], np.nan),
        where=count > 0,
    )
    sample_std = np.full(mean.shape, np.nan)
    for index, n_valid in enumerate(count):
        if n_valid > 1:
            sample_std[index] = np.nanstd(values[:, index], ddof=1)
        elif n_valid == 1:
            sample_std[index] = 0.0
    half_width = CONFIDENCE_Z * sample_std / np.sqrt(np.maximum(count, 1))
    return mean, mean - half_width, mean + half_width, count


def _interpolate_stack(datasets: list[dict], key: str, time_grid: np.ndarray):
    stack = []
    for data in datasets:
        values = np.full(time_grid.shape, np.nan)
        valid = (time_grid >= data["time_s"][0]) & (time_grid <= data["time_s"][-1])
        values[valid] = np.interp(time_grid[valid], data["time_s"], data[key])
        stack.append(values)
    return np.asarray(stack, dtype=float)


def _aggregate_method(datasets: list[dict], time_grid: np.ndarray):
    measured = _interpolate_stack(datasets, "measured_body_vx_mps", time_grid)
    mean, lower, upper, count = _mean_ci(measured)
    per_trial_rmse = np.asarray(
        [
            np.sqrt(np.mean((data["measured_body_vx_mps"] - data["command_x_mps"]) ** 2))
            for data in datasets
        ],
        dtype=float,
    )
    return {
        "n": len(datasets),
        "success_count": sum(data["success"] for data in datasets),
        "mean": mean,
        "lower": lower,
        "upper": upper,
        "sample_count": count,
        "per_trial_tracking_rmse_mean_mps": float(np.mean(per_trial_rmse)),
        "per_trial_tracking_rmse_std_mps": float(np.std(per_trial_rmse, ddof=1)),
    }


def plot(results_dir: Path, out_dir: Path, _urdf_path: Path | None = None):
    apply_paper_style()
    if not (results_dir / "samples.csv").is_file():
        raise RuntimeError(f"Not a Monte Carlo result directory: {results_dir}")

    trials = discover_trials(results_dir)
    datasets = [_load_velocity(trial) for trial in trials]
    groups = {
        robust: [data for data in datasets if data["robust"] == robust]
        for robust in ("OFF", "ON")
    }
    if not all(groups.values()):
        raise RuntimeError("Monte Carlo Baseline/Proposed velocity trials were not both found")

    x_min = float(np.floor(min(data["time_s"][0] for data in datasets) * 10.0) / 10.0)
    x_max = float(np.ceil(max(data["time_s"][-1] for data in datasets) * 10.0) / 10.0)
    time_grid = np.arange(x_min, x_max + 1e-9, TIME_STEP_S)
    aggregate = {
        robust: _aggregate_method(group, time_grid)
        for robust, group in groups.items()
    }
    command_stack = _interpolate_stack(datasets, "command_x_mps", time_grid)
    command_mean, _command_lower, _command_upper, command_count = _mean_ci(command_stack)
    touchdown_times = np.asarray(
        [data["first_lower_touchdown_scenario_time_s"] for data in datasets],
        dtype=float,
    )
    mean_touchdown_time = float(np.mean(touchdown_times))

    out_dir.mkdir(parents=True, exist_ok=True)
    output = out_dir / "fig9_nominal_proposed_vcmd_vs_body_vx_full.png"
    fig, axis = plt.subplots(figsize=(8.0, 4.5))
    axis.axvline(
        mean_touchdown_time, color=REFERENCE_COLOR, linestyle="-.",
        linewidth=1.1, alpha=0.75, zorder=2,
    )
    for robust, color in (("OFF", NOMINAL_COLOR), ("ON", PROPOSED_COLOR)):
        data = aggregate[robust]
        axis.fill_between(
            time_grid, data["lower"], data["upper"], color=color,
            alpha=0.18, linewidth=0, zorder=1,
        )
        axis.plot(
            time_grid, data["mean"], color=color, linestyle="-",
            linewidth=LINE_WIDTH, zorder=3,
        )
    axis.plot(
        time_grid, command_mean, color=REFERENCE_COLOR, linestyle="--",
        linewidth=1.7, zorder=4,
    )

    visible_values = [command_mean]
    for data in aggregate.values():
        visible_values.extend((data["lower"], data["upper"]))
    finite_values = np.concatenate(
        [values[np.isfinite(values)] for values in visible_values]
    )
    data_min = float(np.min(finite_values))
    data_max = float(np.max(finite_values))
    y_pad = max(0.025, 0.06 * (data_max - data_min))
    y_min = data_min - y_pad
    data_y_max = data_max + y_pad
    y_max = y_min + (data_y_max - y_min) / 0.72

    axis.set_xlabel("Scenario time [s]")
    axis.set_ylabel("Forward velocity [m/s]")
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    style_paper_axis(axis, minor=True)
    legend = axis.legend(
        handles=[
            Line2D([0], [0], color=NOMINAL_COLOR, lw=LINE_WIDTH,
                   label="Baseline body-frame $v_x$"),
            Line2D([0], [0], color=PROPOSED_COLOR, lw=LINE_WIDTH,
                   label="Proposed body-frame $v_x$"),
            Line2D([0], [0], color=REFERENCE_COLOR, lw=1.7, ls="--",
                   label="$v_{cmd}$"),
            Line2D([0], [0], color=REFERENCE_COLOR, lw=1.1, ls="-.",
                   label="Lower-step touchdown (mean)"),
        ],
        loc="upper right", ncol=1, fontsize=11,
        frameon=True, fancybox=False, framealpha=1.0,
        edgecolor="0.35", facecolor="white",
    )
    legend.get_frame().set_linewidth(0.8)
    fig.tight_layout(pad=0.35)
    fig.savefig(output, dpi=300)
    fig.savefig(output.with_suffix(".pdf"))
    plt.close(fig)

    report_methods = {}
    for robust, label in (("OFF", "Baseline"), ("ON", "Proposed")):
        data = aggregate[robust]
        report_methods[robust] = {
            "method": label,
            "trial_count": int(data["n"]),
            "successful_trial_count": int(data["success_count"]),
            "timewise_sample_count_min": int(np.min(data["sample_count"])),
            "timewise_sample_count_max": int(np.max(data["sample_count"])),
            "per_trial_tracking_rmse_mean_mps": data[
                "per_trial_tracking_rmse_mean_mps"
            ],
            "per_trial_tracking_rmse_std_mps": data[
                "per_trial_tracking_rmse_std_mps"
            ],
        }
    report = {
        "figure": 9,
        "source": "Monte Carlo experiment",
        "cohort": "all trials regardless of outcome",
        "terrain_height_error_range_m": [
            float(min(data["offset_m"] for data in datasets)),
            float(max(data["offset_m"] for data in datasets)),
        ],
        "time_alignment": "absolute scenario time from scenario.yaml start",
        "plot_time_limits_s": [x_min, x_max],
        "plot_scope": "complete tick.csv duration for every trial",
        "first_lower_touchdown_scenario_time_mean_s": mean_touchdown_time,
        "first_lower_touchdown_scenario_time_std_s": float(
            np.std(touchdown_times, ddof=1)
        ),
        "first_lower_touchdown_sample_count": int(touchdown_times.size),
        "uncertainty_band": (
            "Across-trial 95% confidence interval of mean body-frame vx at each "
            "time (mean +/- 1.96 * sample standard deviation / sqrt(n))."
        ),
        "velocity_command_source": (
            "Across-trial mean of scenario.yaml command schedules aligned by "
            "absolute scenario time"
        ),
        "velocity_command_timewise_sample_count_min": int(np.min(command_count)),
        "velocity_command_timewise_sample_count_max": int(np.max(command_count)),
        "measured_velocity_source": "tick.csv meas_rbd21:24 and meas_rbd0:2",
        "measured_velocity_definition": (
            "world-frame measured base velocity rotated into the measured body frame"
        ),
        "methods": report_methods,
    }
    output.with_name(f"{output.stem}_selection.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    for stem in (
        "fig9_nominal_vcmd_vs_body_vx", "fig10_proposed_vcmd_vs_body_vx"
    ):
        for path in (
            out_dir / f"{stem}.png", out_dir / f"{stem}.pdf",
            out_dir / f"{stem}_selection.json",
        ):
            if path.is_file():
                path.unlink()
    return [output]


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

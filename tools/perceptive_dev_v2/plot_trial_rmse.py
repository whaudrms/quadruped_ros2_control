#!/usr/bin/env python3
"""Plot MPC-plan versus measured-state tracking RMSE for one trial."""

import argparse
import csv
import json
import sys
import textwrap
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from paper_plot_style import apply_figure_font_sizes

from terrain_descent_events import descent_times, load_or_detect_events


def load_tick(path: Path):
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        n_expected = len(header)
        t_col = header.index("t")
        opt_cols = [header.index(f"opt_x{i}") for i in range(24)]
        meas_cols = [header.index(f"meas_rbd{i}") for i in range(36)]
        rows = []
        for row in reader:
            if len(row) != n_expected:
                continue
            try:
                values = [float(value) for value in row]
            except ValueError:
                continue
            rows.append(
                [values[t_col]]
                + [values[index] for index in opt_cols]
                + [values[index] for index in meas_cols]
            )
    if not rows:
        raise RuntimeError(f"No valid samples in {path}")
    data = np.asarray(rows, dtype=float)
    return data[:, 0], data[:, 1:25], data[:, 25:61]


def wrap_angle(values: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(values), np.cos(values))


def rolling_rmse(component_error: np.ndarray, samples: int) -> np.ndarray:
    squared_mean = np.mean(np.square(component_error), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(squared_mean)))
    indices = np.arange(squared_mean.size)
    starts = np.maximum(0, indices - samples + 1)
    counts = indices - starts + 1
    moving_mean = (cumulative[indices + 1] - cumulative[starts]) / counts
    return np.sqrt(moving_mean)


def vector_rmse(component_error: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(component_error))))


def component_rmse(component_error: np.ndarray, names) -> dict:
    values = np.sqrt(np.mean(np.square(component_error), axis=0))
    return {name: float(value) for name, value in zip(names, values)}


def scenario_monitoring_range(trial_dir: Path):
    scenario_path = trial_dir / "scenario.yaml"
    if not scenario_path.is_file():
        # Legacy trials predate run_trial.py copying scenario.yaml into every
        # result directory. Resolve their source scenario by the name recorded
        # in result.json so all trials still use the intended monitoring window.
        result_path = trial_dir / "result.json"
        if result_path.is_file():
            try:
                scenario_name = json.loads(
                    result_path.read_text(encoding="utf-8")
                ).get("scenario")
            except (OSError, json.JSONDecodeError):
                scenario_name = None
            if scenario_name:
                for candidate in sorted((Path(__file__).parent / "scenarios").glob("*.yaml")):
                    try:
                        candidate_data = yaml.safe_load(
                            candidate.read_text(encoding="utf-8")
                        )
                    except (OSError, yaml.YAMLError):
                        continue
                    if candidate_data.get("name") == scenario_name:
                        scenario_path = candidate
                        break
        if not scenario_path.is_file():
            return None
    scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    entry_start = 0.0
    found_entry = False
    for step in scenario.get("steps", []):
        if step.get("name") == "enter_ocs2":
            found_entry = True
            break
        entry_start += float(step["duration"])
    if not found_entry:
        return None
    monitoring_start = float(scenario.get("monitoring_start_sec", entry_start))
    timeout = float(scenario["timeout_sec"])
    # tick.csv begins when StateOCS2 starts, approximately at enter_ocs2.
    return max(0.0, monitoring_start - entry_start), max(0.0, timeout - entry_start)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Plot rolling and overall WBC tracking RMSE for one trial"
    )
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--window-sec", type=float, default=0.25)
    parser.add_argument("--scope", choices=["monitoring", "all"], default="monitoring",
                        help="Use scenario monitoring window (default) or every tick")
    parser.add_argument("--start-sec", type=float, default=None,
                        help="Crop start relative to the first tick")
    parser.add_argument("--end-sec", type=float, default=None,
                        help="Crop end relative to the first tick")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)

    tick_path = args.trial_dir / "tick.csv"
    if not tick_path.is_file() or tick_path.stat().st_size == 0:
        print(f"missing or empty tick.csv: {tick_path}", file=sys.stderr)
        return 1
    if args.window_sec <= 0.0:
        parser.error("--window-sec must be positive")

    t, opt_x, meas_rbd = load_tick(tick_path)
    t_rel = t - t[0]
    terrain_events = load_or_detect_events(args.trial_dir)
    descent_start, descent_complete = descent_times(terrain_events)
    automatic_range = (
        scenario_monitoring_range(args.trial_dir) if args.scope == "monitoring" else None
    )
    start_sec = args.start_sec
    end_sec = args.end_sec
    if automatic_range is not None:
        if start_sec is None:
            start_sec = automatic_range[0]
        if end_sec is None:
            end_sec = automatic_range[1]
    mask = np.ones(t_rel.size, dtype=bool)
    if start_sec is not None:
        mask &= t_rel >= start_sec
    if end_sec is not None:
        mask &= t_rel <= end_sec
    if np.count_nonzero(mask) < 2:
        raise RuntimeError("Selected time range contains fewer than two samples")
    t_rel = t_rel[mask]
    opt_x = opt_x[mask]
    meas_rbd = meas_rbd[mask]

    # opt_x:    [v_com(3), w_c(3), base_pos(3), theta_zyx(3), q_joint(12)]
    # meas_rbd: [theta_zyx(3), base_pos(3), q_joint(12), ...]
    position_error = meas_rbd[:, 3:6] - opt_x[:, 6:9]
    orientation_error = wrap_angle(meas_rbd[:, 0:3] - opt_x[:, 9:12])
    joint_error = meas_rbd[:, 6:18] - opt_x[:, 12:24]

    dt = np.diff(t_rel)
    dt = dt[dt > 0.0]
    median_dt = float(np.median(dt)) if dt.size else 0.002
    window_samples = max(1, int(round(args.window_sec / median_dt)))

    orientation_error_deg = np.degrees(orientation_error)
    categories = [
        ("Base position", position_error, "m", ["x", "y", "z"]),
        ("Base orientation", orientation_error_deg, "deg", ["yaw", "pitch", "roll"]),
        ("Joint position", joint_error, "rad", [f"q{i}" for i in range(12)]),
    ]

    summary = {
        "trial": args.trial_dir.name,
        "tick_csv": str(tick_path.resolve()),
        "samples": int(t_rel.size),
        "time_start_sec": float(t_rel[0]),
        "time_end_sec": float(t_rel[-1]),
        "sample_rate_hz": float(1.0 / median_dt),
        "rolling_window_sec": args.window_sec,
        "scope": args.scope,
        "requested_start_sec": start_sec,
        "requested_end_sec": end_sec,
        "terrain_descent_start_sec": descent_start,
        "terrain_descent_complete_sec": descent_complete,
    }

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for axis, (label, error, unit, component_names) in zip(axes, categories):
        overall = vector_rmse(error)
        rolling = rolling_rmse(error, window_samples)
        key = label.lower().replace(" ", "_")
        summary[f"{key}_rmse_{unit}"] = overall
        summary[f"{key}_component_rmse_{unit}"] = component_rmse(error, component_names)
        axis.plot(t_rel, rolling, linewidth=1.0, color="tab:blue",
                  label=f"rolling RMSE ({args.window_sec:g} s)")
        axis.axhline(overall, color="tab:red", linestyle="--", linewidth=1.0,
                     label=f"overall RMSE = {overall:.5g} {unit}")
        if descent_start is not None and t_rel[0] <= descent_start <= t_rel[-1]:
            axis.axvline(
                descent_start,
                color="tab:purple",
                linestyle="--",
                linewidth=1.2,
                label=None,
            )
        if descent_complete is not None and t_rel[0] <= descent_complete <= t_rel[-1]:
            axis.axvline(
                descent_complete,
                color="tab:green",
                linestyle=":",
                linewidth=1.2,
                label=None,
            )
        if (
            descent_start is not None
            and descent_complete is not None
            and descent_complete >= descent_start
        ):
            axis.axvspan(descent_start, descent_complete, color="gray", alpha=0.08)
        axis.set_ylabel(f"{label}\nRMSE [{unit}]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("Time since first OCS2 tick [s]")
    fig.suptitle(
        "MPC plan vs measured-state tracking RMSE\n"
        + textwrap.fill(args.trial_dir.name, width=105),
        fontsize=11,
    )
    output_path = args.output or (args.trial_dir / "tracking_rmse.png")
    apply_figure_font_sizes(fig, output_path)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=130)
    plt.close(fig)
    summary_path = args.trial_dir / "tracking_rmse.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"saved {output_path}")
    print(f"saved {summary_path}")
    print(f"selected tick-relative range: {t_rel[0]:.3f} to {t_rel[-1]:.3f} s")
    print(
        "overall RMSE: "
        f"base position={summary['base_position_rmse_m']:.6f} m, "
        f"base orientation={summary['base_orientation_rmse_deg']:.6f} deg, "
        f"joint position={summary['joint_position_rmse_rad']:.6f} rad"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

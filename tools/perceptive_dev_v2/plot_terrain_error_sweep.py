#!/usr/bin/env python3
"""Analyze a Robust-ON terrain-perception-error sweep and generate its report."""

import argparse
import csv
import math
import sys
import warnings
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plot_all_results import (
    DASHBOARD_METRICS,
    attach_rmse,
    load_trials,
    metric_value,
    terrain_error_label,
)
from plot_wbc_tracking_error import load_tick
from terrain_descent_events import descent_times, load_or_detect_events
from wbc_plot_utils import command_window_in_tick_time, discover_trials


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RESULTS = SCRIPT_DIR / "results" / "robust_terrain_error_sweep_n5"
EXPECTED_OFFSETS_M = (-0.05, -0.03, 0.03, 0.05)
SUMMARY_METRICS = (
    *((metric, label) for metric, label, _higher in DASHBOARD_METRICS),
    ("terrain_descent_start_tick_sec", "Descent start [s]"),
    ("terrain_descent_complete_tick_sec", "Descent complete [s]"),
    ("terrain_descent_duration_sec", "Descent duration [s]"),
)


def offset_groups(records: list[dict]) -> dict[float, list[dict]]:
    groups = defaultdict(list)
    for record in records:
        if record.get("offset_m") is not None:
            groups[float(record["offset_m"])].append(record)
    return dict(sorted(groups.items()))


def attach_descent_events(records: list[dict]):
    for record in records:
        try:
            events = load_or_detect_events(record["trial_dir"])
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"[warn] descent detection failed for {record['trial']}: {exc}", file=sys.stderr)
            continue
        start, complete = descent_times(events)
        record["terrain_descent_start_tick_sec"] = start
        record["terrain_descent_complete_tick_sec"] = complete
        record["terrain_descent_duration_sec"] = events.get("descent", {}).get("duration_sec")


def values_for(records: list[dict], metric: str) -> list[float]:
    if metric == "success":
        return [100.0 if bool(record.get("success")) else 0.0 for record in records]
    values = [metric_value(record, metric) for record in records]
    return [float(value) for value in values if value is not None and np.isfinite(value)]


def mean_std(records: list[dict], metric: str):
    values = values_for(records, metric)
    if not values:
        return None, None, 0
    return float(np.mean(values)), float(np.std(values)), len(values)


def format_csv_value(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}"
    return value


def write_csv(path: Path, fields: list[str], rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: format_csv_value(row.get(field)) for field in fields})
    print(f"saved {path}")


def summary_row(label: str, records: list[dict]) -> dict:
    success_n = sum(bool(record.get("success")) for record in records)
    row = {
        "Terrain error": label,
        "Method": "Proposed (with robust phase)",
        "N trials": len(records),
        "Success [n]": success_n,
        "Success [%]": 100.0 * success_n / len(records) if records else None,
    }
    for metric, metric_label in SUMMARY_METRICS:
        if metric == "success":
            continue
        mean, std, n = mean_std(records, metric)
        row[f"{metric_label} mean"] = mean
        row[f"{metric_label} std"] = std
        row[f"{metric_label} n"] = n
    return row


def write_summary_tables(records: list[dict], output_dir: Path):
    groups = offset_groups(records)
    metric_fields = []
    for metric, label in SUMMARY_METRICS:
        if metric != "success":
            metric_fields.extend((f"{label} mean", f"{label} std", f"{label} n"))
    fields = [
        "Terrain error", "Method", "N trials", "Success [n]", "Success [%]", *metric_fields
    ]

    all_rows = [summary_row(terrain_error_label(offset), grouped) for offset, grouped in groups.items()]
    all_rows.append(summary_row("Overall", records))
    write_csv(output_dir / "terrain_error_summary_all_trials.csv", fields, all_rows)

    successful = [record for record in records if bool(record.get("success"))]
    successful_groups = offset_groups(successful)
    successful_rows = [
        summary_row(terrain_error_label(offset), successful_groups.get(offset, []))
        for offset in groups
    ]
    successful_rows.append(summary_row("Overall", successful))
    write_csv(
        output_dir / "terrain_error_summary_successful_only.csv", fields, successful_rows
    )

    trial_fields = [
        "trial", "Terrain error", "offset_m", "run", "Method", "success", "fall_reason",
        "time_to_failure", "duration_executed",
        *(label for _metric, label, _higher in DASHBOARD_METRICS),
        "Descent start [s]", "Descent complete [s]", "Descent duration [s]",
    ]
    trial_rows = []
    for record in sorted(records, key=lambda item: (item["offset_m"], item.get("run") or 0, item["trial"])):
        row = {
            "trial": record["trial"],
            "Terrain error": terrain_error_label(float(record["offset_m"])),
            "offset_m": record["offset_m"],
            "run": record.get("run"),
            "Method": "Proposed (with robust phase)",
            "success": record.get("success"),
            "fall_reason": record.get("fall_reason"),
            "time_to_failure": record.get("time_to_failure"),
            "duration_executed": record.get("duration_executed"),
            "Descent start [s]": record.get("terrain_descent_start_tick_sec"),
            "Descent complete [s]": record.get("terrain_descent_complete_tick_sec"),
            "Descent duration [s]": record.get("terrain_descent_duration_sec"),
        }
        for metric, label, _higher in DASHBOARD_METRICS:
            row[label] = metric_value(record, metric)
        trial_rows.append(row)
    write_csv(output_dir / "terrain_error_trials.csv", trial_fields, trial_rows)

def plot_response_dashboard(records: list[dict], output: Path):
    groups = offset_groups(records)
    offsets = np.asarray(list(groups)) * 100.0
    columns = 3
    rows = int(np.ceil(len(DASHBOARD_METRICS) / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(17, 4 * rows), squeeze=False)
    rng = np.random.default_rng(23)
    for axis, (metric, label, _higher) in zip(axes.flat, DASHBOARD_METRICS):
        title, unit = label.rsplit(" [", 1)
        means, stds = [], []
        for grouped in groups.values():
            mean, std, _n = mean_std(grouped, metric)
            means.append(mean if mean is not None else np.nan)
            stds.append(std if std is not None else 0.0)
        axis.errorbar(offsets, means, yerr=stds, color="tab:blue", marker="o", capsize=4)
        for offset, grouped in groups.items():
            for record in grouped:
                value = values_for([record], metric)
                if not value:
                    continue
                failed = not bool(record.get("success"))
                axis.scatter(
                    offset * 100.0 + rng.uniform(-0.09, 0.09),
                    value[0],
                    color="crimson" if failed else "black",
                    marker="x" if failed else "o",
                    s=34,
                    zorder=4,
                )
        axis.axvline(0.0, color="gray", linestyle="--", linewidth=1.0)
        axis.set_title(title)
        axis.set_ylabel(unit.rstrip("]"))
        axis.set_xlabel("Terrain perception error [cm]")
        axis.set_xticks(offsets)
        axis.grid(True, alpha=0.3)
        if metric == "success":
            axis.set_ylim(-5, 105)
    for axis in axes.flat[len(DASHBOARD_METRICS):]:
        axis.set_visible(False)
    fig.suptitle(
        "Robust-phase terrain-error response — mean ± population std; dots: trials; red x: failure",
        fontsize=14,
        fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=140)
    plt.close(fig)
    print(f"saved {output}")


def wilson_interval(successes: int, total: int, z: float = 1.96):
    if total <= 0:
        return np.nan, np.nan
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denominator
    return 100.0 * (center - half), 100.0 * (center + half)


def plot_success_tolerance(records: list[dict], output: Path):
    groups = offset_groups(records)
    offsets = np.asarray(list(groups)) * 100.0
    success_rates, lower, upper, progress_mean, progress_std = [], [], [], [], []
    for grouped in groups.values():
        successes = sum(bool(record.get("success")) for record in grouped)
        rate = 100.0 * successes / len(grouped)
        lo, hi = wilson_interval(successes, len(grouped))
        mean, std, _n = mean_std(grouped, "body_frame_forward_progress")
        success_rates.append(rate)
        lower.append(rate - lo)
        upper.append(hi - rate)
        progress_mean.append(mean)
        progress_std.append(std)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].errorbar(
        offsets, success_rates, yerr=np.asarray([lower, upper]), marker="o",
        color="tab:blue", capsize=5, label="success rate with Wilson 95% CI",
    )
    axes[0].set_ylabel("Success [%]")
    axes[0].set_ylim(-5, 105)
    axes[0].legend()
    axes[1].errorbar(
        offsets, progress_mean, yerr=progress_std, marker="o",
        color="tab:green", capsize=5, label="progress mean ± population std",
    )
    axes[1].set_ylabel("Progress [m]")
    axes[1].set_xlabel("Terrain perception error [cm]")
    axes[1].legend()
    for axis in axes:
        axis.axvline(0.0, color="gray", linestyle="--", linewidth=1.0)
        axis.grid(True, alpha=0.3)
    axes[1].set_xticks(offsets)
    fig.suptitle("Robust-phase empirical tolerance to terrain perception error", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=140)
    plt.close(fig)
    print(f"saved {output}")


def plot_descent_timing(records: list[dict], output: Path):
    groups = offset_groups(records)
    offsets = np.asarray(list(groups)) * 100.0
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for metric, label, color in (
        ("terrain_descent_start_tick_sec", "first lower touchdown", "tab:purple"),
        ("terrain_descent_complete_tick_sec", "descent complete", "tab:green"),
    ):
        values = [mean_std(grouped, metric) for grouped in groups.values()]
        axes[0].errorbar(
            offsets,
            [value[0] for value in values],
            yerr=[value[1] for value in values],
            marker="o", capsize=4, label=label, color=color,
        )
    duration = [mean_std(grouped, "terrain_descent_duration_sec") for grouped in groups.values()]
    axes[1].errorbar(
        offsets,
        [value[0] for value in duration],
        yerr=[value[1] for value in duration],
        marker="o", capsize=4, color="tab:orange", label="descent duration",
    )
    axes[0].set_ylabel("Time since first OCS2 tick [s]")
    axes[1].set_ylabel("Duration [s]")
    axes[1].set_xlabel("Terrain perception error [cm]")
    for axis in axes:
        axis.axvline(0.0, color="gray", linestyle="--", linewidth=1.0)
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[1].set_xticks(offsets)
    fig.suptitle("Terrain-descent timing response", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=140)
    plt.close(fig)
    print(f"saved {output}")


def load_wbc_series(trial: dict):
    loaded = load_tick(trial["tick_path"])
    if loaded is None:
        return None
    time, opt_x, measured, _mode = loaded
    if time.size < 5:
        return None
    time = time - time[0]
    start, end = command_window_in_tick_time(trial, float(time[-1]))
    if end <= start:
        return None
    position = measured[:, 3:6] - opt_x[:, 6:9]
    orientation_raw = measured[:, 0:3] - opt_x[:, 9:12]
    orientation = np.arctan2(np.sin(orientation_raw), np.cos(orientation_raw))
    joint = measured[:, 6:18] - opt_x[:, 12:24]
    magnitudes = np.column_stack(
        (
            np.linalg.norm(position, axis=1),
            np.linalg.norm(np.degrees(orientation), axis=1),
            np.linalg.norm(joint, axis=1),
        )
    )
    event_start, event_complete = descent_times(load_or_detect_events(trial["trial_dir"]))
    return {
        "time": time,
        "start": start,
        "end": end,
        "duration": end - start,
        "magnitudes": magnitudes,
        "position": position,
        "orientation": orientation,
        "descent_start": event_start - start if event_start is not None else None,
        "descent_complete": event_complete - start if event_complete is not None else None,
    }


def aggregate_wbc(series: list[dict], dt: float = 0.01):
    if not series:
        return None
    duration = max(item["duration"] for item in series)
    grid = np.arange(0.0, duration + 1e-9, dt)
    stacks = {"magnitudes": [], "position": [], "orientation": []}
    for item in series:
        query = grid + item["start"]
        valid = (query >= item["time"][0]) & (query <= min(item["time"][-1], item["end"]))
        for key in stacks:
            source = item[key]
            interpolated = np.full((grid.size, source.shape[1]), np.nan)
            for component in range(source.shape[1]):
                interpolated[valid, component] = np.interp(
                    query[valid], item["time"], source[:, component]
                )
            stacks[key].append(interpolated)
    output = {"time": grid, "duration": duration, "n": len(series)}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        for key, values in stacks.items():
            values = np.asarray(values)
            output[f"{key}_mean"] = np.nanmean(values, axis=0)
            output[f"{key}_std"] = np.nanstd(values, axis=0)
    for event in ("descent_start", "descent_complete"):
        values = [item[event] for item in series if item[event] is not None and 0 <= item[event] <= duration]
        output[f"{event}_mean"] = float(np.mean(values)) if values else None
    return output


def build_wbc_aggregates(results_dir: Path, successful_only: bool):
    trials = discover_trials(results_dir)
    groups = defaultdict(list)
    for trial in trials:
        if successful_only and not bool(trial.get("result", {}).get("success")):
            continue
        record = load_wbc_series(trial)
        if record is not None:
            groups[trial["offset_m"]].append(record)
    return {offset: aggregate_wbc(series) for offset, series in sorted(groups.items())}


def plot_wbc_heatmap(aggregates: dict[float, dict], cohort: str, output: Path):
    if not aggregates:
        return
    offsets = np.asarray(list(aggregates)) * 100.0
    duration = max(data["duration"] for data in aggregates.values())
    grid = np.arange(0.0, duration + 1e-9, 0.01)
    metric_meta = (
        (0, "Base position error norm [m]"),
        (1, "Base orientation error norm [deg]"),
        (2, "Joint position error norm [rad]"),
    )
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for axis, (metric_index, title) in zip(axes, metric_meta):
        rows = []
        for data in aggregates.values():
            source_t = data["time"]
            source = data["magnitudes_mean"][:, metric_index]
            values = np.full(grid.shape, np.nan)
            valid = grid <= source_t[-1]
            values[valid] = np.interp(grid[valid], source_t, source)
            rows.append(values)
        matrix = np.asarray(rows)
        finite = matrix[np.isfinite(matrix)]
        vmax = float(np.percentile(finite, 98.0)) if finite.size else None
        mesh = axis.pcolormesh(
            grid, offsets, matrix, shading="nearest", cmap="viridis", vmin=0.0, vmax=vmax
        )
        axis.set_ylabel("Error [cm]")
        axis.set_yticks(offsets)
        axis.set_title(f"{title} (color capped at 98th percentile)")
        fig.colorbar(mesh, ax=axis, pad=0.01)
    axes[-1].set_xlabel("Command-active time [s]")
    fig.suptitle(f"WBC tracking-error heatmap — {cohort}", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=140)
    plt.close(fig)
    print(f"saved {output}")


def plot_wbc_deviation_traces(aggregates: dict[float, dict], cohort: str, output: Path):
    if not aggregates:
        return
    offsets = list(aggregates)
    fig, axes = plt.subplots(2, len(offsets), figsize=(4.5 * len(offsets), 7), squeeze=False)
    names = (("x", "y", "z"), ("yaw", "pitch", "roll"))
    keys = ("position", "orientation")
    units = ("Position deviation [m]", "Orientation deviation [rad]")
    colors = ("tab:blue", "tab:orange", "tab:green")
    row_limits = []
    for key in keys:
        extents = []
        for data in aggregates.values():
            extents.append(np.abs(data[f"{key}_mean"]) + data[f"{key}_std"])
        finite = np.concatenate([extent[np.isfinite(extent)] for extent in extents])
        row_limits.append(max(float(np.max(finite)) * 1.05, 1e-9))

    for column, (offset, data) in enumerate(aggregates.items()):
        for row, (key, component_names, ylabel) in enumerate(zip(keys, names, units)):
            axis = axes[row, column]
            mean, std = data[f"{key}_mean"], data[f"{key}_std"]
            for component, (name, color) in enumerate(zip(component_names, colors)):
                axis.plot(data["time"], mean[:, component], color=color, label=name, linewidth=1.0)
                axis.fill_between(
                    data["time"], mean[:, component] - std[:, component],
                    mean[:, component] + std[:, component], color=color, alpha=0.10,
                )
            if data.get("descent_start_mean") is not None:
                axis.axvline(data["descent_start_mean"], color="tab:purple", linestyle="--")
            if data.get("descent_complete_mean") is not None:
                axis.axvline(data["descent_complete_mean"], color="tab:green", linestyle=":")
            axis.axhline(0.0, color="black", linewidth=0.5)
            axis.set_ylim(-row_limits[row], row_limits[row])
            axis.grid(True, alpha=0.3)
            if column == 0:
                axis.set_ylabel(ylabel)
                axis.legend(fontsize=8)
            if row == 0:
                axis.set_title(f"{terrain_error_label(offset)} (n={data['n']})")
            if row == 1:
                axis.set_xlabel("Command-active time [s]")
    fig.suptitle(
        f"WBC deviation across terrain errors — {cohort} — shared row scales",
        fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output, dpi=130)
    plt.close(fig)
    print(f"saved {output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--skip-wbc", action="store_true")
    args = parser.parse_args(argv)

    results_dir = args.results_dir.resolve()
    output_dir = (args.out_dir or results_dir / "all_visualizations").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    records, skipped = load_trials(results_dir)
    if not records:
        raise RuntimeError(f"No valid trials found in {results_dir}")
    if any(record.get("robust") != "ON" for record in records):
        raise RuntimeError("Terrain-error sweep analysis requires Robust ON trials only")
    offsets = tuple(sorted(offset_groups(records)))
    missing = [offset for offset in EXPECTED_OFFSETS_M if offset not in offsets]
    if missing:
        print(f"[warn] incomplete sweep; missing offsets: {missing}", file=sys.stderr)
    if skipped:
        print(f"[warn] skipped trial directories: {len(skipped)}", file=sys.stderr)

    attach_rmse(records)
    attach_descent_events(records)
    write_summary_tables(records, output_dir)
    plot_response_dashboard(records, output_dir / "terrain_error_response_dashboard.png")
    plot_success_tolerance(records, output_dir / "terrain_error_success_tolerance.png")
    plot_descent_timing(records, output_dir / "terrain_error_descent_timing.png")

    if not args.skip_wbc:
        for successful_only, token, title in (
            (False, "all_trials", "all trials"),
            (True, "successful_only", "successful trials only"),
        ):
            aggregates = build_wbc_aggregates(results_dir, successful_only)
            plot_wbc_heatmap(
                aggregates, title, output_dir / f"terrain_error_wbc_heatmap_{token}.png"
            )
            plot_wbc_deviation_traces(
                aggregates, title, output_dir / f"terrain_error_wbc_deviation_{token}.png"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

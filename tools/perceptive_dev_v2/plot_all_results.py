#!/usr/bin/env python3
"""Generate a complete gallery and aggregate dashboard for every saved trial."""

import argparse
import csv
import html
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
DEFAULT_RESULTS = ROOT / "results"
DASHBOARD_METRICS = (
    ("success", "Success [%]", True),
    ("body_frame_forward_progress", "Progress [m]", True),
    ("body_frame_lateral_progress", "Lateral drift [m]", False),
    ("roll_rms_deg", "Roll RMS [deg]", False),
    ("pitch_rms_deg", "Pitch RMS [deg]", False),
    ("yaw_rms_deg", "Yaw RMS [deg]", False),
    ("base_position_rmse_m", "Base pos. RMSE [m]", False),
    ("base_orientation_rmse_deg", "Base ori. RMSE [deg]", False),
    ("joint_position_rmse_rad", "Joint RMSE [rad]", False),
)


def infer_metadata(trial_dir: Path, result: dict) -> dict:
    name = trial_dir.name
    config = {}
    config_path = trial_dir / "run_config.json"
    if config_path.is_file():
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass

    effective = config.get("effective_task_parameters", {}).get("robustPhase", {})
    enabled = effective.get("enabled")
    if enabled is None:
        robust = "ON" if re.search(r"(?:^|_)ON(?:_|$)|robON", name) else "OFF"
    else:
        robust = "ON" if str(enabled).lower() == "true" else "OFF"

    offset = config.get("terrain_z_offset")
    if offset is None:
        match = re.search(r"off([MP])(\d+)", name)
        if match:
            magnitude = int(match.group(2)) / (10 ** len(match.group(2)))
            offset = magnitude if match.group(1) == "P" else -magnitude
        elif re.search(r"(?:^|_)off0(?:_|$)", name):
            offset = 0.0

    run_match = re.search(r"_run(\d+)(?:_|$)", name)
    run = int(run_match.group(1)) if run_match else None
    splice = effective.get("enable_splice")
    if splice is None:
        splice = "OFF" if "nosplice" in name.lower() else "unknown"
    else:
        splice = "ON" if str(splice).lower() == "true" else "OFF"

    return {
        "trial": name,
        "offset_m": float(offset) if offset is not None else None,
        "robust": robust,
        "splice": splice,
        "run": run,
        "scenario": result.get("scenario", config.get("scenario", "unknown")),
    }


def run_plot(script: Path, trial_dir: Path) -> tuple[bool, str]:
    proc = subprocess.run(
        [sys.executable, str(script), str(trial_dir)],
        capture_output=True,
        text=True,
        timeout=240.0,
    )
    detail = (proc.stdout if proc.returncode == 0 else proc.stderr or proc.stdout).strip()
    return proc.returncode == 0, detail


def load_trials(results_dir: Path) -> tuple[list[dict], list[dict]]:
    valid = []
    skipped = []
    for trial_dir in sorted(path for path in results_dir.iterdir() if path.is_dir()):
        if trial_dir.name == "all_visualizations":
            continue
        result_path = trial_dir / "result.json"
        tick_path = trial_dir / "tick.csv"
        if not result_path.is_file() or not tick_path.is_file() or tick_path.stat().st_size == 0:
            skipped.append({"trial": trial_dir.name, "reason": "missing result.json or tick.csv"})
            continue
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            skipped.append({"trial": trial_dir.name, "reason": f"invalid result.json: {exc}"})
            continue
        record = infer_metadata(trial_dir, result)
        record.update(result)
        record["trial_dir"] = trial_dir
        valid.append(record)
    return valid, skipped


def attach_rmse(records: list[dict]):
    for record in records:
        path = record["trial_dir"] / "tracking_rmse.json"
        if not path.is_file():
            continue
        try:
            rmse = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        record["base_position_rmse_m"] = rmse.get("base_position_rmse_m")
        record["base_orientation_rmse_deg"] = rmse.get("base_orientation_rmse_deg")
        record["joint_position_rmse_rad"] = rmse.get("joint_position_rmse_rad")


def attach_terrain_descent(records: list[dict]):
    for record in records:
        path = record["trial_dir"] / "terrain_descent_events.json"
        if not path.is_file():
            continue
        try:
            descent = json.loads(path.read_text(encoding="utf-8")).get("descent", {})
        except (OSError, json.JSONDecodeError):
            continue
        start = descent.get("first_lower_touchdown")
        complete = descent.get("last_hind_lower_touchdown")
        record["terrain_descent_start_tick_sec"] = (
            start.get("tick_relative_sec") if start else None
        )
        record["terrain_descent_complete_tick_sec"] = (
            complete.get("tick_relative_sec") if complete else None
        )
        record["terrain_descent_duration_sec"] = descent.get("duration_sec")


def group_label(record: dict) -> str:
    offset = record.get("offset_m")
    offset_label = "unknown" if offset is None else f"{offset:+.2f}"
    return f"dz={offset_label}\n{record['robust']}"


def group_key(record: dict):
    offset = record.get("offset_m")
    return (float("inf") if offset is None else offset, 0 if record["robust"] == "ON" else 1)


def metric_value(record: dict, key: str):
    value = record.get(key)
    if value is None:
        return None
    value = float(value)
    return abs(value) if key == "body_frame_lateral_progress" else value


def plot_dashboard(records: list[dict], output: Path):
    groups = defaultdict(list)
    for record in records:
        groups[group_key(record)].append(record)
    keys = sorted(groups)
    labels = [group_label(groups[key][0]) for key in keys]
    colors = ["tab:blue" if groups[key][0]["robust"] == "ON" else "tab:orange" for key in keys]

    fig, axes = plt.subplots(3, 3, figsize=(17, 12))
    rng = np.random.default_rng(7)
    for axis, (metric, column_label, _higher_is_better) in zip(axes.flat, DASHBOARD_METRICS):
        title, unit = column_label.rsplit(" [", 1)
        unit = unit.rstrip("]")
        means, stds = [], []
        values_by_group = []
        for key in keys:
            if metric == "success":
                values = [100.0 if bool(r.get("success")) else 0.0 for r in groups[key]]
            else:
                values = [metric_value(r, metric) for r in groups[key]]
                values = [value for value in values if value is not None and np.isfinite(value)]
            values_by_group.append(values)
            means.append(float(np.mean(values)) if values else np.nan)
            stds.append(float(np.std(values)) if len(values) > 1 else 0.0)
        x = np.arange(len(keys))
        axis.bar(x, means, yerr=stds, color=colors, alpha=0.72, capsize=4)
        for index, (key, values) in enumerate(zip(keys, values_by_group)):
            for record, value in zip(groups[key], values):
                jitter = float(rng.uniform(-0.09, 0.09))
                failed = not bool(record.get("success"))
                axis.scatter(
                    index + jitter,
                    value,
                    color="crimson" if failed else "black",
                    marker="x" if failed else "o",
                    s=38,
                    zorder=4,
                )
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.set_xticks(x, labels)
        axis.grid(axis="y", alpha=0.3)
        if metric == "success":
            axis.set_ylim(0, 110)
    fig.suptitle(
        "All perceptive robust-phase trials — bars: mean +/- population std, dots: each run\n"
        "red x: failed/fell trial; lower is better except success and forward progress",
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output, dpi=140)
    plt.close(fig)


def dashboard_metric_mean(records: list[dict], metric: str):
    if metric == "success":
        values = [100.0 if bool(record.get("success")) else 0.0 for record in records]
    else:
        values = [metric_value(record, metric) for record in records]
        values = [value for value in values if value is not None and np.isfinite(value)]
    return float(np.mean(values)) if values else None


def terrain_error_label(offset_m: float) -> str:
    centimeters = offset_m * 100.0
    if abs(centimeters) < 1e-9:
        return "0 cm"
    if abs(centimeters - round(centimeters)) < 1e-9:
        return f"{round(centimeters):+d} cm"
    return f"{centimeters:+g} cm"


def dashboard_method_row(
    terrain_error: str, method: str, records: list[dict], robust: str
) -> dict:
    method_records = [record for record in records if record.get("robust") == robust]
    return {
        "Terrain error": terrain_error,
        "Method": method,
        **{
            label: dashboard_metric_mean(method_records, metric)
            for metric, label, _higher_is_better in DASHBOARD_METRICS
        },
    }


def write_dashboard_comparison_table(records: list[dict], output: Path):
    """Write terrain-separated and overall Baseline/Proposed dashboard means."""
    rows = []
    offsets = sorted(
        {float(record["offset_m"]) for record in records if record.get("offset_m") is not None}
    )
    for offset in offsets:
        offset_records = [
            record
            for record in records
            if record.get("offset_m") is not None
            and abs(float(record["offset_m"]) - offset) < 1e-9
        ]
        label = terrain_error_label(offset)
        rows.append(dashboard_method_row(label, "Baseline", offset_records, "OFF"))
        rows.append(dashboard_method_row(label, "Proposed", offset_records, "ON"))

    baseline = dashboard_method_row("Overall", "Baseline", records, "OFF")
    proposed = dashboard_method_row("Overall", "Proposed", records, "ON")
    rows.extend((baseline, proposed))
    improvement = {"Terrain error": "Overall", "Method": "Improvement (%)"}
    for _metric, label, higher_is_better in DASHBOARD_METRICS:
        baseline_value = baseline[label]
        proposed_value = proposed[label]
        if baseline_value is None or proposed_value is None or abs(baseline_value) < 1e-12:
            improvement[label] = None
            continue
        difference = (
            proposed_value - baseline_value
            if higher_is_better
            else baseline_value - proposed_value
        )
        improvement[label] = difference / abs(baseline_value) * 100.0
    rows.append(improvement)

    fields = [
        "Terrain error",
        "Method",
        *(label for _metric, label, _higher in DASHBOARD_METRICS),
    ]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    field: (
                        row.get(field)
                        if field in {"Terrain error", "Method"} or row.get(field) is None
                        else f"{row[field]:.6f}"
                    )
                    for field in fields
                }
            )


def write_summary(records: list[dict], skipped: list[dict], output_dir: Path):
    fields = [
        "trial", "offset_m", "robust", "splice", "run", "scenario", "success",
        "fall_reason", "time_to_failure", "duration_executed",
        "body_frame_forward_progress", "body_frame_lateral_progress",
        "roll_rms_deg", "pitch_rms_deg", "yaw_rms_deg", "min_base_z", "base_z_std",
        "base_position_rmse_m", "base_orientation_rmse_deg", "joint_position_rmse_rad",
        "terrain_descent_start_tick_sec", "terrain_descent_complete_tick_sec",
        "terrain_descent_duration_sec",
    ]
    with (output_dir / "all_trials_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field) for field in fields})
    serializable = [{field: record.get(field) for field in fields} for record in records]
    (output_dir / "all_trials_summary.json").write_text(
        json.dumps({"trials": serializable, "skipped": skipped}, indent=2),
        encoding="utf-8",
    )


def write_gallery(records: list[dict], skipped: list[dict], output_dir: Path, aggregate_images):
    cards = []
    for record in records:
        trial = record["trial"]
        status = "PASS" if record.get("success") else f"FAIL: {record.get('fall_reason')}"
        images = []
        for filename in ("tracking_rmse.png", "robust_phase_foot_z.png"):
            if (record["trial_dir"] / filename).is_file():
                images.append(f'<img loading="lazy" src="../{html.escape(trial)}/{filename}">')
        cards.append(
            f'<section><h2>{html.escape(trial)}</h2>'
            f'<p><b>{status}</b> | offset={record.get("offset_m")} m | '
            f'robust={record.get("robust")} | run={record.get("run")}</p>'
            + "".join(images) + "</section>"
        )
    aggregate_html = "".join(
        f'<section><h2>{html.escape(path.stem)}</h2><img src="{html.escape(path.name)}"></section>'
        for path in aggregate_images
    )
    skipped_html = "".join(
        f'<li>{html.escape(item["trial"])}: '
        f'{html.escape(item.get("reason") or item.get("error") or "unknown error")}</li>'
        for item in skipped
    ) or "<li>None</li>"
    selection_report = output_dir / "wbc_max_contrast_selection.json"
    selection_link = (
        '<a href="wbc_max_contrast_selection.json">Max-contrast selected trials (JSON)</a>'
        if selection_report.is_file()
        else ""
    )
    dashboard_table = output_dir / "all_trials_dashboard_comparison.csv"
    dashboard_table_link = (
        '<a href="all_trials_dashboard_comparison.csv">Dashboard comparison table (CSV)</a>'
        if dashboard_table.is_file()
        else ""
    )
    sweep_table_links = "".join(
        f'<a href="{html.escape(path.name)}">{html.escape(path.stem)} (CSV)</a>'
        for path in sorted(output_dir.glob("terrain_error_*.csv"))
    )
    page = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>GO2 experiment visualizations</title>
<style>
body{{font-family:sans-serif;margin:24px;background:#f5f6f8;color:#1c2430}}
section{{background:white;padding:16px;margin:18px 0;border-radius:8px;box-shadow:0 1px 5px #ccd}}
img{{max-width:100%;height:auto;margin:8px 0;border:1px solid #ddd}}
a{{margin-right:18px}} h1,h2{{overflow-wrap:anywhere}}
</style></head><body>
<h1>GO2 perceptive robust-phase experiment gallery</h1>
<p>{len(records)} valid trials; {len(skipped)} skipped.</p>
<p><a href="all_trials_summary.csv">CSV summary</a>
<a href="all_trials_summary.json">JSON summary</a>
{dashboard_table_link}
{sweep_table_links}
{selection_link}</p>
{aggregate_html}
<h2>Skipped/incomplete trials</h2><ul>{skipped_html}</ul>
<h1>Per-trial plots</h1>{''.join(cards)}
</body></html>"""
    (output_dir / "index.html").write_text(page, encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--skip-individual", action="store_true")
    parser.add_argument("--skip-foot-z", action="store_true")
    parser.add_argument("--skip-wbc", action="store_true", help="Skip aggregate WBC plots.")
    args = parser.parse_args(argv)
    requested_results_name = args.results_dir.name
    results_dir = args.results_dir.resolve()
    output_dir = results_dir / "all_visualizations"
    output_dir.mkdir(parents=True, exist_ok=True)
    records, skipped = load_trials(results_dir)
    if not records:
        raise RuntimeError(f"No valid trials found in {results_dir}")
    robust_states = {record.get("robust") for record in records}
    terrain_offsets = {
        float(record["offset_m"])
        for record in records
        if record.get("offset_m") is not None
    }
    terrain_sweep = robust_states == {"ON"} and (
        "terrain_error_sweep" in requested_results_name
        or "terrain_error_sweep" in results_dir.name
        or len(terrain_offsets) >= 2
    )

    failures = []
    if not args.skip_individual:
        for index, record in enumerate(records, 1):
            trial_dir = record["trial_dir"]
            print(f"[{index}/{len(records)}] {trial_dir.name}", flush=True)
            ok, detail = run_plot(ROOT / "plot_trial_rmse.py", trial_dir)
            if not ok:
                failures.append({"trial": trial_dir.name, "plot": "tracking_rmse", "error": detail})
            if not args.skip_foot_z:
                ok, detail = run_plot(ROOT / "plot_robust_phase.py", trial_dir)
                if not ok:
                    failures.append({"trial": trial_dir.name, "plot": "robust_phase_foot_z", "error": detail})

    attach_rmse(records)
    attach_terrain_descent(records)
    dashboard = output_dir / "all_trials_dashboard.png"
    plot_dashboard(records, dashboard)
    dashboard_comparison = output_dir / "all_trials_dashboard_comparison.csv"
    if terrain_sweep:
        if dashboard_comparison.is_file():
            dashboard_comparison.unlink()
    else:
        write_dashboard_comparison_table(records, dashboard_comparison)
    write_summary(records, skipped + failures, output_dir)

    aggregate_images = [dashboard]
    if terrain_sweep:
        for pattern in ("wbc_tracking_error*.png", "wbc_deviation_components*.png"):
            for stale_output in output_dir.glob(pattern):
                stale_output.unlink()
        selection_report = output_dir / "wbc_max_contrast_selection.json"
        if selection_report.is_file():
            selection_report.unlink()
        for pattern in ("terrain_error_*.png", "terrain_error_*.csv"):
            for stale_output in output_dir.glob(pattern):
                stale_output.unlink()
        command = [
            sys.executable,
            str(ROOT / "plot_terrain_error_sweep.py"),
            "--results-dir",
            str(results_dir),
            "--out-dir",
            str(output_dir),
        ]
        if args.skip_wbc:
            command.append("--skip-wbc")
        proc = subprocess.run(command, capture_output=True, text=True, timeout=480.0)
        if proc.returncode != 0:
            failures.append(
                {
                    "trial": "aggregate",
                    "plot": "plot_terrain_error_sweep.py",
                    "error": proc.stderr.strip() or proc.stdout.strip(),
                }
            )
        aggregate_images.extend(sorted(output_dir.glob("terrain_error_*.png")))
    elif not args.skip_wbc:
        # WBC aggregate images are fully reproducible. Remove old naming schemes
        # and stale cohort outputs before generating the current complete set.
        for pattern in ("wbc_tracking_error*.png", "wbc_deviation_components*.png"):
            for stale_image in output_dir.glob(pattern):
                stale_image.unlink()
        selection_report = output_dir / "wbc_max_contrast_selection.json"
        if selection_report.is_file():
            selection_report.unlink()
        for script_name in ("plot_wbc_tracking_error.py", "plot_wbc_deviation_components.py"):
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / script_name),
                    "--results-dir",
                    str(results_dir),
                    "--out-dir",
                    str(output_dir),
                ],
                capture_output=True,
                text=True,
                timeout=240.0,
            )
            if proc.returncode != 0:
                failures.append(
                    {"trial": "aggregate", "plot": script_name, "error": proc.stderr.strip()}
                )
        aggregate_images.extend(sorted(output_dir.glob("wbc_tracking_error_*.png")))
        aggregate_images.extend(sorted(output_dir.glob("wbc_deviation_components_*.png")))

    # Rewrite after aggregate generation so late failures are included.
    write_summary(records, skipped + failures, output_dir)
    write_gallery(records, skipped + failures, output_dir, aggregate_images)
    print(f"saved gallery: {output_dir / 'index.html'}")
    print(f"valid trials: {len(records)}, skipped/plot failures: {len(skipped) + len(failures)}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())

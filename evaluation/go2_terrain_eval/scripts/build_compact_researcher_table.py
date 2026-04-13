#!/usr/bin/env python3

import csv
from pathlib import Path


INPUT_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_summary.csv")
OUTPUT_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_compact_summary.csv")
OUTPUT_TXT = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_compact_summary.txt")

KEEP_COLUMNS = [
    "terrain_height_cm",
    "contact_timing_bias_s",
    "n_runs",
    "n_runs_total",
    "n_runs_excluded",
    "outcome_majority",
    "success_rate",
    "stable_rate",
    "unstable_rate",
    "fail_rate",
    "startup_valid_rate",
    "late_stabilization_rate",
    "end_only_stabilized_count",
    "end_only_stabilized_rate_total",
    "end_only_stabilized_rate_excluded",
    "forward_progress_mean",
    "forward_progress_std",
    "lateral_progress_mean",
    "lateral_progress_std",
    "roll_rms_mean",
    "roll_rms_std",
    "pitch_rms_mean",
    "pitch_rms_std",
    "yaw_rms_mean",
    "yaw_rms_std",
    "duration_mean",
    "duration_std",
    "time_to_failure_mean",
    "time_to_failure_std",
    "min_base_z_mean",
    "min_base_z_std",
    "start_z_mean",
    "end_z_mean",
    "delta_z_mean",
    "obs_state_norm_mean",
    "opt_state_norm_mean",
    "obs_input_norm_mean",
    "opt_input_norm_mean",
    "planned_mode_mode",
    "obs_mode_mode",
]


def as_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def fmt(value):
    if value is None or value == "":
        return "-"
    try:
        return f"{float(value):.4f}"
    except ValueError:
        return str(value)


def load_rows():
    with INPUT_CSV.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    rows.sort(key=lambda r: (as_float(r["terrain_height_cm"]) or 0.0, as_float(r["contact_timing_bias_s"]) or 0.0))
    return rows


def write_compact_csv(rows):
    with OUTPUT_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=KEEP_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in KEEP_COLUMNS})


def write_compact_txt(rows):
    lines = []
    current_terrain = None
    for row in rows:
        terrain = row["terrain_height_cm"]
        if terrain != current_terrain:
            if lines:
                lines.append("")
            lines.append(f"terrain {terrain} cm")
            current_terrain = terrain
        lines.append(
            "  d={d}s | n={n}/{nt} | outcome={outcome} | success={succ} | stable={stable} | fail={fail} | "
            "end_only={eoc} ({eor}) | fwd={fwd} | lat={lat} | roll={roll} | pitch={pitch} | ttf={ttf}".format(
                d=fmt(row["contact_timing_bias_s"]),
                n=row["n_runs"],
                nt=row.get("n_runs_total", "-"),
                outcome=row["outcome_majority"],
                succ=fmt(row["success_rate"]),
                stable=fmt(row["stable_rate"]),
                fail=fmt(row["fail_rate"]),
                eoc=row.get("end_only_stabilized_count", "-"),
                eor=fmt(row.get("end_only_stabilized_rate_total", "")),
                fwd=fmt(row["forward_progress_mean"]),
                lat=fmt(row["lateral_progress_mean"]),
                roll=fmt(row["roll_rms_mean"]),
                pitch=fmt(row["pitch_rms_mean"]),
                ttf=fmt(row["time_to_failure_mean"]),
            )
        )
    OUTPUT_TXT.write_text("\n".join(lines) + "\n")


def main():
    rows = load_rows()
    write_compact_csv(rows)
    write_compact_txt(rows)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_TXT}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, pstdev


def safe_mean(values):
    vals = [v for v in values if v is not None]
    return mean(vals) if vals else None


def safe_std(values):
    vals = [v for v in values if v is not None]
    return pstdev(vals) if len(vals) > 1 else 0.0 if len(vals) == 1 else None


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-root",
        default="/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/results",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/summary",
    )
    parser.add_argument(
        "--tag-filter",
        default="contactflagbias",
        help="Only include result directories whose names contain this string.",
    )
    parser.add_argument(
        "--terrain-height-cm",
        type=float,
        default=5.0,
        help="Terrain height to stamp into the CSV for the current batch.",
    )
    return parser.parse_args()


def extract_bias_from_name(name: str):
    token = None
    for part in name.split("_"):
        if part.startswith("ctp"):
            token = part
            break
    if token is None:
        return None
    raw = token[3:]
    if not raw:
        return None
    return float(raw) / 100.0


def outcome_label(row):
    if not row.get("task_completion_success", False):
        return "fail"
    forward = row.get("body_frame_forward_progress")
    lateral = abs(row.get("body_frame_lateral_progress", 0.0))
    roll = row.get("roll_rms_deg", 0.0)
    pitch = row.get("pitch_rms_deg", 0.0)
    if forward is None:
        return "fail"
    if forward < 0.5:
        return "fail"
    if lateral > 0.5 or roll > 5.0 or pitch > 5.0:
        return "unstable"
    return "stable"


def startup_validity(row):
    duration = row.get("duration_executed_s")
    min_base_z = row.get("min_base_z_m")
    startup_progress = row.get("startup_gait_body_frame_forward_progress_m")

    # Exclude runs that never really entered a valid locomotion phase.
    if duration is not None and duration < 8.0:
        return False, "startup_failure"
    if min_base_z is not None and min_base_z < 0.08:
        return False, "startup_collapse"
    if startup_progress is not None and startup_progress < -0.20:
        return False, "startup_failure"
    return True, None


def load_runs(results_root: Path, tag_filter: str, terrain_height_cm: float):
    rows = []
    for result_dir in sorted(results_root.iterdir()):
        if not result_dir.is_dir():
            continue
        if tag_filter and tag_filter not in result_dir.name:
            continue
        result_json = result_dir / "result.json"
        if not result_json.exists():
            continue
        with result_json.open() as f:
            data = json.load(f)
        bias = extract_bias_from_name(result_dir.name)
        row = {
            "run_id": result_dir.name,
            "timestamp": result_dir.name.split("_")[0],
            "terrain_key": next((p for p in result_dir.name.split("_") if p.startswith("ctp") or p == "d000"), None),
            "terrain_height_cm": terrain_height_cm,
            "contact_timing_bias_s": bias if bias is not None else 0.0,
            "repeat_idx": None,
            "success": data.get("success"),
            "task_completion_success": data.get("task_completion_success"),
            "fall_reason": data.get("fall_reason"),
            "time_to_failure_s": data.get("time_to_failure"),
            "duration_executed_s": data.get("duration_executed"),
            "distance_xy_m": data.get("distance_xy"),
            "body_frame_forward_progress_m": data.get("body_frame_forward_progress"),
            "body_frame_lateral_progress_m": data.get("body_frame_lateral_progress"),
            "roll_rms_deg": data.get("roll_rms_deg"),
            "pitch_rms_deg": data.get("pitch_rms_deg"),
            "yaw_rms_deg": data.get("yaw_rms_deg"),
            "yaw_change_deg": data.get("yaw_change_deg"),
            "min_base_z_m": data.get("min_base_z"),
            "base_z_std_m": data.get("base_z_std"),
            "startup_gait_body_frame_forward_progress_m": data.get("startup_gait_body_frame_forward_progress"),
            "start_pose_x": (data.get("start_pose") or [None, None, None])[0],
            "start_pose_y": (data.get("start_pose") or [None, None, None])[1],
            "start_pose_z": (data.get("start_pose") or [None, None, None])[2],
            "end_pose_x": (data.get("end_pose") or [None, None, None])[0],
            "end_pose_y": (data.get("end_pose") or [None, None, None])[1],
            "end_pose_z": (data.get("end_pose") or [None, None, None])[2],
            "result_dir": str(result_dir),
            "controller_csv_path": str(result_dir / "controller_state_input.csv"),
        }
        startup_valid, exclude_reason = startup_validity(row)
        row["startup_valid"] = startup_valid
        row["exclude_from_analysis"] = not startup_valid
        row["exclude_reason"] = exclude_reason
        row["outcome_label"] = outcome_label(
            {
                "task_completion_success": row["task_completion_success"],
                "body_frame_forward_progress": row["body_frame_forward_progress_m"],
                "body_frame_lateral_progress": row["body_frame_lateral_progress_m"],
                "roll_rms_deg": row["roll_rms_deg"],
                "pitch_rms_deg": row["pitch_rms_deg"],
            }
        )
        rows.append(row)
    return rows


def write_run_csv(rows, path: Path):
    fieldnames = [
        "run_id",
        "timestamp",
        "terrain_key",
        "terrain_height_cm",
        "contact_timing_bias_s",
        "repeat_idx",
        "success",
        "task_completion_success",
        "outcome_label",
        "fall_reason",
        "time_to_failure_s",
        "duration_executed_s",
        "distance_xy_m",
        "body_frame_forward_progress_m",
        "body_frame_lateral_progress_m",
        "roll_rms_deg",
        "pitch_rms_deg",
        "yaw_rms_deg",
        "yaw_change_deg",
        "min_base_z_m",
        "base_z_std_m",
        "startup_gait_body_frame_forward_progress_m",
        "startup_valid",
        "exclude_from_analysis",
        "exclude_reason",
        "start_pose_x",
        "start_pose_y",
        "start_pose_z",
        "end_pose_x",
        "end_pose_y",
        "end_pose_z",
        "result_dir",
        "controller_csv_path",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_aggregate_csv(rows, path: Path):
    grouped = {}
    for row in rows:
        if row.get("exclude_from_analysis"):
            continue
        key = (row["terrain_height_cm"], row["contact_timing_bias_s"])
        grouped.setdefault(key, []).append(row)

    fieldnames = [
        "terrain_height_cm",
        "contact_timing_bias_s",
        "n_runs",
        "success_rate",
        "stable_rate",
        "unstable_rate",
        "fail_rate",
        "forward_progress_mean",
        "forward_progress_std",
        "lateral_progress_mean",
        "lateral_progress_std",
        "roll_rms_mean",
        "roll_rms_std",
        "pitch_rms_mean",
        "pitch_rms_std",
        "time_to_failure_mean",
        "time_to_failure_std",
    ]

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for (terrain_height_cm, bias), group in sorted(grouped.items()):
            n = len(group)
            stable = sum(1 for r in group if r["outcome_label"] == "stable")
            unstable = sum(1 for r in group if r["outcome_label"] == "unstable")
            fail = sum(1 for r in group if r["outcome_label"] == "fail")
            writer.writerow(
                {
                    "terrain_height_cm": terrain_height_cm,
                    "contact_timing_bias_s": bias,
                    "n_runs": n,
                    "success_rate": sum(1 for r in group if r["task_completion_success"]) / n,
                    "stable_rate": stable / n,
                    "unstable_rate": unstable / n,
                    "fail_rate": fail / n,
                    "forward_progress_mean": safe_mean([r["body_frame_forward_progress_m"] for r in group]),
                    "forward_progress_std": safe_std([r["body_frame_forward_progress_m"] for r in group]),
                    "lateral_progress_mean": safe_mean([r["body_frame_lateral_progress_m"] for r in group]),
                    "lateral_progress_std": safe_std([r["body_frame_lateral_progress_m"] for r in group]),
                    "roll_rms_mean": safe_mean([r["roll_rms_deg"] for r in group]),
                    "roll_rms_std": safe_std([r["roll_rms_deg"] for r in group]),
                    "pitch_rms_mean": safe_mean([r["pitch_rms_deg"] for r in group]),
                    "pitch_rms_std": safe_std([r["pitch_rms_deg"] for r in group]),
                    "time_to_failure_mean": safe_mean([r["time_to_failure_s"] for r in group]),
                    "time_to_failure_std": safe_std([r["time_to_failure_s"] for r in group]),
                }
            )


def main():
    args = parse_args()
    results_root = Path(args.results_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_runs(results_root, args.tag_filter, args.terrain_height_cm)
    write_run_csv(rows, output_dir / "contact_timing_bias_runs.csv")
    write_aggregate_csv(rows, output_dir / "contact_timing_bias_aggregate.csv")
    print(f"[done] wrote {len(rows)} runs to {output_dir}")


if __name__ == "__main__":
    main()

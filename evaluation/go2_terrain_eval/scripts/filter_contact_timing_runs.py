#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from statistics import mean, pstdev


RESULTS_ROOT_DEFAULT = "/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/results"
OUTPUT_DIR_DEFAULT = "/home/ho/ros2_ws/filtered_results"

SHORT_DURATION_SEC = 8.0
MIN_BASE_Z = 0.08
MIN_FORWARD_PROGRESS = 0.5
STARTUP_FAILURE_PROGRESS = -0.20
LATE_STAB_EARLY_START = 0.0
LATE_STAB_EARLY_END = 5.0
LATE_STAB_LATE_START = 10.0
LATE_STAB_LATE_END = 18.0
MIN_EARLY_WINDOW_SAMPLES = 20
MIN_LATE_WINDOW_SAMPLES = 20
STABLE_ROLL_RMS_DEG = 5.0
STABLE_LATERAL_PROGRESS_M = 0.5
END_ONLY_STABILIZED_MIN_DURATION = 18.0
END_ONLY_STABILIZED_MAX_FORWARD = 0.5
END_ONLY_STABILIZED_MIN_DELTA_Z = 0.15
ROUND_DIGITS = 4


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", default=RESULTS_ROOT_DEFAULT)
    parser.add_argument("--output-dir", default=OUTPUT_DIR_DEFAULT)
    parser.add_argument(
        "--name-filter",
        default="contactflagbias",
        help="Only process run directories whose names contain this substring.",
    )
    return parser.parse_args()


def safe_float(v):
    if v in (None, "", "nan", "NaN"):
        return math.nan
    try:
        return float(v)
    except Exception:
        return math.nan


def safe_mean(values):
    vals = [v for v in values if v is not None and not math.isnan(v)]
    return mean(vals) if vals else math.nan


def safe_std(values):
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return math.nan
    return pstdev(vals) if len(vals) > 1 else 0.0


def round_row(row):
    rounded = {}
    for k, v in row.items():
        if isinstance(v, float):
            rounded[k] = round(v, ROUND_DIGITS) if not math.isnan(v) else ""
        else:
            rounded[k] = v
    return rounded


def find_run_directories(results_root: Path, name_filter: str):
    dirs = [p for p in results_root.iterdir() if p.is_dir()]
    if name_filter:
        dirs = [p for p in dirs if name_filter in p.name]
    return sorted(dirs)


def parse_run_metadata(run_name: str):
    terrain_height_cm = math.nan
    terrain_name = ""
    if "ocs2_stepdown_1cm" in run_name:
        terrain_height_cm = 1.0
        terrain_name = "ocs2_stepdown_1cm"
    elif "ocs2_stepdown_3cm" in run_name:
        terrain_height_cm = 3.0
        terrain_name = "ocs2_stepdown_3cm"
    elif "ocs2_stepdown_7cm" in run_name:
        terrain_height_cm = 7.0
        terrain_name = "ocs2_stepdown_7cm"
    elif "ocs2_stepdown_9cm" in run_name:
        terrain_height_cm = 9.0
        terrain_name = "ocs2_stepdown_9cm"
    elif "ocs2_stepdown" in run_name:
        terrain_height_cm = 5.0
        terrain_name = "ocs2_stepdown"

    bias = 0.0
    m = re.search(r"ctp(\d+)", run_name)
    if m:
        bias = int(m.group(1)) / 100.0

    repeat_idx = math.nan
    m = re.search(r"_r(\d+)", run_name)
    if m:
        repeat_idx = int(m.group(1))

    return {
        "terrain_name": terrain_name,
        "terrain_height_cm": terrain_height_cm,
        "contact_timing_bias_s": bias,
        "repeat_idx": repeat_idx,
    }


def load_result_json(run_dir: Path):
    path = run_dir / "result.json"
    if not path.exists():
        raise FileNotFoundError("missing result.json")
    with path.open() as f:
        return json.load(f)


def load_controller_csv(run_dir: Path):
    path = run_dir / "controller_state_input.csv"
    if not path.exists():
        raise FileNotFoundError("missing controller_state_input.csv")
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                if k in ("obs_mode", "planned_mode"):
                    parsed[k] = v
                else:
                    parsed[k] = safe_float(v)
            rows.append(parsed)
    return rows


def filter_window(rows, start, end):
    return [r for r in rows if "time" in r and not math.isnan(r["time"]) and start <= r["time"] <= end]


def get_time_bounds(rows):
    times = [r["time"] for r in rows if "time" in r and not math.isnan(r["time"])]
    if not times:
        return math.nan, math.nan
    return min(times), max(times)


def filter_relative_window(rows, rel_start, rel_end):
    t0, _ = get_time_bounds(rows)
    if math.isnan(t0):
        return []
    return filter_window(rows, t0 + rel_start, t0 + rel_end)


def get_relative_tail_window(rows, rel_duration):
    t0, t1 = get_time_bounds(rows)
    if math.isnan(t0) or math.isnan(t1):
        return []
    start = max(t0, t1 - rel_duration)
    return filter_window(rows, start, t1)


def mean_contact_sum(rows, prefix):
    cols = [f"{prefix}_{leg}" for leg in ("fl", "fr", "rl", "rr")]
    vals = []
    for r in rows:
        s = 0.0
        ok = False
        for c in cols:
            if c in r and not math.isnan(r[c]):
                s += r[c]
                ok = True
        if ok:
            vals.append(s)
    return safe_mean(vals)


def mode_of_values(values):
    vals = [v for v in values if v not in ("", None) and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return ""
    return Counter(vals).most_common(1)[0][0]


def detect_base_z_column(rows, result):
    if not rows:
        return "", ""

    target_z = safe_float(result.get("min_base_z"))

    def candidate_score(prefix, key):
        vals = [r.get(key, math.nan) for r in rows]
        vals = [v for v in vals if not math.isnan(v)]
        if len(vals) < 20:
            return None
        mean_v = safe_mean(vals)
        min_v = min(vals) if vals else math.nan
        max_v = max(vals) if vals else math.nan
        std_v = safe_std(vals)
        if math.isnan(mean_v) or math.isnan(min_v) or math.isnan(max_v):
            return None
        if min_v < -0.05 or max_v > 1.0:
            return None
        if mean_v < 0.02 or mean_v > 0.6:
            return None
        score = abs(mean_v - (target_z if not math.isnan(target_z) else mean_v))
        score += abs(min_v - (target_z if not math.isnan(target_z) else min_v))
        score += 0.1 * std_v
        return (score, prefix, key)

    candidates = []
    for prefix in ("obs_state_", "opt_state_"):
        sample = rows[0]
        for key in sample.keys():
            if key.startswith(prefix):
                scored = candidate_score(prefix, key)
                if scored is not None:
                    candidates.append(scored)

    if not candidates:
        return "", ""
    candidates.sort(key=lambda x: x[0])
    _, prefix, key = candidates[0]
    return key, prefix


def compute_start_end_z(rows, z_col):
    if not rows or not z_col:
        return math.nan, math.nan, 0, 0

    start_rows = filter_relative_window(rows, 0.0, 1.0)
    end_rows = get_relative_tail_window(rows, 1.0)
    if not start_rows:
        n = max(1, int(len(rows) * 0.1))
        start_rows = rows[:n]
    if not end_rows:
        n = max(1, int(len(rows) * 0.1))
        end_rows = rows[-n:]

    start_vals = [r.get(z_col, math.nan) for r in start_rows]
    end_vals = [r.get(z_col, math.nan) for r in end_rows]
    start_z = safe_mean(start_vals)
    end_z = safe_mean(end_vals)
    return start_z, end_z, len(start_rows), len(end_rows)


def classify_end_only_stabilized(row):
    start_z = row.get("start_z", math.nan)
    end_z = row.get("end_z", math.nan)
    delta_z = row.get("delta_z", math.nan)

    if row.get("exclusion_reason") != "low_progress":
        return ""
    if not bool(row.get("success")):
        return ""
    duration = row.get("duration_executed", math.nan)
    forward = row.get("forward_progress", math.nan)
    if math.isnan(duration) or math.isnan(forward) or math.isnan(start_z) or math.isnan(end_z) or math.isnan(delta_z):
        return ""
    if (
        duration >= END_ONLY_STABILIZED_MIN_DURATION
        and forward < END_ONLY_STABILIZED_MAX_FORWARD
        and delta_z > END_ONLY_STABILIZED_MIN_DELTA_Z
    ):
        return "end_only_stabilized"
    return ""


def check_startup_valid(result, controller_rows):
    duration = safe_float(result.get("duration_executed"))
    min_base_z = safe_float(result.get("min_base_z"))
    startup_progress = safe_float(result.get("startup_gait_body_frame_forward_progress"))

    if not math.isnan(duration) and duration < SHORT_DURATION_SEC:
        return False, "short_duration", f"duration_executed={duration:.4f}"
    if not math.isnan(min_base_z) and min_base_z < MIN_BASE_Z:
        return False, "startup_collapse", f"min_base_z={min_base_z:.4f}"
    if not math.isnan(startup_progress) and startup_progress < STARTUP_FAILURE_PROGRESS:
        return False, "startup_failure", f"startup_progress={startup_progress:.4f}"

    early = filter_relative_window(controller_rows, LATE_STAB_EARLY_START, LATE_STAB_EARLY_END)
    if len(early) < MIN_EARLY_WINDOW_SAMPLES:
        return False, "insufficient_startup_samples", f"early_samples={len(early)}"

    early_contact_sum = mean_contact_sum(early, "obs_contact")
    if not math.isnan(early_contact_sum) and early_contact_sum > 3.98 and (not math.isnan(startup_progress) and startup_progress < 0.01):
        return False, "startup_failure", f"early_obs_contact_sum={early_contact_sum:.4f}, startup_progress={startup_progress:.4f}"
    return True, "", ""


def check_late_stabilization(result, controller_rows):
    early = filter_relative_window(controller_rows, LATE_STAB_EARLY_START, LATE_STAB_EARLY_END)
    late = filter_relative_window(controller_rows, LATE_STAB_LATE_START, LATE_STAB_LATE_END)
    if len(early) < MIN_EARLY_WINDOW_SAMPLES or len(late) < MIN_LATE_WINDOW_SAMPLES:
        return False, ""

    startup_progress = safe_float(result.get("startup_gait_body_frame_forward_progress"))
    overall_forward = safe_float(result.get("body_frame_forward_progress"))
    duration = safe_float(result.get("duration_executed"))
    start_pose = result.get("start_pose")
    end_pose = result.get("end_pose")
    start_z = start_pose[2] if isinstance(start_pose, list) and len(start_pose) >= 3 else math.nan
    end_z = end_pose[2] if isinstance(end_pose, list) and len(end_pose) >= 3 else math.nan

    if (
        not math.isnan(startup_progress)
        and not math.isnan(overall_forward)
        and not math.isnan(duration)
        and not math.isnan(start_z)
        and not math.isnan(end_z)
        and duration >= 18.0
        and startup_progress < 0.01
        and overall_forward < MIN_FORWARD_PROGRESS
        and (end_z - start_z) > 0.15
    ):
        return True, (
            f"startup_progress={startup_progress:.4f}, "
            f"forward_progress={overall_forward:.4f}, "
            f"start_z={start_z:.4f}, end_z={end_z:.4f}, duration={duration:.4f}"
        )
    return False, ""


def check_forward_progress(result):
    progress = safe_float(result.get("body_frame_forward_progress"))
    if math.isnan(progress):
        return False, "missing_forward_progress"
    if progress < MIN_FORWARD_PROGRESS:
        return False, f"low_progress:{progress:.4f}"
    return True, ""


def check_short_duration(result):
    duration = safe_float(result.get("duration_executed"))
    if math.isnan(duration):
        return False, "missing_duration"
    if duration < SHORT_DURATION_SEC:
        return False, f"short_duration:{duration:.4f}"
    return True, ""


def assign_outcome_label(row):
    if (not bool(row["task_completion_success"])) or (not math.isnan(row["time_to_failure"])):
        return "fail"
    if bool(row["success"]) and row["roll_rms_deg"] < STABLE_ROLL_RMS_DEG and not bool(row["late_stabilization"]):
        return "stable"
    if bool(row["success"]):
        return "unstable"
    return "fail"


def l2_norm_mean(rows, prefix):
    indices = []
    sample = rows[0] if rows else {}
    for k in sample:
        if k.startswith(prefix):
            indices.append(k)
    if not indices:
        return math.nan

    vals = []
    for r in rows:
        s = 0.0
        has = False
        for k in indices:
            v = r.get(k, math.nan)
            if not math.isnan(v):
                s += v * v
                has = True
        if has:
            vals.append(math.sqrt(s))
    return safe_mean(vals)


def compute_controller_summary(rows):
    summary = {
        "obs_state_norm_mean": l2_norm_mean(rows, "obs_state_"),
        "opt_state_norm_mean": l2_norm_mean(rows, "opt_state_"),
        "obs_input_norm_mean": l2_norm_mean(rows, "obs_input_"),
        "opt_input_norm_mean": l2_norm_mean(rows, "opt_input_"),
        "planned_mode_mode": mode_of_values([r.get("planned_mode", "") for r in rows]),
        "obs_mode_mode": mode_of_values([r.get("obs_mode", "") for r in rows]),
    }
    for leg in ("fl", "fr", "rl", "rr"):
        summary[f"obs_contact_mean_{leg}"] = safe_mean([safe_float(r.get(f"obs_contact_{leg}")) for r in rows])
        summary[f"plan_contact_mean_{leg}"] = safe_mean([safe_float(r.get(f"plan_contact_{leg}")) for r in rows])
    return summary


def build_filtered_and_excluded(results_root: Path, name_filter: str):
    filtered_rows = []
    excluded_rows = []

    for run_dir in find_run_directories(results_root, name_filter):
        run_id = run_dir.name
        meta = parse_run_metadata(run_id)
        base = {
            "run_id": run_id,
            "original_path": str(run_dir),
            **meta,
        }

        try:
            result = load_result_json(run_dir)
        except FileNotFoundError as e:
            filtered_rows.append({**base, "startup_valid": False, "late_stabilization": False, "forward_progress": math.nan, "duration_executed": math.nan, "success": False, "task_completion_success": False, "filter_pass": False, "exclusion_reason": "missing_file"})
            excluded_rows.append({"run_id": run_id, "original_path": str(run_dir), "exclusion_reason": "missing_file", "detail": str(e)})
            continue
        except Exception as e:
            filtered_rows.append({**base, "startup_valid": False, "late_stabilization": False, "forward_progress": math.nan, "duration_executed": math.nan, "success": False, "task_completion_success": False, "filter_pass": False, "exclusion_reason": "corrupted_file"})
            excluded_rows.append({"run_id": run_id, "original_path": str(run_dir), "exclusion_reason": "corrupted_file", "detail": f"result.json read failed: {e}"})
            continue

        try:
            controller_rows = load_controller_csv(run_dir)
        except FileNotFoundError as e:
            filtered_rows.append({**base, "startup_valid": False, "late_stabilization": False, "forward_progress": safe_float(result.get("body_frame_forward_progress")), "duration_executed": safe_float(result.get("duration_executed")), "success": bool(result.get("success")), "task_completion_success": bool(result.get("task_completion_success")), "filter_pass": False, "exclusion_reason": "missing_file"})
            excluded_rows.append({"run_id": run_id, "original_path": str(run_dir), "exclusion_reason": "missing_file", "detail": str(e)})
            continue
        except Exception as e:
            filtered_rows.append({**base, "startup_valid": False, "late_stabilization": False, "forward_progress": safe_float(result.get("body_frame_forward_progress")), "duration_executed": safe_float(result.get("duration_executed")), "success": bool(result.get("success")), "task_completion_success": bool(result.get("task_completion_success")), "filter_pass": False, "exclusion_reason": "corrupted_file"})
            excluded_rows.append({"run_id": run_id, "original_path": str(run_dir), "exclusion_reason": "corrupted_file", "detail": f"controller_state_input.csv read failed: {e}"})
            continue

        startup_valid, startup_reason, startup_detail = check_startup_valid(result, controller_rows)
        late_stab, late_detail = check_late_stabilization(result, controller_rows)
        progress_ok, progress_detail = check_forward_progress(result)
        duration_ok, duration_detail = check_short_duration(result)
        controller_summary = compute_controller_summary(controller_rows)
        z_col, z_source = detect_base_z_column(controller_rows, result)
        start_z, end_z, start_z_samples, end_z_samples = compute_start_end_z(controller_rows, z_col)
        delta_z = end_z - start_z if not math.isnan(start_z) and not math.isnan(end_z) else math.nan

        exclusion_reason = ""
        exclusion_detail = ""
        if not startup_valid:
            exclusion_reason = startup_reason
            exclusion_detail = startup_detail
        elif not progress_ok:
            exclusion_reason = "low_progress"
            exclusion_detail = progress_detail
        elif not duration_ok:
            exclusion_reason = "short_duration"
            exclusion_detail = duration_detail

        row = {
            **base,
            "startup_valid": startup_valid,
            "late_stabilization": late_stab,
            "forward_progress": safe_float(result.get("body_frame_forward_progress")),
            "duration_executed": safe_float(result.get("duration_executed")),
            "success": bool(result.get("success")),
            "task_completion_success": bool(result.get("task_completion_success")),
            "filter_pass": exclusion_reason == "",
            "exclusion_reason": exclusion_reason,
            "secondary_label": "",
            "fall_reason": result.get("fall_reason", ""),
            "time_to_failure": safe_float(result.get("time_to_failure")),
            "body_frame_lateral_progress": safe_float(result.get("body_frame_lateral_progress")),
            "roll_rms_deg": safe_float(result.get("roll_rms_deg")),
            "pitch_rms_deg": safe_float(result.get("pitch_rms_deg")),
            "yaw_rms_deg": safe_float(result.get("yaw_rms_deg")),
            "yaw_change_deg": safe_float(result.get("yaw_change_deg")),
            "min_base_z": safe_float(result.get("min_base_z")),
            "base_z_std": safe_float(result.get("base_z_std")),
            "startup_gait_body_frame_forward_progress": safe_float(result.get("startup_gait_body_frame_forward_progress")),
            "distance_xy": safe_float(result.get("distance_xy")),
            "mean_forward_velocity": safe_float(result.get("mean_forward_velocity")),
            "start_z": start_z,
            "end_z": end_z,
            "delta_z": delta_z,
            "base_z_column": z_col,
            "base_z_column_source": z_source,
            "start_z_samples": start_z_samples,
            "end_z_samples": end_z_samples,
            **controller_summary,
        }
        row["secondary_label"] = classify_end_only_stabilized(row)
        row["outcome_label"] = assign_outcome_label(row)
        filtered_rows.append(row)

        if exclusion_reason:
            excluded_rows.append({
                "run_id": run_id,
                "original_path": str(run_dir),
                "exclusion_reason": exclusion_reason,
                "secondary_label": row["secondary_label"],
                "detail": exclusion_detail if exclusion_detail else f"z_col={z_col}, start_z={start_z}, end_z={end_z}, delta_z={delta_z}, start_samples={start_z_samples}, end_samples={end_z_samples}",
            })

    return filtered_rows, excluded_rows


def build_researcher_summary(all_rows):
    groups = {}
    for row in all_rows:
        key = (row["terrain_height_cm"], row["contact_timing_bias_s"])
        groups.setdefault(key, []).append(row)

    summary_rows = []
    for key in sorted(groups.keys()):
        terrain_height_cm, bias = key
        group_all = groups[key]
        group = [r for r in group_all if r["filter_pass"]]
        n_runs = len(group)
        n_total = len(group_all)
        stable = [r for r in group if r["outcome_label"] == "stable"]
        unstable = [r for r in group if r["outcome_label"] == "unstable"]
        fail = [r for r in group if r["outcome_label"] == "fail"]
        end_only = [r for r in group_all if r["secondary_label"] == "end_only_stabilized"]
        outcome_majority = Counter([r["outcome_label"] for r in group]).most_common(1)[0][0] if group else "none"

        summary_rows.append(
            {
                "terrain_height_cm": terrain_height_cm,
                "contact_timing_bias_s": bias,
                "n_runs": n_runs,
                "n_runs_total": n_total,
                "n_runs_excluded": n_total - n_runs,
                "success_rate": (sum(1 for r in group if r["task_completion_success"]) / n_runs) if n_runs else math.nan,
                "stable_rate": (len(stable) / n_runs) if n_runs else math.nan,
                "unstable_rate": (len(unstable) / n_runs) if n_runs else math.nan,
                "fail_rate": (len(fail) / n_runs) if n_runs else math.nan,
                "outcome_majority": outcome_majority,
                "startup_valid_rate": (sum(1 for r in group if r["startup_valid"]) / n_runs) if n_runs else math.nan,
                "late_stabilization_rate": (sum(1 for r in group if r["late_stabilization"]) / n_runs) if n_runs else math.nan,
                "end_only_stabilized_count": len(end_only),
                "end_only_stabilized_rate_total": len(end_only) / n_total if n_total else math.nan,
                "end_only_stabilized_rate_excluded": len(end_only) / (n_total - n_runs) if (n_total - n_runs) else math.nan,
                "forward_progress_mean": safe_mean([r["forward_progress"] for r in group]),
                "forward_progress_std": safe_std([r["forward_progress"] for r in group]),
                "lateral_progress_mean": safe_mean([r["body_frame_lateral_progress"] for r in group]),
                "lateral_progress_std": safe_std([r["body_frame_lateral_progress"] for r in group]),
                "roll_rms_mean": safe_mean([r["roll_rms_deg"] for r in group]),
                "roll_rms_std": safe_std([r["roll_rms_deg"] for r in group]),
                "pitch_rms_mean": safe_mean([r["pitch_rms_deg"] for r in group]),
                "pitch_rms_std": safe_std([r["pitch_rms_deg"] for r in group]),
                "yaw_rms_mean": safe_mean([r["yaw_rms_deg"] for r in group]),
                "yaw_rms_std": safe_std([r["yaw_rms_deg"] for r in group]),
                "yaw_change_deg_mean": safe_mean([r["yaw_change_deg"] for r in group]),
                "yaw_change_deg_std": safe_std([r["yaw_change_deg"] for r in group]),
                "duration_mean": safe_mean([r["duration_executed"] for r in group]),
                "duration_std": safe_std([r["duration_executed"] for r in group]),
                "time_to_failure_mean": safe_mean([r["time_to_failure"] for r in fail]),
                "time_to_failure_std": safe_std([r["time_to_failure"] for r in fail]),
                "min_base_z_mean": safe_mean([r["min_base_z"] for r in group]),
                "min_base_z_std": safe_std([r["min_base_z"] for r in group]),
                "base_z_std_mean": safe_mean([r["base_z_std"] for r in group]),
                "base_z_std_std": safe_std([r["base_z_std"] for r in group]),
                "startup_gait_body_frame_forward_progress_mean": safe_mean([r["startup_gait_body_frame_forward_progress"] for r in group]),
                "startup_gait_body_frame_forward_progress_std": safe_std([r["startup_gait_body_frame_forward_progress"] for r in group]),
                "distance_xy_mean": safe_mean([r["distance_xy"] for r in group]),
                "distance_xy_std": safe_std([r["distance_xy"] for r in group]),
                "mean_forward_velocity_mean": safe_mean([r["mean_forward_velocity"] for r in group]),
                "mean_forward_velocity_std": safe_std([r["mean_forward_velocity"] for r in group]),
                "start_z_mean": safe_mean([r["start_z"] for r in group]),
                "start_z_std": safe_std([r["start_z"] for r in group]),
                "end_z_mean": safe_mean([r["end_z"] for r in group]),
                "end_z_std": safe_std([r["end_z"] for r in group]),
                "delta_z_mean": safe_mean([r["delta_z"] for r in group]),
                "delta_z_std": safe_std([r["delta_z"] for r in group]),
                "obs_state_norm_mean": safe_mean([r["obs_state_norm_mean"] for r in group]),
                "opt_state_norm_mean": safe_mean([r["opt_state_norm_mean"] for r in group]),
                "obs_input_norm_mean": safe_mean([r["obs_input_norm_mean"] for r in group]),
                "opt_input_norm_mean": safe_mean([r["opt_input_norm_mean"] for r in group]),
                "obs_contact_mean_fl": safe_mean([r["obs_contact_mean_fl"] for r in group]),
                "obs_contact_mean_fr": safe_mean([r["obs_contact_mean_fr"] for r in group]),
                "obs_contact_mean_rl": safe_mean([r["obs_contact_mean_rl"] for r in group]),
                "obs_contact_mean_rr": safe_mean([r["obs_contact_mean_rr"] for r in group]),
                "plan_contact_mean_fl": safe_mean([r["plan_contact_mean_fl"] for r in group]),
                "plan_contact_mean_fr": safe_mean([r["plan_contact_mean_fr"] for r in group]),
                "plan_contact_mean_rl": safe_mean([r["plan_contact_mean_rl"] for r in group]),
                "plan_contact_mean_rr": safe_mean([r["plan_contact_mean_rr"] for r in group]),
                "planned_mode_mode": mode_of_values([r["planned_mode_mode"] for r in group]),
                "obs_mode_mode": mode_of_values([r["obs_mode_mode"] for r in group]),
            }
        )
    return summary_rows


def write_csv(path: Path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(round_row(row))


def print_researcher_summary(summary_rows):
    by_terrain = {}
    for row in summary_rows:
        by_terrain.setdefault(row["terrain_height_cm"], []).append(row)
    for terrain in sorted(by_terrain.keys()):
        print(f"terrain {int(terrain)}cm")
        for row in sorted(by_terrain[terrain], key=lambda r: r["contact_timing_bias_s"]):
            print(
                f"  d={row['contact_timing_bias_s']:.2f} : "
                f"stable_rate={row['stable_rate']:.4f}, "
                f"unstable_rate={row['unstable_rate']:.4f}, "
                f"fail_rate={row['fail_rate']:.4f}, "
                f"end_only_total_rate={row['end_only_stabilized_rate_total']:.4f}, "
                f"forward_mean={row['forward_progress_mean']:.4f}, "
                f"roll_mean={row['roll_rms_mean']:.4f}"
            )


def summarize_end_only_stabilized(filtered_rows):
    tagged = [r for r in filtered_rows if r.get("secondary_label") == "end_only_stabilized"]
    print(f"low_progress total: {sum(1 for r in filtered_rows if r.get('exclusion_reason') == 'low_progress')}")
    print(f"end_only_stabilized total: {len(tagged)}")
    counts = Counter((r["terrain_height_cm"], r["contact_timing_bias_s"]) for r in tagged)
    for (terrain, bias), count in sorted(counts.items()):
        print(f"terrain {int(terrain)}cm, d={bias:.2f} -> end_only_stabilized {count} runs")


def main():
    args = parse_args()
    results_root = Path(args.results_root)
    output_dir = Path(args.output_dir)

    filtered_rows, excluded_rows = build_filtered_and_excluded(results_root, args.name_filter)
    valid_rows = [r for r in filtered_rows if r["filter_pass"]]
    summary_rows = build_researcher_summary(filtered_rows)

    filtered_fieldnames = [
        "run_id",
        "original_path",
        "terrain_name",
        "terrain_height_cm",
        "contact_timing_bias_s",
        "repeat_idx",
        "startup_valid",
        "late_stabilization",
        "secondary_label",
        "start_z",
        "end_z",
        "delta_z",
        "forward_progress",
        "duration_executed",
        "success",
        "task_completion_success",
        "filter_pass",
        "exclusion_reason",
    ]
    excluded_fieldnames = ["run_id", "original_path", "exclusion_reason", "secondary_label", "detail"]
    summary_fieldnames = [
        "terrain_height_cm",
        "contact_timing_bias_s",
        "n_runs",
        "n_runs_total",
        "n_runs_excluded",
        "success_rate",
        "stable_rate",
        "unstable_rate",
        "fail_rate",
        "outcome_majority",
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
        "yaw_change_deg_mean",
        "yaw_change_deg_std",
        "duration_mean",
        "duration_std",
        "time_to_failure_mean",
        "time_to_failure_std",
        "min_base_z_mean",
        "min_base_z_std",
        "base_z_std_mean",
        "base_z_std_std",
        "startup_gait_body_frame_forward_progress_mean",
        "startup_gait_body_frame_forward_progress_std",
        "distance_xy_mean",
        "distance_xy_std",
        "mean_forward_velocity_mean",
        "mean_forward_velocity_std",
        "start_z_mean",
        "start_z_std",
        "end_z_mean",
        "end_z_std",
        "delta_z_mean",
        "delta_z_std",
        "obs_state_norm_mean",
        "opt_state_norm_mean",
        "obs_input_norm_mean",
        "opt_input_norm_mean",
        "obs_contact_mean_fl",
        "obs_contact_mean_fr",
        "obs_contact_mean_rl",
        "obs_contact_mean_rr",
        "plan_contact_mean_fl",
        "plan_contact_mean_fr",
        "plan_contact_mean_rl",
        "plan_contact_mean_rr",
        "planned_mode_mode",
        "obs_mode_mode",
    ]

    filtered_rows_sorted = sorted(
        [{k: r.get(k, "") for k in filtered_fieldnames} for r in filtered_rows],
        key=lambda r: (
            math.inf if r["terrain_height_cm"] == "" or (isinstance(r["terrain_height_cm"], float) and math.isnan(r["terrain_height_cm"])) else r["terrain_height_cm"],
            math.inf if r["contact_timing_bias_s"] == "" or (isinstance(r["contact_timing_bias_s"], float) and math.isnan(r["contact_timing_bias_s"])) else r["contact_timing_bias_s"],
            math.inf if r["repeat_idx"] == "" or (isinstance(r["repeat_idx"], float) and math.isnan(r["repeat_idx"])) else r["repeat_idx"],
        ),
    )
    write_csv(output_dir / "contact_timing_bias_runs_filtered.csv", filtered_rows_sorted, filtered_fieldnames)
    write_csv(output_dir / "excluded_runs_log.csv", excluded_rows, excluded_fieldnames)
    write_csv(output_dir / "contact_timing_bias_researcher_summary.csv", summary_rows, summary_fieldnames)

    total_runs = len(filtered_rows)
    valid_runs = len(valid_rows)
    excluded_runs = total_runs - valid_runs
    reason_counts = Counter([r["exclusion_reason"] for r in filtered_rows if r["exclusion_reason"]])

    print(f"total runs: {total_runs}")
    print(f"valid runs: {valid_runs}")
    print(f"excluded runs: {excluded_runs}")
    for reason in [
        "startup_failure",
        "startup_collapse",
        "late_stabilization",
        "insufficient_startup_samples",
        "low_progress",
        "short_duration",
        "missing_file",
        "corrupted_file",
    ]:
        print(f"{reason}: {reason_counts.get(reason, 0)}")
    summarize_end_only_stabilized(filtered_rows)
    print_researcher_summary(summary_rows)


if __name__ == "__main__":
    main()

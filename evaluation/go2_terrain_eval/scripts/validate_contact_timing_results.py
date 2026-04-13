#!/usr/bin/env python3
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


FILTERED_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_runs_filtered.csv")
EXCLUDED_CSV = Path("/home/ho/ros2_ws/filtered_results/excluded_runs_log.csv")
SUMMARY_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_summary.csv")
COMPACT_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_compact_summary.csv")
RESULTS_ROOT = Path("/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/results")

REPORT_TXT = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_validation_report.txt")
SAMPLES_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_validation_samples.csv")

TARGET_COMBOS = [
    (1.0, 0.10),
    (3.0, 0.12),
    (5.0, 0.09),
    (5.0, 0.10),
    (7.0, 0.07),
    (9.0, 0.12),
]


def safe_float(v):
    if v in (None, "", "nan", "NaN"):
        return math.nan
    try:
        return float(v)
    except Exception:
        return math.nan


def fmt(v):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def read_json(path):
    with path.open() as f:
        return json.load(f)


def get_result_fields(run_dir):
    result = read_json(run_dir / "result.json")
    duration = safe_float(result.get("duration_executed"))
    fwd = safe_float(result.get("body_frame_forward_progress"))
    min_base_z = safe_float(result.get("min_base_z"))
    success = bool(result.get("success"))
    task_success = bool(result.get("task_completion_success"))
    return result, duration, fwd, min_base_z, success, task_success


def get_controller_time_bounds(run_dir):
    path = run_dir / "controller_state_input.csv"
    if not path.exists():
        return math.nan, math.nan
    start = math.nan
    end = math.nan
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = safe_float(row.get("time"))
            if math.isnan(t):
                continue
            if math.isnan(start):
                start = t
            end = t
    return start, end


def key_of(row):
    return (safe_float(row["terrain_height_cm"]), safe_float(row["contact_timing_bias_s"]))


def compute_rates(rows):
    n = len(rows)
    if n == 0:
        return {
            "n": 0,
            "success_rate": math.nan,
            "stable_rate": math.nan,
            "unstable_rate": math.nan,
            "fail_rate": math.nan,
        }
    stable = 0
    unstable = 0
    fail = 0
    success = 0
    for r in rows:
        run_dir = Path(r["original_path"])
        result, _, _, _, success_flag, task_success = get_result_fields(run_dir)
        roll_rms = safe_float(result.get("roll_rms_deg"))
        time_to_failure = safe_float(result.get("time_to_failure"))
        late_stab = (r.get("late_stabilization") == "True")
        if task_success:
            success += 1
        if (not task_success) or (not math.isnan(time_to_failure)):
            fail += 1
        elif success_flag and roll_rms < 5.0 and not late_stab:
            stable += 1
        elif success_flag:
            unstable += 1
        else:
            fail += 1
    return {
        "n": n,
        "success_rate": success / n,
        "stable_rate": stable / n,
        "unstable_rate": unstable / n,
        "fail_rate": fail / n,
    }


def pick_examples(rows, want_pass):
    subset = [r for r in rows if (r["filter_pass"] == "True") == want_pass]
    subset.sort(key=lambda r: (r["run_id"]))
    return subset[:2]


def make_note(row):
    if row["filter_pass"] == "True":
        return "pass"
    return f"excluded:{row['exclusion_reason']}"


def validate():
    filtered = read_csv(FILTERED_CSV)
    excluded = read_csv(EXCLUDED_CSV)
    summary = read_csv(SUMMARY_CSV)
    compact = read_csv(COMPACT_CSV)

    excluded_map = {(r["run_id"], r["original_path"]): r for r in excluded}
    filtered_map = {(r["run_id"], r["original_path"]): r for r in filtered}

    lines = []
    sample_rows = []

    lines.append("Contact Timing Bias Validation Report")
    lines.append("")

    lines.append("[Global consistency checks]")
    lines.append(f"filtered rows: {len(filtered)}")
    lines.append(f"excluded log rows: {len(excluded)}")

    filtered_excluded = [r for r in filtered if r["filter_pass"] == "False"]
    lines.append(f"filtered rows with filter_pass=False: {len(filtered_excluded)}")
    lines.append(f"excluded log count matches filtered exclusions: {len(filtered_excluded) == len(excluded)}")

    missing_in_excluded = []
    for r in filtered_excluded:
        key = (r["run_id"], r["original_path"])
        if key not in excluded_map:
            missing_in_excluded.append(key)
    extra_in_excluded = []
    for r in excluded:
        key = (r["run_id"], r["original_path"])
        if key not in filtered_map:
            extra_in_excluded.append(key)

    lines.append(f"missing in excluded log: {len(missing_in_excluded)}")
    lines.append(f"extra in excluded log: {len(extra_in_excluded)}")
    lines.append(f"secondary_label=end_only_stabilized count: {sum(1 for r in filtered if r.get('secondary_label') == 'end_only_stabilized')}")
    lines.append("")

    summary_by_key = {key_of(r): r for r in summary}
    compact_by_key = {key_of(r): r for r in compact}

    lines.append("[Spot-check target combos]")
    for terrain, bias in TARGET_COMBOS:
        combo_rows = [r for r in filtered if key_of(r) == (terrain, bias)]
        combo_valid = [r for r in combo_rows if r["filter_pass"] == "True"]
        combo_excluded = [r for r in combo_rows if r["filter_pass"] == "False"]
        rates = compute_rates(combo_valid)
        summary_row = summary_by_key.get((terrain, bias))
        compact_row = compact_by_key.get((terrain, bias))

        lines.append(f"terrain={terrain:.1f}cm, d={bias:.2f}s")
        lines.append(f"  raw count from filtered CSV: total={len(combo_rows)}, pass={len(combo_valid)}, excluded={len(combo_excluded)}")

        actual_dirs = [
            p for p in RESULTS_ROOT.iterdir()
            if p.is_dir() and p.name == ""
        ]
        # Use filtered CSV as directory index but verify raw folder existence.
        existing_raw = sum(1 for r in combo_rows if Path(r["original_path"]).is_dir())
        lines.append(f"  raw run folders existing: {existing_raw}")

        if summary_row:
            lines.append(
                "  summary match: "
                f"n_runs csv={summary_row['n_runs']} vs actual={rates['n']} | "
                f"success_rate csv={summary_row['success_rate']} vs actual={fmt(rates['success_rate'])} | "
                f"stable_rate csv={summary_row['stable_rate']} vs actual={fmt(rates['stable_rate'])} | "
                f"fail_rate csv={summary_row['fail_rate']} vs actual={fmt(rates['fail_rate'])}"
            )
        else:
            lines.append("  summary row: missing")

        if compact_row:
            lines.append(
                "  compact match: "
                f"n_runs={compact_row['n_runs']}, outcome_majority={compact_row['outcome_majority']}, "
                f"success_rate={compact_row['success_rate']}"
            )
        else:
            lines.append("  compact row: missing")

        reason_counts = Counter(r["exclusion_reason"] for r in combo_excluded)
        lines.append(f"  exclusion reasons: {dict(reason_counts)}")

        pass_examples = pick_examples(combo_rows, True)
        excl_examples = pick_examples(combo_rows, False)

        for label, example_rows in [("pass examples", pass_examples), ("excluded examples", excl_examples)]:
            lines.append(f"  {label}:")
            if not example_rows:
                lines.append("    - none")
                continue
            for row in example_rows:
                run_dir = Path(row["original_path"])
                _, duration, fwd, min_base_z, success, task_success = get_result_fields(run_dir)
                t0, t1 = get_controller_time_bounds(run_dir)
                lines.append(
                    f"    - {row['run_id']} | pass={row['filter_pass']} | reason={row['exclusion_reason'] or '-'} | "
                    f"success={success} | task_success={task_success} | duration={fmt(duration)} | "
                    f"forward={fmt(fwd)} | min_base_z={fmt(min_base_z)} | ctrl_time=[{fmt(t0)}, {fmt(t1)}]"
                )
                sample_rows.append({
                    "terrain_height_cm": terrain,
                    "contact_timing_bias_s": bias,
                    "run_id": row["run_id"],
                    "filter_pass": row["filter_pass"],
                    "exclusion_reason": row["exclusion_reason"],
                    "success": success,
                    "task_completion_success": task_success,
                    "duration_executed": duration,
                    "body_frame_forward_progress": fwd,
                    "min_base_z": min_base_z,
                    "controller_time_start": t0,
                    "controller_time_end": t1,
                    "note": make_note(row),
                })
        lines.append("")

    REPORT_TXT.write_text("\n".join(lines) + "\n")

    sample_fields = [
        "terrain_height_cm",
        "contact_timing_bias_s",
        "run_id",
        "filter_pass",
        "exclusion_reason",
        "success",
        "task_completion_success",
        "duration_executed",
        "body_frame_forward_progress",
        "min_base_z",
        "controller_time_start",
        "controller_time_end",
        "note",
    ]
    with SAMPLES_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=sample_fields)
        writer.writeheader()
        for row in sample_rows:
            out = {}
            for k in sample_fields:
                v = row.get(k, "")
                if isinstance(v, float):
                    out[k] = "" if math.isnan(v) else round(v, 4)
                else:
                    out[k] = v
            writer.writerow(out)

    print(f"Wrote {REPORT_TXT}")
    print(f"Wrote {SAMPLES_CSV}")


if __name__ == "__main__":
    validate()

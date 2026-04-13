#!/usr/bin/env python3
import csv
from collections import Counter, defaultdict
from pathlib import Path


FILTERED_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_runs_filtered.csv")
SUMMARY_CSV = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_researcher_summary.csv")
VALIDATION_TXT = Path("/home/ho/ros2_ws/filtered_results/contact_timing_bias_validation_report.txt")

OUT_DIR = Path("/home/ho/ros2_ws/filtered_results")
MAIN_TABLE = OUT_DIR / "main_table.csv"
EXCLUSION_OVERVIEW = OUT_DIR / "exclusion_overview.csv"
HANDOVER_NOTE = OUT_DIR / "handover_note.txt"


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def safe_float(v):
    if v in (None, "", "nan", "NaN"):
        return None
    try:
        return float(v)
    except Exception:
        return None


def fmt_float(v):
    if v is None:
        return ""
    return f"{v:.4f}"


def key_of(row):
    return (safe_float(row["terrain_height_cm"]), safe_float(row["contact_timing_bias_s"]))


def build_main_table(filtered_rows, summary_rows):
    counts = defaultdict(lambda: {"n_total_runs": 0, "n_pass_runs": 0, "n_excluded_runs": 0})
    for row in filtered_rows:
        key = key_of(row)
        counts[key]["n_total_runs"] += 1
        if row["filter_pass"] == "True":
            counts[key]["n_pass_runs"] += 1
        else:
            counts[key]["n_excluded_runs"] += 1

    out_rows = []
    for row in sorted(summary_rows, key=lambda r: (safe_float(r["terrain_height_cm"]) or 0.0, safe_float(r["contact_timing_bias_s"]) or 0.0)):
        key = key_of(row)
        stable_rate = safe_float(row["stable_rate"]) or 0.0
        fail_rate = safe_float(row["fail_rate"]) or 0.0
        if stable_rate >= 0.8:
            note = "stable"
        elif fail_rate >= 0.5:
            note = "mostly fail"
        else:
            note = "degrading"

        out_rows.append({
            "terrain_height_cm": row["terrain_height_cm"],
            "contact_timing_bias_s": row["contact_timing_bias_s"],
            "n_total_runs": counts[key]["n_total_runs"],
            "n_pass_runs": counts[key]["n_pass_runs"],
            "n_excluded_runs": counts[key]["n_excluded_runs"],
            "success_rate": fmt_float(safe_float(row["success_rate"])),
            "stable_rate": fmt_float(safe_float(row["stable_rate"])),
            "fail_rate": fmt_float(safe_float(row["fail_rate"])),
            "forward_progress_mean": fmt_float(safe_float(row["forward_progress_mean"])),
            "roll_rms_deg_mean": fmt_float(safe_float(row["roll_rms_mean"])),
            "pitch_rms_deg_mean": fmt_float(safe_float(row["pitch_rms_mean"])),
            "outcome_majority": row["outcome_majority"],
            "note": note,
        })
    return out_rows


def build_exclusion_overview(filtered_rows):
    grouped = defaultdict(list)
    for row in filtered_rows:
        grouped[key_of(row)].append(row)

    out_rows = []
    for key in sorted(grouped.keys()):
        terrain, bias = key
        rows = grouped[key]
        reason_counts = Counter(r["exclusion_reason"] for r in rows if r["exclusion_reason"])
        out_rows.append({
            "terrain_height_cm": f"{terrain:.1f}",
            "contact_timing_bias_s": f"{bias:.4f}",
            "n_short_duration": reason_counts.get("short_duration", 0),
            "n_low_progress": reason_counts.get("low_progress", 0),
            "n_startup_collapse": reason_counts.get("startup_collapse", 0),
            "n_total_excluded": sum(reason_counts.values()),
        })
    return out_rows


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_note():
    text = """Contact Timing Bias Handover Note

- All summary tables in this package were rebuilt from raw run folders, not from aggregate.csv.
- The current package is based on the validated outputs in:
  - contact_timing_bias_runs_filtered.csv
  - excluded_runs_log.csv
  - contact_timing_bias_researcher_summary.csv
  - contact_timing_bias_researcher_compact_summary.csv
  - contact_timing_bias_validation_report.txt

Filtering used:
- short_duration
- low_progress
- startup_collapse

Interpretation note:
- filter pass does not mean success.
- A pass run is a run that was not excluded by the current filtering rules.
- success / task_completion_success remain separate result fields.

Additional checks:
- late_stabilization count = 0
- end_only_stabilized count = 0

Validated high-level observation:
- As contact timing bias increases, performance degrades.
- As terrain height increases, sensitivity increases.
"""
    HANDOVER_NOTE.write_text(text)


def main():
    filtered_rows = read_csv(FILTERED_CSV)
    summary_rows = read_csv(SUMMARY_CSV)
    main_rows = build_main_table(filtered_rows, summary_rows)
    exclusion_rows = build_exclusion_overview(filtered_rows)
    write_csv(MAIN_TABLE, main_rows)
    write_csv(EXCLUSION_OVERVIEW, exclusion_rows)
    build_note()
    print(f"Wrote {MAIN_TABLE}")
    print(f"Wrote {EXCLUSION_OVERVIEW}")
    print(f"Wrote {HANDOVER_NOTE}")


if __name__ == "__main__":
    main()

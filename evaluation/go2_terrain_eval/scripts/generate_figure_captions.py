#!/usr/bin/env python3
# Usage:
#   python3 /home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/scripts/generate_figure_captions.py
#
# Reads:
#   /home/ho/ros2_ws/filtered_results/main_table.csv
#   /home/ho/ros2_ws/filtered_results/exclusion_overview.csv
#   /home/ho/ros2_ws/filtered_results/figures/
#
# Writes:
#   /home/ho/ros2_ws/filtered_results/figures/figure_captions.txt
#   /home/ho/ros2_ws/filtered_results/figures/figure_captions_short.txt
#   /home/ho/ros2_ws/filtered_results/figures/figure_caption_generation_log.txt

import csv
import math
from pathlib import Path


MAIN_TABLE = Path("/home/ho/ros2_ws/filtered_results/main_table.csv")
EXCLUSION_OVERVIEW = Path("/home/ho/ros2_ws/filtered_results/exclusion_overview.csv")
FIG_DIR = Path("/home/ho/ros2_ws/filtered_results/figures")

LONG_OUT = FIG_DIR / "figure_captions.txt"
SHORT_OUT = FIG_DIR / "figure_captions_short.txt"
LOG_OUT = FIG_DIR / "figure_caption_generation_log.txt"

FIGURES = [
    (1, "success_rate_vs_timing_bias", "Success Rate vs Contact Timing Bias"),
    (2, "stable_rate_vs_timing_bias", "Stable Rate vs Contact Timing Bias"),
    (3, "fail_rate_vs_timing_bias", "Fail Rate vs Contact Timing Bias"),
    (4, "forward_progress_vs_timing_bias", "Forward Progress vs Contact Timing Bias"),
    (5, "roll_rms_vs_timing_bias", "Roll RMS vs Contact Timing Bias"),
    (6, "pitch_rms_vs_timing_bias", "Pitch RMS vs Contact Timing Bias"),
    (7, "success_rate_heatmap", "Success Rate Heatmap"),
    (8, "stable_rate_heatmap", "Stable Rate Heatmap"),
    (9, "fail_rate_heatmap", "Fail Rate Heatmap"),
]


def safe_float(v):
    if v in (None, "", "nan", "NaN"):
        return math.nan
    try:
        return float(v)
    except Exception:
        return math.nan


def load_tables():
    with MAIN_TABLE.open(newline="") as f:
        main_rows = list(csv.DictReader(f))
    with EXCLUSION_OVERVIEW.open(newline="") as f:
        exclusion_rows = list(csv.DictReader(f))

    for row in main_rows:
        for key in [
            "terrain_height_cm",
            "contact_timing_bias_s",
            "n_total_runs",
            "n_pass_runs",
            "n_excluded_runs",
            "success_rate",
            "stable_rate",
            "fail_rate",
            "forward_progress_mean",
            "roll_rms_deg_mean",
            "pitch_rms_deg_mean",
        ]:
            row[key] = safe_float(row[key])

    for row in exclusion_rows:
        for key in [
            "terrain_height_cm",
            "contact_timing_bias_s",
            "n_short_duration",
            "n_low_progress",
            "n_startup_collapse",
            "n_total_excluded",
        ]:
            row[key] = safe_float(row[key])

    return main_rows, exclusion_rows


def check_figure_files():
    found = []
    missing = []
    for _, base, _ in FIGURES:
        png = FIG_DIR / f"{base}.png"
        pdf = FIG_DIR / f"{base}.pdf"
        if png.exists():
            found.append(str(png))
        else:
            missing.append(str(png))
        if pdf.exists():
            found.append(str(pdf))
        else:
            missing.append(str(pdf))
    return found, missing


def rows_by_terrain(rows):
    out = {}
    for row in rows:
        t = row["terrain_height_cm"]
        out.setdefault(t, []).append(row)
    for t in out:
        out[t].sort(key=lambda r: r["contact_timing_bias_s"])
    return out


def summarize_main_trends(main_rows):
    terrain_map = rows_by_terrain(main_rows)

    success_drop = []
    stable_drop = []
    fail_rise = []
    fwd_drop = []
    roll_rise = []
    pitch_rise = []

    for terrain, rows in sorted(terrain_map.items()):
        valid = [r for r in rows if not math.isnan(r["success_rate"])]
        if len(valid) < 2:
            continue
        first = valid[0]
        last = valid[-1]

        if last["success_rate"] < first["success_rate"]:
            success_drop.append(int(terrain))
        if last["stable_rate"] < first["stable_rate"]:
            stable_drop.append(int(terrain))
        if last["fail_rate"] > first["fail_rate"]:
            fail_rise.append(int(terrain))
        if (
            not math.isnan(first["forward_progress_mean"])
            and not math.isnan(last["forward_progress_mean"])
            and last["forward_progress_mean"] < first["forward_progress_mean"]
        ):
            fwd_drop.append(int(terrain))
        if (
            not math.isnan(first["roll_rms_deg_mean"])
            and not math.isnan(last["roll_rms_deg_mean"])
            and last["roll_rms_deg_mean"] > first["roll_rms_deg_mean"]
        ):
            roll_rise.append(int(terrain))
        if (
            not math.isnan(first["pitch_rms_deg_mean"])
            and not math.isnan(last["pitch_rms_deg_mean"])
            and last["pitch_rms_deg_mean"] > first["pitch_rms_deg_mean"]
        ):
            pitch_rise.append(int(terrain))

    return {
        "success_drop": success_drop,
        "stable_drop": stable_drop,
        "fail_rise": fail_rise,
        "fwd_drop": fwd_drop,
        "roll_rise": roll_rise,
        "pitch_rise": pitch_rise,
    }


def summarize_exclusions(exclusion_rows):
    total_short = sum(int(r["n_short_duration"]) for r in exclusion_rows if not math.isnan(r["n_short_duration"]))
    total_low = sum(int(r["n_low_progress"]) for r in exclusion_rows if not math.isnan(r["n_low_progress"]))
    total_collapse = sum(int(r["n_startup_collapse"]) for r in exclusion_rows if not math.isnan(r["n_startup_collapse"]))
    return {
        "short_duration": total_short,
        "low_progress": total_low,
        "startup_collapse": total_collapse,
    }


def build_long_caption(index, base_name, title, trends, excl):
    if base_name == "success_rate_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure shows the success rate as a function of contact timing bias for each terrain height. "
            "Across the tested terrains, success rate generally decreases as the timing bias increases, although the exact transition is terrain-dependent. "
            "Higher step-down heights tend to show degradation at smaller timing biases than the lower-height cases."
        )
    if base_name == "stable_rate_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure reports the fraction of runs classified as stable under the current filtering and outcome rules. "
            "Stable locomotion is maintained over a wider timing-bias range on the lower terrains, while the higher terrains show a narrower stable region. "
            "The decline is not perfectly monotonic for every terrain, so the trend should be interpreted as a boundary shift rather than a strict linear decrease."
        )
    if base_name == "fail_rate_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure shows the fraction of runs classified as failures as the timing bias increases. "
            "Failure-dominant regions emerge earlier on the larger step-down terrains, indicating stronger sensitivity to contact timing mismatch under more difficult terrain conditions. "
            "The displayed rates are based on the validated run-level reconstruction from raw runs."
        )
    if base_name == "forward_progress_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure summarizes the mean forward progress of the non-excluded runs for each terrain-height and timing-bias combination. "
            "Forward progress remains high over a limited bias range and then drops as the timing bias increases. "
            "This reduction becomes more pronounced at the larger terrain heights."
        )
    if base_name == "roll_rms_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure reports the mean roll RMS over the non-excluded runs. "
            "Roll variation tends to increase as the timing bias grows, especially near the degrading and failure regions. "
            "The increase is more noticeable on the larger terrain heights."
        )
    if base_name == "pitch_rms_vs_timing_bias":
        return (
            f"Figure {index}. {title}\n"
            "This figure reports the mean pitch RMS over the non-excluded runs. "
            "Pitch variation also increases with timing bias and becomes substantially larger near the boundary between stable and failing behavior. "
            "This tendency is stronger for the higher step-down terrains."
        )
    if base_name == "success_rate_heatmap":
        return (
            f"Figure {index}. {title}\n"
            "This heatmap provides a compact overview of success rate over the terrain-height and timing-bias grid. "
            "It highlights the shrinking success region as the timing bias increases and as the terrain becomes more challenging. "
            "Blank or missing entries correspond to combinations without usable pass runs for plotting."
        )
    if base_name == "stable_rate_heatmap":
        return (
            f"Figure {index}. {title}\n"
            "This heatmap visualizes the stable-rate distribution over the full terrain-height and timing-bias grid. "
            "The stable region is broad at low terrain heights and progressively contracts as terrain height increases. "
            "The transition is gradual in some regions and abrupt in others, depending on the terrain-bias combination."
        )
    if base_name == "fail_rate_heatmap":
        return (
            f"Figure {index}. {title}\n"
            "This heatmap shows where failures become dominant in the terrain-height and timing-bias plane. "
            "Higher timing bias and larger terrain height jointly increase the failure rate, revealing the regions in which nominal contact timing becomes insufficiently robust. "
            "These rates are reported from the validated run-level tables used in the handover package."
        )
    return f"Figure {index}. {title}\nCaption unavailable."


def build_short_caption(index, base_name):
    mapping = {
        "success_rate_vs_timing_bias": "Success rate decreases as contact timing bias increases, with higher terrains degrading earlier.",
        "stable_rate_vs_timing_bias": "The stable locomotion region shrinks as timing bias and terrain difficulty increase.",
        "fail_rate_vs_timing_bias": "Failure rate rises with timing bias and becomes larger on higher terrains.",
        "forward_progress_vs_timing_bias": "Mean forward progress drops as contact timing bias increases.",
        "roll_rms_vs_timing_bias": "Roll variation grows near the degrading and failure regions.",
        "pitch_rms_vs_timing_bias": "Pitch variation increases as timing bias moves toward the unstable region.",
        "success_rate_heatmap": "The success-rate heatmap shows a contracting feasible region at larger timing bias and terrain height.",
        "stable_rate_heatmap": "The stable-rate heatmap highlights the narrowing stable region across terrain and timing bias.",
        "fail_rate_heatmap": "The fail-rate heatmap shows where failure-dominant regions emerge in the terrain-bias grid.",
    }
    return f"Figure {index}. {mapping[base_name]}"


def write_caption_files(long_texts, short_texts):
    LONG_OUT.write_text("\n\n".join(long_texts) + "\n")
    SHORT_OUT.write_text("\n".join(short_texts) + "\n")


def write_caption_log(found, missing):
    lines = ["found_figure_files:"]
    lines.extend(found)
    lines.append("")
    lines.append("missing_figure_files:")
    lines.extend(missing)
    lines.append("")
    lines.append(f"caption_file_long: {LONG_OUT}")
    lines.append(f"caption_file_short: {SHORT_OUT}")
    LOG_OUT.write_text("\n".join(lines) + "\n")


def main():
    main_rows, exclusion_rows = load_tables()
    found, missing = check_figure_files()
    trends = summarize_main_trends(main_rows)
    excl = summarize_exclusions(exclusion_rows)

    long_texts = []
    short_texts = []
    for index, base_name, title in FIGURES:
        long_texts.append(build_long_caption(index, base_name, title, trends, excl))
        short_texts.append(build_short_caption(index, base_name))

    write_caption_files(long_texts, short_texts)
    write_caption_log(found, missing)
    print(f"Wrote {LONG_OUT}")
    print(f"Wrote {SHORT_OUT}")
    print(f"Wrote {LOG_OUT}")


if __name__ == "__main__":
    main()

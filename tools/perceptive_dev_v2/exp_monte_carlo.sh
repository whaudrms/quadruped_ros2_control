#!/usr/bin/env bash
# Paired Monte Carlo experiment for robust-phase evaluation.
#
# For every sampled terrain perception error, run Robust ON and OFF with the
# same sample. The first mode is randomized per pair to reduce run-order bias.
# Samples are generated once in samples.csv and reused when the batch resumes.
# To append samples, keep RESULTS_DIR and MASTER_SEED and increase N_SAMPLES
# to the desired total. Existing samples are verified and backed up first.
#
# Default uncertainty model:
#   terrain_z_offset ~ Uniform(-0.05, +0.05) m
# MASTER_SEED reproduces the sampled offsets and ON/OFF ordering. The current
# run_trial.py has no MuJoCo RNG seed option, so it does not seed simulator
# internals such as contact dynamics or scheduler timing.
#
# OCP parameters default to task.info: robustPhase (d, bounds, weights,
# hard/slack boundaries, splice, logging), mpc.mpcDesiredFrequency, and
# sqp.sqpIteration. Only robustPhase.enabled is toggled for the paired trials.
# v_max is derived as 2*d_max/(t_a+t_b) before runtime liftoff clamping.
# Optional ROBUST_T_A/ROBUST_T_B explicitly override task.info timing for a batch.
# Use a new RESULTS_DIR when changing parameters.
# Examples:
#   ROBUST_T_A=0.05 ROBUST_T_B=0.05 DRY_RUN=1 N_SAMPLES=1 \
#     bash tools/perceptive_dev_v2/exp_monte_carlo.sh
#   DRY_RUN=1 N_SAMPLES=2 bash tools/perceptive_dev_v2/exp_monte_carlo.sh
#   N_SAMPLES=50 MASTER_SEED=20260831 \
#     bash tools/perceptive_dev_v2/exp_monte_carlo.sh
#   RESULTS_DIR=/tmp/go2_mc N_SAMPLES=10 \
#     bash tools/perceptive_dev_v2/exp_monte_carlo.sh

set -Eeuo pipefail

cd "$(dirname "$0")/../.."

N_SAMPLES="${N_SAMPLES:-100}"
MASTER_SEED="${MASTER_SEED:-20260910}"
OFFSET_MIN="${OFFSET_MIN:--0.05}"
OFFSET_MAX="${OFFSET_MAX:-0.05}"
RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/monte_carlo_n${N_SAMPLES}_seed${MASTER_SEED}}"
DDS_COOLDOWN_SEC="${DDS_COOLDOWN_SEC:-3}"
DRY_RUN="${DRY_RUN:-0}"
# Empty values inherit task.info; optional per-batch offsets are seconds.
ROBUST_T_A="${ROBUST_T_A:-}"
ROBUST_T_B="${ROBUST_T_B:-}"
MANIFEST="$RESULTS_DIR/samples.csv"
failures=0

if ! [[ "$N_SAMPLES" =~ ^[1-9][0-9]*$ ]]; then
    echo "[monte_carlo] N_SAMPLES must be a positive integer: $N_SAMPLES" >&2
    exit 2
fi
if ! [[ "$MASTER_SEED" =~ ^-?[0-9]+$ ]]; then
    echo "[monte_carlo] MASTER_SEED must be an integer: $MASTER_SEED" >&2
    exit 2
fi
if [[ "$DRY_RUN" != "0" && "$DRY_RUN" != "1" ]]; then
    echo "[monte_carlo] DRY_RUN must be 0 or 1: $DRY_RUN" >&2
    exit 2
fi

mkdir -p "$RESULTS_DIR"

# Generate the complete design before launching any trial. If a manifest is
# already present, validate and reuse it so an interrupted experiment resumes
# with exactly the same samples and pair ordering.
python3 - "$MANIFEST" "$N_SAMPLES" "$MASTER_SEED" "$OFFSET_MIN" "$OFFSET_MAX" <<'PY'
import csv
import os
import random
import shutil
import sys
import time
from pathlib import Path

manifest = Path(sys.argv[1])
n_samples = int(sys.argv[2])
master_seed = int(sys.argv[3])
offset_min = float(sys.argv[4])
offset_max = float(sys.argv[5])

if not offset_min < offset_max:
    raise SystemExit(
        f"OFFSET_MIN must be smaller than OFFSET_MAX: {offset_min} >= {offset_max}"
    )

fieldnames = [
    "sample_id",
    "sample_seed",
    "terrain_z_offset",
    "first_mode",
    "master_seed",
    "offset_min",
    "offset_max",
]

rows = []
if manifest.exists():
    with manifest.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if reader.fieldnames != fieldnames:
        raise SystemExit(f"existing manifest has an incompatible header: {manifest}")
    if len(rows) > n_samples:
        raise SystemExit(
            f"existing manifest contains {len(rows)} samples, requested {n_samples}: "
            f"N_SAMPLES cannot shrink an existing batch; choose another RESULTS_DIR"
        )
    for row in rows:
        if int(row["master_seed"]) != master_seed:
            raise SystemExit(
                f"existing manifest uses MASTER_SEED={row['master_seed']}, requested "
                f"{master_seed}: use its original seed or choose another RESULTS_DIR"
            )
        if float(row["offset_min"]) != offset_min or float(row["offset_max"]) != offset_max:
            raise SystemExit(
                "existing manifest uses a different offset range: use its original "
                "OFFSET_MIN/OFFSET_MAX or choose another RESULTS_DIR"
            )
    if len(rows) == n_samples:
        print(f"[monte_carlo] reusing manifest: {manifest}")
        raise SystemExit(0)

manifest.parent.mkdir(parents=True, exist_ok=True)
temporary = manifest.with_suffix(manifest.suffix + ".tmp")
master_rng = random.Random(master_seed)

generated = []
for sample_id in range(1, n_samples + 1):
    sample_seed = master_rng.randrange(2**32)
    sample_rng = random.Random(sample_seed)
    offset = sample_rng.uniform(offset_min, offset_max)
    first_mode = "on" if sample_rng.getrandbits(1) == 0 else "off"
    generated.append({
        "sample_id": str(sample_id),
        "sample_seed": str(sample_seed),
        "terrain_z_offset": f"{offset:.8f}",
        "first_mode": first_mode,
        "master_seed": str(master_seed),
        "offset_min": f"{offset_min:.8f}",
        "offset_max": f"{offset_max:.8f}",
    })

if rows != generated[:len(rows)]:
    raise SystemExit("existing samples differ from the seeded sequence; refusing to change them")

with temporary.open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(generated)

if manifest.exists():
    backup = manifest.with_name(f"samples.before_extend_{time.time_ns()}.csv")
    shutil.copy2(manifest, backup)
    print(f"[monte_carlo] original manifest backup: {backup}")
os.replace(temporary, manifest)
print(f"[monte_carlo] manifest: {manifest}; samples {len(rows)} -> {n_samples}")
PY

trial_exists() {
    local tag="$1"
    local trial_dir
    while IFS= read -r trial_dir; do
        if [[ -f "$trial_dir/result.json" ]]; then
            return 0
        fi
    done < <(
        find "$RESULTS_DIR" -mindepth 1 -maxdepth 1 -type d \
            -name "*_${tag}" -print 2>/dev/null
    )
    return 1
}

run_one() {
    local sample_id="$1"
    local sample_seed="$2"
    local offset="$3"
    local robust="$4"
    local padded_id tag
    local -a run_mode_args=()

    printf -v padded_id '%04d' "$sample_id"
    tag="mc_s${padded_id}_seed${sample_seed}_${robust}"

    if [[ "$DRY_RUN" != "1" ]] && trial_exists "$tag"; then
        echo "[monte_carlo] SKIP existing trial: $tag"
        return 0
    fi

    echo
    echo "=========================================================="
    echo "[monte_carlo] sample=$sample_id seed=$sample_seed robust=$robust offset=$offset"
    echo "=========================================================="

    if [[ "$DRY_RUN" == "1" ]]; then
        run_mode_args+=(--dry-run)
    fi

    if [[ -n "$ROBUST_T_A" ]]; then
        run_mode_args+=(--robust-t-a "$ROBUST_T_A")
    fi
    if [[ -n "$ROBUST_T_B" ]]; then
        run_mode_args+=(--robust-t-b "$ROBUST_T_B")
    fi

    # Omit task.info override flags so edits to the OCP configuration take effect.
    # Scenario, terrain, uncertainty sampling and runner options remain here.
    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --terrain-z-offset "$offset" \
        --terrain-z-offset-only-below-z 0.15 \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        "${run_mode_args[@]}" \
        --tag "$tag"; then
        echo "[monte_carlo] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi

    if [[ "$DRY_RUN" != "1" ]]; then
        echo "[monte_carlo] DDS cooldown: ${DDS_COOLDOWN_SEC}s"
        sleep "$DDS_COOLDOWN_SEC"
    fi
}

# Run a paired ON/OFF comparison for every uncertainty sample. Both modes see
# the identical offset, while first_mode randomizes which one executes first.
while IFS=, read -r sample_id sample_seed offset first_mode \
        _manifest_seed _manifest_min _manifest_max; do
    if [[ "$first_mode" == "on" ]]; then
        run_one "$sample_id" "$sample_seed" "$offset" on
        run_one "$sample_id" "$sample_seed" "$offset" off
    else
        run_one "$sample_id" "$sample_seed" "$offset" off
        run_one "$sample_id" "$sample_seed" "$offset" on
    fi
done < <(tail -n +2 "$MANIFEST")

if [[ "$DRY_RUN" != "1" ]]; then
    echo
    echo "[monte_carlo] generating tables, plots, and all_visualizations"
    if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
        --results-dir "$RESULTS_DIR"; then
        echo "[monte_carlo] FAILED: all_visualizations" >&2
        failures=$((failures + 1))
    fi
fi

total_trials=$((2 * N_SAMPLES))
if [[ "$RESULTS_DIR" = /* ]]; then
    results_display="$RESULTS_DIR"
else
    results_display="$(pwd)/$RESULTS_DIR"
fi

echo
echo "[monte_carlo] all $total_trials paired trials attempted or resumed; failures=$failures"
echo "[monte_carlo] manifest: $results_display/samples.csv"
echo "[monte_carlo] results : $results_display"
echo "[monte_carlo] gallery : $results_display/all_visualizations/index.html"

if (( failures > 0 )); then
    exit 1
fi

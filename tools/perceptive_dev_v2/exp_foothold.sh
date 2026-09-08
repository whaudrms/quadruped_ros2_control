#!/usr/bin/env bash
# Paired single-experiment setup for explaining foothold prediction error.
#
# Scientific question:
#   With the same biased terrain map, does Robust ON handle the mismatch
#   between the perceived landing plane and the unknown physical contact plane
#   more gracefully than Robust OFF (perceptive nominal)?
#
# Default step-down condition:
#   true lower surface z      =  0.100 m
#   terrain_z_offset          = -0.048 m
#   perceived lower surface z =  0.052 m
#
# This intentionally uses a negative error. The nominal planner therefore
# expects the FL foot to descend below the true surface, while MuJoCo makes the
# foot contact the unchanged physical surface earlier than expected.
#
# The script runs exactly one paired comparison (two trials):
#   1. Robust OFF: perceptive nominal
#   2. Robust ON : proposed robust phase
# Both trials use the same scenario, terrain, offset, MPC settings, and robust
# parameters; only robustPhase.enabled changes. This is a mechanism case study,
# not a statistical performance experiment.
#
# Important generated data:
#   foothold_plan_snapshots.csv = every completed MPC solve's future optimized
#                                 state, swing z_ref, touchdown target, and robust
#                                 window. Earlier snapshots remain after contact.
#                                 FL z FK is evaluated offline by the plotter.
#   tick.csv                    = closed-loop measured state/contact outcome.
#   tick.csv opt_x              = current MPC policy state, NOT the saved
#                                 future foothold plan.
#
# Usage:
#   bash tools/perceptive_dev_v2/exp_foothold.sh
#   DRY_RUN=1 bash tools/perceptive_dev_v2/exp_foothold.sh
#   TERRAIN_Z_OFFSET=-0.050 bash tools/perceptive_dev_v2/exp_foothold.sh
#   ORDER=on-first bash tools/perceptive_dev_v2/exp_foothold.sh
#
# Results:
#   tools/perceptive_dev_v2/results/exp_foothold/<PAIR_ID>/

set -Eeuo pipefail

cd "$(dirname "$0")/../.."

TERRAIN_Z_OFFSET="${TERRAIN_Z_OFFSET:--0.048}"
ROBUST_D="${ROBUST_D:-0.05}"
PAIR_ID="${PAIR_ID:-$(date +%Y%m%d_%H%M%S)}"
RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/exp_foothold/${PAIR_ID}}"
ORDER="${ORDER:-off-first}"
DDS_COOLDOWN_SEC="${DDS_COOLDOWN_SEC:-3}"
DRY_RUN="${DRY_RUN:-0}"
failures=0

if [[ "$ORDER" != "off-first" && "$ORDER" != "on-first" ]]; then
    echo "[exp_foothold] ORDER must be off-first or on-first: $ORDER" >&2
    exit 2
fi
if [[ "$DRY_RUN" != "0" && "$DRY_RUN" != "1" ]]; then
    echo "[exp_foothold] DRY_RUN must be 0 or 1: $DRY_RUN" >&2
    exit 2
fi
if ! python3 - "$TERRAIN_Z_OFFSET" "$ROBUST_D" <<'PY'
import math
import sys

offset = float(sys.argv[1])
robust_d = float(sys.argv[2])
if not math.isfinite(offset):
    raise SystemExit("TERRAIN_Z_OFFSET must be finite")
if not math.isfinite(robust_d) or robust_d <= 0.0:
    raise SystemExit("ROBUST_D must be finite and positive")
if offset >= 0.0:
    raise SystemExit(
        "This experiment requires a negative TERRAIN_Z_OFFSET so physical "
        "contact occurs earlier/higher than the perceived landing plane"
    )
PY
then
    exit 2
fi

mkdir -p "$RESULTS_DIR"

run_one() {
    local robust="$1"
    local label tag
    local -a dry_run_args=()

    if [[ "$robust" == "off" ]]; then
        label="nominal"
    else
        label="proposed"
    fi
    tag="foothold_pair_${PAIR_ID}_${label}_dz${TERRAIN_Z_OFFSET}"

    echo
    echo "=========================================================="
    echo "[exp_foothold] pair=$PAIR_ID mode=$label robust=$robust"
    echo "[exp_foothold] terrain_z_offset=$TERRAIN_Z_OFFSET"
    echo "=========================================================="

    if [[ "$DRY_RUN" == "1" ]]; then
        dry_run_args+=(--dry-run)
    fi

    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-p 10 \
        --robust-d "$ROBUST_D" \
        --robust-v-max 0.6 \
        --robust-hard-boundary-start off \
        --robust-hard-boundary-end off \
        --robust-slack-boundary-start on \
        --robust-slack-boundary-end on \
        --robust-slack-weight-start 20 \
        --robust-slack-weight-end 20 \
        --robust-splice off \
        --robust-verbose on \
        --mpc-frequency 10 \
        --sqp-iterations 2 \
        --terrain-z-offset "$TERRAIN_Z_OFFSET" \
        --terrain-z-offset-only-below-z 0.15 \
        --foothold-plan-log on \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        "${dry_run_args[@]}" \
        --tag "$tag"; then
        echo "[exp_foothold] FAILED: $label" >&2
        failures=$((failures + 1))
    fi

    if [[ "$DRY_RUN" != "1" ]]; then
        echo "[exp_foothold] DDS cooldown: ${DDS_COOLDOWN_SEC}s"
        sleep "$DDS_COOLDOWN_SEC"
    fi
}

if [[ "$ORDER" == "off-first" ]]; then
    run_one off
    run_one on
else
    run_one on
    run_one off
fi

if [[ "$DRY_RUN" != "1" && "$failures" -eq 0 ]]; then
    echo
    echo "[exp_foothold] generating pre-contact policy vs actual-contact figure"
    if ! python3 tools/perceptive_dev_v2/plot_foothold_policy_vs_actual.py \
        --results-dir "$RESULTS_DIR"; then
        echo "[exp_foothold] FAILED: foothold policy visualization" >&2
        failures=$((failures + 1))
    fi
fi

if [[ "$RESULTS_DIR" = /* ]]; then
    results_display="$RESULTS_DIR"
else
    results_display="$(pwd)/$RESULTS_DIR"
fi

echo
echo "[exp_foothold] paired trials attempted; failures=$failures"
echo "[exp_foothold] results: $results_display"
echo "[exp_foothold] figure : $results_display/all_visualizations/fig_foothold_policy_vs_actual.png"
echo "[exp_foothold] plot these as separate quantities:"
echo "  pre-contact MPC plan  -> foothold_plan_snapshots.csv: opt_x0..opt_x23 (offline FL FK)"
echo "  planner reference     -> foothold_plan_snapshots.csv: z_ref"
echo "  actual trajectory     -> tick.csv: meas_rbd + measured_mode"
echo "  do not label tick.csv opt_x as the predicted foothold target"

if (( failures > 0 )); then
    exit 1
fi

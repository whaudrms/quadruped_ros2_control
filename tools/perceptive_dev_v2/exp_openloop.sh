#!/usr/bin/env bash
# Paired frozen-policy experiment for the FL step-down planning mechanism.
#
# This is an open-loop *planning* comparison, not open-loop robot execution:
# each controller still runs closed loop long enough to produce a pre-contact
# MPC policy, then plot_openloop_policy_comparison.py freezes one completed
# solve and plots its full available FL swing prediction without later replans.
#
# Both selected policies must target the same biased lower plane.  The plotter
# also checks base pose, joint pose, all-foot positions, swing duration, and
# touchdown target at the two policy-start states.  It refuses to make the
# figure when the independently realized states are not sufficiently matched.
# A true identical-x0 counterfactual solve requires an offline MPC replay API,
# which the controller does not currently expose.
#
# Default geometry (basic_step_short_v2):
#   true lower surface               =  0.100 m
#   perception-only terrain offset   = -0.048 m
#   perceived lower surface          =  0.052 m
#   FL foot-frame offset             =  0.060 m
#   true/perceived FL contact levels =  0.160 / 0.112 m
#
# Usage:
#   bash tools/perceptive_dev_v2/exp_openloop.sh
#   DRY_RUN=1 bash tools/perceptive_dev_v2/exp_openloop.sh
#   ORDER=on-first bash tools/perceptive_dev_v2/exp_openloop.sh
#
# Diagnostic escape hatch (the output JSON records the failed checks):
#   ALLOW_STATE_MISMATCH=1 bash tools/perceptive_dev_v2/exp_openloop.sh
#
# Results:
#   tools/perceptive_dev_v2/results/exp_openloop/<PAIR_ID>/

set -Eeuo pipefail

cd "$(dirname "$0")/../.."

TERRAIN_Z_OFFSET="${TERRAIN_Z_OFFSET:--0.048}"
ROBUST_D="${ROBUST_D:-0.05}"
PAIR_ID="${PAIR_ID:-$(date +%Y%m%d_%H%M%S)}"
RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/exp_openloop/${PAIR_ID}}"
ORDER="${ORDER:-off-first}"
DDS_COOLDOWN_SEC="${DDS_COOLDOWN_SEC:-3}"
DRY_RUN="${DRY_RUN:-0}"
ALLOW_STATE_MISMATCH="${ALLOW_STATE_MISMATCH:-0}"
failures=0
unmatched=0

if [[ "$ORDER" != "off-first" && "$ORDER" != "on-first" ]]; then
    echo "[exp_openloop] ORDER must be off-first or on-first: $ORDER" >&2
    exit 2
fi
for flag_name in DRY_RUN ALLOW_STATE_MISMATCH; do
    flag_value="${!flag_name}"
    if [[ "$flag_value" != "0" && "$flag_value" != "1" ]]; then
        echo "[exp_openloop] $flag_name must be 0 or 1: $flag_value" >&2
        exit 2
    fi
done
if ! python3 - "$TERRAIN_Z_OFFSET" "$ROBUST_D" <<'PY'
import math
import sys

offset = float(sys.argv[1])
robust_d = float(sys.argv[2])
if not math.isfinite(offset) or offset >= 0.0:
    raise SystemExit("TERRAIN_Z_OFFSET must be finite and negative")
if not math.isfinite(robust_d) or robust_d <= 0.0:
    raise SystemExit("ROBUST_D must be finite and positive")
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
    tag="openloop_pair_${PAIR_ID}_${label}_dz${TERRAIN_Z_OFFSET}"

    echo
    echo "=========================================================="
    echo "[exp_openloop] pair=$PAIR_ID mode=$label robust=$robust"
    echo "[exp_openloop] terrain_z_offset=$TERRAIN_Z_OFFSET"
    echo "=========================================================="

    if [[ "$DRY_RUN" == "1" ]]; then
        dry_run_args+=(--dry-run)
    fi

    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-d "$ROBUST_D" \
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
        echo "[exp_openloop] FAILED: $label" >&2
        failures=$((failures + 1))
    fi

    if [[ "$DRY_RUN" != "1" ]]; then
        echo "[exp_openloop] DDS cooldown: ${DDS_COOLDOWN_SEC}s"
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
    strict_output="$RESULTS_DIR/all_visualizations/fig_openloop_policy_comparison.png"
    diagnostic_output="$RESULTS_DIR/all_visualizations/fig_openloop_policy_comparison_unmatched.png"
    plot_args=(
        --results-dir "$RESULTS_DIR"
        --output "$strict_output"
    )
    if [[ "$ALLOW_STATE_MISMATCH" == "1" ]]; then
        plot_args+=(--allow-state-mismatch)
    fi
    echo
    echo "[exp_openloop] validating and plotting frozen policies"
    if ! python3 tools/perceptive_dev_v2/plot_openloop_policy_comparison.py \
        "${plot_args[@]}"; then
        echo "[exp_openloop] strict state match failed; generating a warned diagnostic figure" >&2
        if ! python3 tools/perceptive_dev_v2/plot_openloop_policy_comparison.py \
            --results-dir "$RESULTS_DIR" \
            --output "$diagnostic_output" \
            --allow-state-mismatch; then
            echo "[exp_openloop] FAILED: diagnostic figure generation" >&2
            failures=$((failures + 1))
        else
            echo "[exp_openloop] diagnostic: $diagnostic_output"
            unmatched=1
        fi
    fi
fi

if [[ "$RESULTS_DIR" = /* ]]; then
    results_display="$RESULTS_DIR"
else
    results_display="$(pwd)/$RESULTS_DIR"
fi

echo
echo "[exp_openloop] paired trials attempted; failures=$failures unmatched=$unmatched"
echo "[exp_openloop] results: $results_display"
echo "[exp_openloop] figure : $results_display/all_visualizations/fig_openloop_policy_comparison.png"
echo "[exp_openloop] unmatched diagnostic (when needed): $results_display/all_visualizations/fig_openloop_policy_comparison_unmatched.png"
echo "[exp_openloop] report : $results_display/all_visualizations/fig_openloop_policy_comparison.json"
echo "[exp_openloop] unmatched report (when needed): $results_display/all_visualizations/fig_openloop_policy_comparison_unmatched.json"

if (( failures > 0 )); then
    exit 1
fi

#!/usr/bin/env bash
# Critical decisive cell from chat5.md A2 verdict:
#   Δz = -0.02, d = 0.03  (band condition: z_actual=0.10 ∈ [0.05, 0.11])
#   ↔ robust phase's design promise SHOULD hold here
#
# 2 trials: robust ON vs OFF, MPC=10Hz, basic_step_short.
# If robust ON does NOT beat robust OFF (impact velocity / roll / fall) here,
# chat5 says the issue is formulation/weight/implementation, not sweep params.

set -u

cd "$(dirname "$0")/../.."
RESULTS_DIR="tools/perceptive_dev_v2/results/critical_band_inside"
failures=0

run_one() {
    local robust="$1" tag="$2"
    echo
    echo "=========================================================="
    echo "[crit] tag=$tag  robust=$robust  off=-0.02  d=0.03"
    echo "=========================================================="
    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_short \
        --terrain basic_step_short \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-d 0.03 \
        --robust-splice on \
        --robust-verbose on \
        --mpc-frequency 10 \
        --terrain-z-offset -0.02 \
        --terrain-z-offset-only-below-z 0.15 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        --tag "$tag"; then
        echo "[crit] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi
    echo "[crit] DDS cooldown: 3s"
    sleep 3
}

run_one on  crit_band_robON_offM02
run_one off crit_band_robOFF_offM02

echo
echo "[crit] generating all_visualizations"
if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
    --results-dir "$RESULTS_DIR"; then
    echo "[crit] FAILED: all_visualizations" >&2
    failures=$((failures + 1))
fi

echo
echo "[crit] both trials attempted; failures=$failures"
echo "[crit] results: $(pwd)/$RESULTS_DIR"
echo "[crit] gallery: $(pwd)/$RESULTS_DIR/all_visualizations/index.html"
exit "$failures"

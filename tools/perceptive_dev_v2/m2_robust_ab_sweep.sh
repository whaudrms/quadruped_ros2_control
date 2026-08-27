#!/usr/bin/env bash
# M2 robust phase A/B sweep at low MPC rate (per chat4.md recommendation).
#
# 6 trials: 2 (robust on/off) × 3 (terrain_z_offset on box2 only: 0, +0.05, -0.05).
# MPC at 10 Hz (vs M2 default 50 Hz) to expose contact-timing mismatch over
# longer windows; sqp.dt unchanged at 0.02 s (per (A) decision).
#
# terrain_z_offset is restricted to surfaces with true top z < 0.15 m, which on
# basic_step_short selects ONLY box2 (z=0.10), leaving box1 (z=0.20) unchanged.
#
# Physical interpretation (corrected from an earlier draft — earlier comment
# had the signs reversed; see WithoutSplice.md "Physical interpretation" table
# and Findings F0/F2):
#   +0.05  perception sees box2 5cm HIGHER than reality (z=0.15 vs actual 0.10).
#          Robust ON foot-frame target at t_b = perceived + offset_foot − d
#          = 0.18 m, which is 2 cm ABOVE the natural touchdown 0.16 m. Foot
#          trajectory ends ABOVE the actual ground at scheduled t_b → no contact
#          at t_b → LATE / missed contact (late events dominant in the log).
#   -0.05  perception sees box2 5cm LOWER than reality (z=0.05 vs actual 0.10).
#          Robust ON foot-frame target = 0.08 m = 8 cm BELOW natural touchdown
#          0.16 m. Foot trajectory aims through the actual ground; foot meets
#          actual ground well before scheduled t_b → HARD EARLY contact.
#
# Run from quadruped_ros2_control/ root:
#   bash tools/perceptive_dev_v2/m2_robust_ab_sweep.sh

set -u  # Keep attempting the matrix and report a non-zero result at the end.

cd "$(dirname "$0")/../.."
RESULTS_DIR="tools/perceptive_dev_v2/results/robust_ab_sweep"
failures=0

run_one() {
    local robust="$1" off="$2" tag="$3"
    echo
    echo "=========================================================="
    echo "[sweep] tag=$tag  robust=$robust  offset=$off"
    echo "=========================================================="
    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward \
        --terrain basic_step_short \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-p 10 \
        --robust-d 0.03 \
        --robust-v-max 0.6 \
        --robust-splice on \
        --robust-verbose on \
        --mpc-frequency 10 \
        --terrain-z-offset "$off" \
        --terrain-z-offset-only-below-z 0.15 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        --tag "$tag"; then
        echo "[sweep] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi
    echo "[sweep] DDS cooldown: 3s"
    sleep 3
}

# 6-trial matrix
run_one on   0.00  m2ab10_robON_off0
run_one on   0.05  m2ab10_robON_offP05
run_one on  -0.05  m2ab10_robON_offM05
run_one off  0.00  m2ab10_robOFF_off0
run_one off  0.05  m2ab10_robOFF_offP05
run_one off -0.05  m2ab10_robOFF_offM05

echo
echo "[sweep] generating all_visualizations"
if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
    --results-dir "$RESULTS_DIR"; then
    echo "[sweep] FAILED: all_visualizations" >&2
    failures=$((failures + 1))
fi

echo
echo "[sweep] all 6 trials attempted; failures=$failures"
echo "[sweep] results: $(pwd)/$RESULTS_DIR"
echo "[sweep] gallery: $(pwd)/$RESULTS_DIR/all_visualizations/index.html"
exit "$failures"

#!/usr/bin/env bash
# Run the good_data v2 ablation with n=10 repetitions per experiment cell.
#
# Matrix:
#   terrain_z_offset {-0.03, +0.03}
#   x {robust ON with splice OFF, robust OFF}
#   x 10 repetitions
#
# Robust ON formulation:
#   boundary cost retained, hard start/end OFF,
#   quadratic-slack start/end ON with weights 20/20.
# Solver configuration:
#   MPC 10 Hz, maximum 2 SQP iterations per solve. Actual rolling
#   last/average/maximum iteration counts are logged and plotted per trial.
#
# Results:
#   tools/perceptive_dev_v2/results/exp/

set -u

cd "$(dirname "$0")/../.."

RESULTS_DIR="tools/perceptive_dev_v2/results/exp"
REPETITIONS=50
failures=0

run_one() {
    local robust="$1"
    local offset="$2"
    local tag="$3"

    echo
    echo "=========================================================="
    echo "[exp] tag=$tag robust=$robust offset=$offset"
    echo "=========================================================="

    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-p 10 \
        --robust-d 0.05 \
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
        --terrain-z-offset "$offset" \
        --terrain-z-offset-only-below-z 0.15 \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        --tag "$tag"; then
        echo "[exp] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi

    echo "[exp] DDS cooldown: 3s"
    sleep 3
}

# Preserve the original interleaved ON/OFF order for the -0.03 m cell.
for run in $(seq 1 "$REPETITIONS"); do
    run_one on -0.03 \
        "exp_n10_ON_nosplice_offM03_d05_P10_sqp2_run${run}"
    run_one off -0.03 \
        "exp_n10_OFF_offM03_d05_P10_sqp2_run${run}"
done

# Preserve the original interleaved ON/OFF order for the +0.03 m cell.
for run in $(seq 1 "$REPETITIONS"); do
    run_one on 0.03 \
        "exp_n10_p03_ON_nosplice_offP03_d05_P10_sqp2_run${run}"
    run_one off 0.03 \
        "exp_n10_p03_OFF_offP03_d05_P10_sqp2_run${run}"
done

echo
echo "[exp] generating all_visualizations"
if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
    --results-dir "$RESULTS_DIR"; then
    echo "[exp] FAILED: all_visualizations" >&2
    failures=$((failures + 1))
fi

total_trials=$((2 * 2 * REPETITIONS))
echo
echo "[exp] all ${total_trials} trials attempted; failures=$failures"
echo "[exp] results: $(pwd)/$RESULTS_DIR"
echo "[exp] gallery: $(pwd)/$RESULTS_DIR/all_visualizations/index.html"
exit "$failures"

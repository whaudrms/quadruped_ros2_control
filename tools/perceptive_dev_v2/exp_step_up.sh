#!/usr/bin/env bash
# Step-up counterpart of exp.sh with the same experiment conditions.
#
# Physical terrain:
#   0.10 m step up at x=0.60 m
# Initial pose:
#   base z=0.20 m above the z=0.00 start floor. This preserves exp.sh's
#   start-surface-relative spawn height (base z=0.40 over a z=0.20 platform).
# Swing reference: 0.12 m clearance by default (override with SWING_HEIGHT).
#
# Matrix:
#   landing-surface terrain_z_offset {-0.03, +0.03}
#   x {robust ON with splice OFF, robust OFF}
#   x 50 repetitions
#
# Robust ON formulation:
#   boundary cost retained, hard start/end OFF,
#   quadratic-slack start/end ON with weights 20/20.
# Solver configuration:
#   MPC 20 Hz, maximum 2 SQP iterations per solve. Actual rolling
#   last/average/maximum iteration counts are logged and plotted per trial.
#
# Results:
#   tools/perceptive_dev_v2/results/exp_step_up/
# Append 50 repetitions per condition: START_RUN=51 REPETITIONS=50 bash tools/perceptive_dev_v2/exp_step_up.sh

set -u

cd "$(dirname "$0")/../.."

RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/exp_step_up}"
START_RUN="${START_RUN:-1}"
REPETITIONS="${REPETITIONS:-50}"
SWING_HEIGHT="${SWING_HEIGHT:-0.12}"
failures=0

if ! [[ "$START_RUN" =~ ^[1-9][0-9]*$ && "$REPETITIONS" =~ ^[1-9][0-9]*$ ]]; then
    echo "START_RUN and REPETITIONS must be positive integers without leading zeros" >&2
    exit 2
fi
END_RUN=$((START_RUN + REPETITIONS - 1))
echo "[exp_step_up] run range: $START_RUN..$END_RUN per condition"

run_one() {
    local robust="$1"
    local offset="$2"
    local tag="$3"

    echo
    echo "=========================================================="
    echo "[exp_step_up] tag=$tag robust=$robust offset=$offset swing_height=$SWING_HEIGHT"
    echo "=========================================================="

     if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_up_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --terrain-z-offset "$offset" \
        --terrain-z-offset-only-below-z 0.03 \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        "${run_mode_args[@]}" \
        --tag "$tag"; then
        echo "[monte_carlo] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi


    echo "[exp_step_up] DDS cooldown: 3s"
    sleep 3
}

# Preserve exp.sh's interleaved ON/OFF order for the -0.03 m cell.
for run in $(seq "$START_RUN" "$END_RUN"); do
    run_one on -0.03 \
        "exp_step_up_n50_ON_nosplice_offM03_d05_P10_sqp2_run${run}"
    run_one off -0.03 \
        "exp_step_up_n50_OFF_offM03_d05_P10_sqp2_run${run}"
done

# Preserve exp.sh's interleaved ON/OFF order for the +0.03 m cell.
for run in $(seq "$START_RUN" "$END_RUN"); do
    run_one on 0.03 \
        "exp_step_up_n50_p03_ON_nosplice_offP03_d05_P10_sqp2_run${run}"
    run_one off 0.03 \
        "exp_step_up_n50_p03_OFF_offP03_d05_P10_sqp2_run${run}"
done

echo
echo "[exp_step_up] generating all_visualizations"
if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
    --results-dir "$RESULTS_DIR"; then
    echo "[exp_step_up] FAILED: all_visualizations" >&2
    failures=$((failures + 1))
fi

total_trials=$((2 * 2 * REPETITIONS))
echo
echo "[exp_step_up] all ${total_trials} trials attempted; failures=$failures"
echo "[exp_step_up] results: $(pwd)/$RESULTS_DIR"
echo "[exp_step_up] gallery: $(pwd)/$RESULTS_DIR/all_visualizations/index.html"
exit "$failures"

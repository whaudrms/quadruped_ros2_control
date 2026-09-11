#!/usr/bin/env bash
# Reproduce the 2026-05-10 good_data v2 n=3 ablation as closely as possible.
#
# Matrix:
#   terrain_z_offset {-0.03, +0.03}
#   x {robust ON with splice OFF, robust OFF}
#   x 3 repetitions
#
# Results:
#   tools/perceptive_dev_v2/results/good_data_reproduce/

set -u

cd "$(dirname "$0")/../.."

RESULTS_DIR="tools/perceptive_dev_v2/results/good_data_reproduce"
failures=0

run_one() {
    local robust="$1"
    local offset="$2"
    local tag="$3"

    echo
    echo "=========================================================="
    echo "[good_data] tag=$tag robust=$robust offset=$offset"
    echo "=========================================================="

    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust "$robust" \
        --robust-d 0.05 \
        --robust-splice off \
        --robust-verbose on \
        --mpc-frequency 10 \
        --terrain-z-offset "$offset" \
        --terrain-z-offset-only-below-z 0.15 \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        --tag "$tag"; then
        echo "[good_data] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi

    echo "[good_data] DDS cooldown: 3s"
    sleep 3
}

# Preserve the original interleaved ON/OFF order for the -0.03 m cell.
for run in 1 2 3; do
    run_one on -0.03 \
        "v2_ablation_n3_ON_nosplice_offM03_d05_P10_sqp2_run${run}"
    run_one off -0.03 \
        "v2_ablation_n3_OFF_offM03_d05_P10_sqp2_run${run}"
done

# Preserve the original interleaved ON/OFF order for the +0.03 m cell.
for run in 1 2 3; do
    run_one on 0.03 \
        "v2_ablation_n3_p03_ON_nosplice_offP03_d05_P10_sqp2_run${run}"
    run_one off 0.03 \
        "v2_ablation_n3_p03_OFF_offP03_d05_P10_sqp2_run${run}"
done

echo
echo "[good_data] generating all_visualizations"
if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
    --results-dir "$RESULTS_DIR"; then
    echo "[good_data] FAILED: all_visualizations" >&2
    failures=$((failures + 1))
fi

echo
echo "[good_data] all 12 trials attempted; failures=$failures"
echo "[good_data] results: $(pwd)/$RESULTS_DIR"
echo "[good_data] gallery: $(pwd)/$RESULTS_DIR/all_visualizations/index.html"
exit "$failures"

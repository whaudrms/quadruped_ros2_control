#!/usr/bin/env bash
# Robust-ON terrain-perception-error response sweep.
#
# Matrix (default):
#   terrain_z_offset {-0.05, -0.03, +0.03, +0.05} m
#   x N_RUNS=5 repetitions
#   x robust phase ON only
#
# Results:
#   tools/perceptive_dev_v2/results/robust_terrain_error_sweep_n<N_RUNS>/
#
# Examples:
#   N_RUNS=1 bash tools/perceptive_dev_v2/m2_robust_terrain_error_sweep.sh
#   N_RUNS=5 bash tools/perceptive_dev_v2/m2_robust_terrain_error_sweep.sh
#   RESULTS_DIR=/tmp/sweep N_RUNS=1 bash tools/perceptive_dev_v2/m2_robust_terrain_error_sweep.sh

set -u

cd "$(dirname "$0")/../.."

N_RUNS="${N_RUNS:-5}"
RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/robust_terrain_error_sweep_n${N_RUNS}}"
DDS_COOLDOWN_SEC="${DDS_COOLDOWN_SEC:-3}"
DRY_RUN="${DRY_RUN:-0}"
failures=0

if ! [[ "$N_RUNS" =~ ^[1-9][0-9]*$ ]]; then
    echo "[terrain_sweep] N_RUNS must be a positive integer: $N_RUNS" >&2
    exit 2
fi

offset_token() {
    case "$1" in
        -0.05) echo "M05" ;;
        -0.03) echo "M03" ;;
         0.03) echo "P03" ;;
         0.05) echo "P05" ;;
        *) echo "invalid" ; return 1 ;;
    esac
}

trial_exists() {
    local tag="$1"
    local trial_dir
    while IFS= read -r trial_dir; do
        if [[ -f "$trial_dir/result.json" ]]; then
            return 0
        fi
    done < <(
        find "$RESULTS_DIR" -mindepth 1 -maxdepth 1 -type d -name "*_${tag}" -print \
            2>/dev/null
    )
    return 1
}

run_one() {
    local offset="$1" run="$2" token tag
    local -a run_mode_args=()
    token="$(offset_token "$offset")" || return 1
    tag="robust_terrain_sweep_n${N_RUNS}_ON_nosplice_off${token}_d05_P10_sqp2_run${run}"

    if [[ "$DRY_RUN" != "1" ]] && trial_exists "$tag"; then
        echo "[terrain_sweep] SKIP existing trial: $tag"
        return 0
    fi

    echo
    echo "=========================================================="
    echo "[terrain_sweep] tag=$tag robust=on offset=$offset"
    echo "=========================================================="

    if [[ "$DRY_RUN" == "1" ]]; then
        run_mode_args+=(--dry-run)
    fi

    if ! python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_only_reproduce \
        --terrain basic_step_short_v2 \
        --mode perceptive_dev_v2 \
        --robust on \
        --robust-p 10 \
        --robust-d 0.05 \
        --robust-v-max 0.6 \
        --robust-splice off \
        --robust-verbose on \
        --mpc-frequency 10 \
        --terrain-z-offset "$offset" \
        --terrain-z-offset-only-below-z 0.15 \
        --metrics-grace-sec 5 \
        --post-trial-hold-sec 0 \
        --results-dir "$RESULTS_DIR" \
        --force-cleanup \
        "${run_mode_args[@]}" \
        --tag "$tag"; then
        echo "[terrain_sweep] FAILED: $tag" >&2
        failures=$((failures + 1))
    fi

    if [[ "$DRY_RUN" != "1" ]]; then
        echo "[terrain_sweep] DDS cooldown: ${DDS_COOLDOWN_SEC}s"
        sleep "$DDS_COOLDOWN_SEC"
    fi
}

# Reverse the order on alternating repetitions to reduce monotonic run-order bias.
for run in $(seq 1 "$N_RUNS"); do
    if (( run % 2 == 1 )); then
        offsets=(-0.05 -0.03 0.03 0.05)
    else
        offsets=(0.05 0.03 -0.03 -0.05)
    fi
    for offset in "${offsets[@]}"; do
        run_one "$offset" "$run"
    done
done

if [[ "$DRY_RUN" != "1" ]]; then
    echo
    echo "[terrain_sweep] generating tables, plots, and all_visualizations"
    if ! python3 tools/perceptive_dev_v2/plot_all_results.py \
        --results-dir "$RESULTS_DIR"; then
        echo "[terrain_sweep] FAILED: all_visualizations" >&2
        failures=$((failures + 1))
    fi
fi

attempted=$((4 * N_RUNS))
if [[ "$RESULTS_DIR" = /* ]]; then
    results_display="$RESULTS_DIR"
else
    results_display="$(pwd)/$RESULTS_DIR"
fi
echo
echo "[terrain_sweep] all $attempted trials attempted or resumed; failures=$failures"
echo "[terrain_sweep] results: $results_display"
echo "[terrain_sweep] gallery: $results_display/all_visualizations/index.html"
exit "$failures"

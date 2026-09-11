#!/usr/bin/env bash
# Run one scenario once with Robust ON; no OFF baseline or repetitions.
# All OCP settings come from task.info except robustPhase.enabled=true.
#
# Usage (from quadruped_ros2_control):
#   bash tools/perceptive_dev_v2/exp_robust_only.sh
#   DRY_RUN=1 bash tools/perceptive_dev_v2/exp_robust_only.sh
#   TERRAIN_Z_OFFSET=0 bash tools/perceptive_dev_v2/exp_robust_only.sh
#   FORCE_CLEANUP=1 bash tools/perceptive_dev_v2/exp_robust_only.sh

set -Eeuo pipefail
cd "$(dirname "$0")/../.."

SCENARIO="${SCENARIO:-standing_trot_forward_only_reproduce}"
TERRAIN="${TERRAIN:-basic_step_up_short_v2}"
TERRAIN_Z_OFFSET="${TERRAIN_Z_OFFSET:-0.0}"
# Empty explicitly selects all non-floor surfaces; default targets the lower step.
TERRAIN_Z_OFFSET_ONLY_BELOW_Z="${TERRAIN_Z_OFFSET_ONLY_BELOW_Z-0.15}"
RESULTS_DIR="${RESULTS_DIR:-tools/perceptive_dev_v2/results/robust_only}"
TAG="${TAG:-robust_only}"
DRY_RUN="${DRY_RUN:-0}"
FORCE_CLEANUP="${FORCE_CLEANUP:-0}"

for name in DRY_RUN FORCE_CLEANUP; do
    if [[ "${!name}" != "0" && "${!name}" != "1" ]]; then
        echo "[robust_only] $name must be 0 or 1: ${!name}" >&2
        exit 2
    fi
done

run_args=()
if [[ "$DRY_RUN" == "1" ]]; then
    run_args+=(--dry-run)
fi
if [[ "$FORCE_CLEANUP" == "1" ]]; then
    run_args+=(--force-cleanup)
fi
if [[ -n "$TERRAIN_Z_OFFSET_ONLY_BELOW_Z" ]]; then
    run_args+=(--terrain-z-offset-only-below-z "$TERRAIN_Z_OFFSET_ONLY_BELOW_Z")
fi

echo "[robust_only] one trial: scenario=$SCENARIO terrain=$TERRAIN offset=$TERRAIN_Z_OFFSET"
echo "[robust_only] Robust ON; OCP parameters inherited from task.info"

exec python3 tools/perceptive_dev_v2/run_trial.py \
    --scenario "$SCENARIO" \
    --terrain "$TERRAIN" \
    --mode perceptive_dev_v2 \
    --terrain-z-offset "$TERRAIN_Z_OFFSET" \
    --foothold-plan-log on \
    --metrics-grace-sec 5 \
    --post-trial-hold-sec 0 \
    --results-dir "$RESULTS_DIR" \
    --tag "$TAG" \
    "${run_args[@]}"

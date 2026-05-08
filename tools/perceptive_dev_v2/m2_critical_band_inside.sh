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
TASK_INFO="descriptions/unitree/go2_description/config/ocs2/task.info"
BAK="${TASK_INFO}.crit.bak"

cp "$TASK_INFO" "$BAK"
trap 'echo "[crit] restoring task.info from $BAK"; cp "$BAK" "$TASK_INFO"; rm -f "$BAK"' EXIT INT TERM

set_robust() {
    sed -i -E "s/^(\s*enabled\s+)(true|false)(.*)$/\1$1\3/" "$TASK_INFO"
    grep -E "^\s*enabled\s+" "$TASK_INFO" | head -1
}

run_one() {
    local robust="$1" tag="$2"
    set_robust "$robust"
    echo
    echo "=========================================================="
    echo "[crit] tag=$tag  robust=$robust  off=-0.02  d=0.03"
    echo "=========================================================="
    python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward_short \
        --terrain basic_step_short \
        --mode perceptive_dev_v2 \
        --mpc-frequency 10 \
        --terrain-z-offset -0.02 \
        --terrain-z-offset-only-below-z 0.15 \
        --tag "$tag" 2>&1 | tail -3
}

run_one true   crit_band_robON_offM02
run_one false  crit_band_robOFF_offM02

echo
echo "[crit] both trials done."

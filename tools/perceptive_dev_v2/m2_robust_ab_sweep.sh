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
# task.info: robustPhase.enabled is toggled in-place per trial (backed up to
# .info.bak by this script and restored at the end, even on Ctrl+C).
#
# Run from quadruped_ros2_control/ root:
#   zsh -c "source /home/cora/GO2_ws/setup_quadruped.sh && \
#           bash tools/perceptive_dev_v2/m2_robust_ab_sweep.sh"

set -u  # don't set -e: we want all 6 trials to attempt even if one falls

cd "$(dirname "$0")/../.."
TASK_INFO="descriptions/unitree/go2_description/config/ocs2/task.info"
BAK="${TASK_INFO}.absweep.bak"

cp "$TASK_INFO" "$BAK"
trap 'echo "[sweep] restoring task.info from $BAK"; cp "$BAK" "$TASK_INFO"; rm -f "$BAK"' EXIT INT TERM

set_robust() {
    local val="$1"  # "true" or "false"
    sed -i -E "s/^(\s*enabled\s+)(true|false)(.*)$/\1${val}\3/" "$TASK_INFO"
    grep -E "^\s*enabled\s+" "$TASK_INFO" | head -1
}

run_one() {
    local robust="$1" off="$2" tag="$3"
    set_robust "$robust"
    echo
    echo "=========================================================="
    echo "[sweep] tag=$tag  robust=$robust  offset=$off"
    echo "=========================================================="
    python3 tools/perceptive_dev_v2/run_trial.py \
        --scenario standing_trot_forward \
        --terrain basic_step_short \
        --mode perceptive_dev_v2 \
        --mpc-frequency 10 \
        --terrain-z-offset "$off" \
        --terrain-z-offset-only-below-z 0.15 \
        --tag "$tag" 2>&1 | tail -3
}

# 6-trial matrix
run_one true   0.00  m2ab10_robON_off0
run_one true   0.05  m2ab10_robON_offP05
run_one true  -0.05  m2ab10_robON_offM05
run_one false  0.00  m2ab10_robOFF_off0
run_one false  0.05  m2ab10_robOFF_offP05
run_one false -0.05  m2ab10_robOFF_offM05

echo
echo "[sweep] all 6 trials done."

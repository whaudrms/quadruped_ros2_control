## Rough Ground Progress

Date: 2026-03-27

### Scope
- Active controller branch:
  - `ocs2_quadruped_controller_test1`
  - `go2_description_test1`
- Original packages are frozen.
- Current target terrain family:
  - `rough_ground`
  - `rough_ground_inside`
  - `rough_ground_all_on`
  - `rough_ground_softedge`

### Current Best Controller Baseline
- Name:
  - `warmup1 + xprog1 + rear_td4mm_after_warmup + low_speed_uphill_hold_bias + adaptive_upstep_swing_boost`

### Meaning Of Current Controller Changes
- `warmup1`
  - Initial 1.5s after stand-up:
  - terrain pitch correction ramps in gradually
  - base-z terrain following ramps in gradually
- `xprog1`
  - nominal foothold x uses more desired forward progression
- `rear_td4mm_after_warmup`
  - rear-leg touchdown heights get `+0.004 m` after warmup
- `low_speed_uphill_hold_bias`
  - at near-zero planar speed, body pose gets a small uphill bias to reduce backward sliding on terrain
- `adaptive_upstep_swing_boost`
  - only on upward swing steps:
  - extra apex height is added locally
  - global swing height is unchanged

### Rejected Controller Changes
- global `swingHeight 0.11`
  - one good run, poor reproducibility
- global `swingTimeScale 0.22`
  - broke stand-up reproducibility
- `phaseTransitionStanceTime 0.17`
  - worsened stability
- `touchDownVelocity -0.025`
  - worse drift / progression
- front support stronger x-anchor
  - reduced progression too much
- rear slope penalty in selector
  - broke stand-up

### Terrain Roles
- `eval_rough_ground.xml`
  - original harder map
  - use for final verification
- `eval_rough_ground_inside.xml`
  - terrain starts under robot
  - use to isolate on-terrain support / hold behavior
- `eval_rough_ground_all_on.xml`
  - all-feet-on-terrain start
  - use to isolate full on-terrain support / no-edge behavior
- `eval_rough_ground_softedge.xml`
  - softer bridge map between inside and original outside
  - use for middle-stage tuning

### Current Softedge Status
- `softedge v2` works as a bridge map
- latest verified auto run:
  - result:
    - `results/20260327_020723_rough_ground_softedge_perceptive_softedge_currentcheck1/result.json`
  - metrics:
    - `success true`
    - `distance_xy ~ 0.968`
    - `end_x ~ 0.882`
    - `roll_rms ~ 0.418 deg`
    - `pitch_rms ~ 1.181 deg`

### User Visual Findings To Preserve
- On `rough_ground_inside`, robot can get on terrain.
- Main remaining issue was static/on-terrain support, not only edge crossing.
- On `rough_ground_all_on`, mild sliding is acceptable.
- On `rough_ground_softedge`, user observed death around roughly the later part of the map, beyond the shorter automatic smoke range.

### Next Step
- Run a longer `softedge` automatic scenario to reach the later terrain patch and reproduce the failure point.

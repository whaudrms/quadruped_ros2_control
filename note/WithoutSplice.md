# Robust Phase Without Splice — A/B Comparison Report

This report documents the state of the in-OCP robust phase **before** any
event-triggered schedule splice or WBC contact-flag override is added (i.e. with
Track ② step (a) detection-only and Track ② step (b) deliberately deferred), and
records a first low-MPC-rate A/B sweep designed to expose whether the robust
phase's contact-timing-uncertainty handling actually helps the closed-loop robot
on a perception-noisy descent.

The deferral matters: per `chat4.md`, mixing robust-phase OCP changes with a
WBC reactive layer makes it impossible to attribute observed stability gains
to the OCP itself. The "without splice" configuration isolates the robust
phase's effect.

## Scope: what is and isn't built

In place (this branch, commits up to `6f4f665`):

| layer | status |
|---|---|
| Robust phase guard `g(x_a)=+d, g(x_b)=-d`, `ġ ≤ 0`, `Σ w_v ġ²` inside MPC OCP | **DONE** (M1''/M2 commits `0fe5079, 508dbed, d46a921, 9509863, f7efd82, 785f68d, f0d57c9`) |
| Per-leg projection `(n, p_plane)` from `ConvexRegionSelector` | **DONE** (M2, `9509863`) |
| `NormalVelocityConstraintCppAd::isActive` deactivation in robust window | **DONE** (M1'' part of `0fe5079`) |
| `FootCollisionConstraint::isActive` deactivation in robust window | **DONE** (M1'' part of `0fe5079`) |
| Track ② step (a) — `[robust_event] type=early\|late` detection logging in `CtrlComponent::updateState` | **DONE** (`f86da92, be27135`) |
| Stage 8/9 cleanup (Python refiner, one-shot OCP, `[robust_refine] override` log) | **DONE** (`2c4b986`) |

Deliberately not in place:

| layer | status |
|---|---|
| Track ② step (b) — schedule splice via `gait_schedule_ptr_->setModeSchedule(...)` on detected sustained early/late contact | **NOT IMPLEMENTED** |
| WBC contact-flag override (passing measured contact instead of scheduled to `WbcBase::update(... mode ...)`) | **NOT IMPLEMENTED** |
| MPC `getContactFlags(t)` rewriting to consult `observation_.mode` | **NOT IMPLEMENTED** |

This is the configuration referred to here as **"without splice"**.

## Configuration of record

Codebase parameters at the time of the sweep:

| layer | value |
|---|---|
| Robot | Go2 (`quadruped_ros2_control` perceptive controller) |
| Scene | `basic_step_short.xml` (floor + box1 z=0.20 + box2 z=0.10, edge at x=0.30) |
| Scenario | `standing_trot_forward` (16 s; stand → enter OCS2 → forward at 0.3 m/s) |
| Gait | `standing_trot` (swing 0.25 s × 2 + all-stance 0.05 s × 2; period 0.6 s) |
| OCP discretization (`sqp.dt`) | **0.02 s** (100 nodes / 2 s horizon) |
| MPC re-solve rate (`mpcDesiredFrequency`) | **10 Hz** for the sweep (M2 default 50 Hz) |
| `update_rate` (controller_manager / WBC tick) | 1000 Hz |
| Robust window length (`P`) | 5 nodes → `T_robust = P · sqp.dt = 0.10 s` (40 % of swing) |
| Uncertainty half-width (`d`) | 0.03 m |
| Boundary soft-penalty weight (`w_boundary`) | 1000 |
| Approach-velocity cost weight (`w_v`) | 1.0 |
| Foot-frame offset above contact (`foot_frame_offset`) | 0.06 m (Go2 ankle frame) |
| `terrain_source` | `convex_region` (per-leg `(n, p_plane)` from `ConvexRegionSelector`) |
| Joint PD on hardware interface | `Kp = 0`, `Kd = 6` (feedforward + velocity damping only) |

Two configuration knobs were varied across the sweep:
`task.info: robustPhase.enabled` (true/false) and the launch parameter
`terrain_z_offset` (m), restricted to surfaces with true top z below 0.15 m
(i.e., box2 only).

## Why these choices

`chat4.md` recommends "same WBC, same MPC rate, same terrain" → "original
perceptive MPC vs robust-phase perceptive MPC" comparison, with MPC rate
**lowered** (10/20/50 Hz sweep) so contact-timing mismatches stay open for
several control ticks before MPC can replan. We chose **10 Hz** as the first
test point.

`sqp.dt` is held at **0.02 s** rather than scaled with the MPC period: these
are conceptually independent. Lowering `sqp.dt` to 0.10 s along with
`mpcDesiredFrequency=10` would make `T_robust = P · sqp.dt = 0.50 s`, which
exceeds the 0.25 s swing duration — the robust window would intrude on the
next swing. Keeping `sqp.dt = 0.02 s` preserves `T_robust = 0.10 s` (40 % of
swing) and isolates the MPC re-solve rate as the only varying time-scale
parameter.

`terrain_z_offset` is restricted to box2 to model the realistic case: the
robot starts on box1 (well-perceived because directly under the body at
trial start) and descends to box2 (perception target with intrinsic
uncertainty). Applying the offset to box1 too would make the start
configuration itself faulty, contaminating the comparison.

The `±0.05 m` magnitude was chosen because it is comparable to typical
`grid_map_sdf` height-map noise on cluttered terrain. Note that with
`d = 0.03 m`, this magnitude **exceeds the robust-phase guard band**
(see "Findings" below).

## Sweep matrix

Six trials, single seed each:

| trial tag | `robustPhase.enabled` | `terrain_z_offset` (box2 only) |
|---|---|---|
| `m2ab10_robON_off0`     | true  | 0.00 |
| `m2ab10_robON_offP05`   | true  | +0.05 |
| `m2ab10_robON_offM05`   | true  | −0.05 |
| `m2ab10_robOFF_off0`    | false | 0.00 |
| `m2ab10_robOFF_offP05`  | false | +0.05 |
| `m2ab10_robOFF_offM05`  | false | −0.05 |

Physical interpretation of the offsets (MuJoCo physics is unchanged in all
cases — the actual box2 stays at z=0.10):

| offset | controller's perceived box2 top | meaning for the descent |
|---|---|---|
| **+0.05** | z = 0.15 (5 cm too high) | OCP plans to land on z=0.15 but the foot reaches z=0.10 first → **early contact** |
| **0.00** | z = 0.10 (correct) | Nominal case |
| **−0.05** | z = 0.05 (5 cm too low) | OCP plans to land on z=0.05 but the actual surface is at z=0.10 → **foot driven 5 cm into the actual terrain** |

The −0.05 case is the harsh case (perception under-shoots the descent target).

Driver: [`tools/perceptive_dev_v2/m2_robust_ab_sweep.sh`](../tools/perceptive_dev_v2/m2_robust_ab_sweep.sh).
The script `sed`-toggles `task.info: robustPhase.enabled` per trial and
restores from `task.info.absweep.bak` on exit (also on Ctrl-C).
Each trial uses the new `--mpc-frequency` flag in `run_trial.py`, which
in-place edits `mpcDesiredFrequency` in `task.info` and restores it via
`finally` (also on fall / SIGKILL of `auto_input_metrics.py`).

## Results

```text
trial               success  fall          dist[m]  min_z  end_z   pitch_rms[°]  roll_rms[°]  base_z_std
robON  off=0.00     True     -             1.136    0.317  0.471         5.56          1.47    0.0446
robON  off=+0.05    True     -             1.661    0.317  0.482         5.32          1.95    0.0436
robON  off=-0.05    False    roll_limit    0.771    0.317  0.439        10.60         13.13    0.0557
robOFF off=0.00     True     -             1.501    0.317  0.456         5.38          1.56    0.0486
robOFF off=+0.05    True     -             1.695    0.317  0.473         5.34          2.10    0.0448
robOFF off=-0.05    False    roll_limit    1.119    0.317  0.331        11.60          7.12    0.0621
```

Trial result directories:
`tools/perceptive_dev_v2/results/20260508_22*_basic_step_short_perceptive_dev_v2_m2ab10_*`.

## Findings

### F1 (preliminary, single seed)

The harsh case `−0.05` makes BOTH configurations fall (`roll_limit`).
That on its own is not surprising. What is surprising:

| | distance covered | roll_rms |
|---|---|---|
| robust ON,  off=−0.05 | **0.77 m** | **13.13°** |
| robust OFF, off=−0.05 | 1.12 m | 7.12° |

Robust ON falls **earlier** and rolls **worse** in the harsh case. The
mechanism is straightforward once written out:

```
target z_foot at t_b  =  perceived_terrain_z + foot_frame_offset − d
                       =  (0.10 − 0.05) + 0.06 − 0.03
                       =  0.08 m
contact-point target  =  z_foot − foot_frame_offset = 0.02 m
actual terrain z      =  0.10 m
```

The robust-phase boundary equality is therefore commanding the contact
point to **0.08 m below the actual terrain**. The soft penalty
(`w_boundary = 1000`) drives the foot hard against the box surface.
Asymmetric reaction force across the diagonal trot pair → roll instability
→ fall. Robust OFF, by contrast, only commands the ankle frame to reach
z = 0.05 (perceived terrain top), and lacks the additional `−d` push, so
the violation amplitude is smaller and the body rolls less.

The reason this is interesting is that this is the case for which the
robust phase was supposedly designed. Per `robust_phase.tex` and
`chat1.md` the design intent is "land safely anywhere in the
`[perceived − d, perceived + d]` band". With `d = 0.03` and `|offset| =
0.05`, the actual terrain at z=0.10 lies **outside the band**
(`[0.02, 0.08]`). The robust phase's guarantee is voided. The phase then
behaves like a strong soft constraint that commits even harder to the
wrong surface.

### F2 (preliminary, single seed)

The mild case `+0.05` shows **no measurable benefit** from the robust phase:

| | distance | pitch_rms | roll_rms |
|---|---|---|---|
| robust ON,  off=+0.05 | 1.661 | 5.32 | 1.95 |
| robust OFF, off=+0.05 | 1.695 | 5.34 | 2.10 |

This is the case for which the robust phase's intent — "expect early
contact and don't trip on it" — should help. The numbers are nearly
identical. Likely reasons (not yet verified, listed in priority of
plausibility):

1. **MPC at 10 Hz is still fast enough to absorb early contact via
   replanning.** The next OCP solve sees the foot already on the ground
   (initial state has it there) and re-plans starting from a stance-like
   configuration; over a 100 ms cycle this implicit recovery may rival
   the explicit robust-phase reference. `chat4.md` predicted this and
   recommended sweeping down to 2–5 Hz.
2. **The hardware-interface `Kp = 0` cushions the mismatch.** With
   position-PID disabled, the joint command does not aggressively fight
   the ground; the foot just stays planted and ground reaction propagates
   through the leg without amplifying torque error. Both robust ON and
   robust OFF benefit equally.
3. **The +0.05 case is gentler than expected.** The foot lands on the
   surface, not below it; the resulting state has high force but no
   penetration. There may simply be no instability to compare against.

### F3 (consistent with M2 A/B at 50 Hz)

The `0.00` baseline (no perception noise) shows robust ON paying ~24 % in
distance covered (1.14 m vs 1.50 m), with body-stability metrics roughly
equal. This matches the M2 A/B finding at 50 Hz (1.17 vs 1.57; commit
`8d6b027` Appendix C.2): the robust-phase soft penalty pulls the foot
slightly below the swing planner's nominal trajectory and slows forward
progress. **The cost is consistent and present whether or not perception
noise exists.**

## What this report does not yet answer

- **Whether the F2 null result is due to MPC rate or to the test scenario
  being too gentle.** Resolution: a lower-rate sweep (2 / 5 / 10 Hz at
  off = 0) would clarify.
- **Whether the robust phase actually helps when its design assumption
  `|perception error| ≤ d` holds.** Resolution: a `d` sweep (0.03 / 0.05
  / 0.07) at fixed offset = ±0.05 would close this.
- **Whether the F1 result is stable across runs.** Resolution: 5–10 seeds
  per cell.
- **Whether the foot impact velocity (the metric robust phase most
  directly targets) actually decreases under robust ON.** Resolution: a
  per-tick foot-velocity computation from `tick.csv` joint velocities
  via Pinocchio FK at the moment of first measured contact.

## Implication for Track ②

Track ② step (b) — schedule splice + WBC override — was deferred so this
report could be produced cleanly. The sweep result motivates further
deferring step (b) at least until **F1 is resolved by a `d` sweep**. The
specific concern: if step (b) is added on top of a configuration where
robust phase's guard band fails to contain the actual terrain (as in the
−0.05 case here), the schedule splice will fire on every cycle the foot
hits the surface (sustained-contact threshold trivially satisfied), and
the splice will simply latch the leg to stance after the body has already
been rolled by the impact. The reactive layer would mask, not fix, the
underlying parameter mismatch.

The cleaner sequence is:

1. Sweep `d` at fixed offset. Confirm the regime where robust phase
   actually delivers its design promise (lower fall rate, lower foot
   impact velocity than original).
2. (chat4 recommendation) Sweep MPC rate at off = 0 to confirm the
   robust phase's effect is rate-dependent in the predicted direction.
3. Add multi-seed bars for the cells of interest.
4. Then decide whether step (b) (schedule splice, WBC override held
   separately for the comparison-purity reason from chat4) is worth
   adding, and on what threshold.

## Files of record

| path | role |
|---|---|
| [`tools/perceptive_dev_v2/m2_robust_ab_sweep.sh`](../tools/perceptive_dev_v2/m2_robust_ab_sweep.sh) | 6-trial driver script |
| [`tools/perceptive_dev_v2/run_trial.py`](../tools/perceptive_dev_v2/run_trial.py) | adds `--mpc-frequency` and `--terrain-z-offset-only-below-z` flags |
| [`controllers/.../perceptive/publisher/StaticPlanarTerrainPublisher.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/publisher/StaticPlanarTerrainPublisher.cpp) | `terrain_z_offset_only_below_z` filter |
| [`controllers/.../launch/mujoco.launch.py`](../controllers/ocs2_quadruped_controller/launch/mujoco.launch.py) | exposes the new launch arg |
| [`descriptions/unitree/go2_description/config/ocs2/task.info`](../descriptions/unitree/go2_description/config/ocs2/task.info) | `robustPhase` block (toggled per trial by the sweep script) |
| [`note/m1pp_robust_phase_in_ocs2.md`](m1pp_robust_phase_in_ocs2.md) | M1'' implementation + review |
| [`note/m2_robust_phase_terrain_aware.md`](m2_robust_phase_terrain_aware.md) | M2 implementation + Track ② step (a) Appendix D |
| this file | sweep methodology + results + implication for Track ② step (b) |

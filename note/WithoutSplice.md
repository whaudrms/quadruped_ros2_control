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
| Per-leg `p_plane` from `ConvexRegionSelector` (terrain plane normal `n` is still hard-wired to `e_z`) | **DONE for `p_plane`; `n` deferred to M2.x** (M2, `9509863`) |
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
cases — the actual box2 stays at z=0.10). Robust ON foot-frame target at
`t_b` is `target_zf = perceived_terrain_z + foot_frame_offset − d` where
`foot_frame_offset = 0.06 m` and `d = 0.03 m`; the natural-touchdown foot
frame z is `actual_terrain_z + foot_frame_offset = 0.16 m`:

| offset | perceived box2 top | robust ON `target_zf(t_b)` | vs natural touchdown 0.16 | regime |
|---|---|---|---|---|
| **+0.05** | z = 0.15 (5 cm too high) | **0.18 m** | 2 cm **above** | foot trajectory ends ABOVE actual ground → no contact at scheduled `t_b` → **late / missed contact** |
| **0.00** | z = 0.10 (correct) | **0.13 m** | 3 cm **below** | foot reaches actual ground a few ms before `t_b` → mild design-driven early contact (M2 baseline behavior) |
| **−0.05** | z = 0.05 (5 cm too low) | **0.08 m** | 8 cm **below** | foot trajectory aims through the actual ground; meets it well before `t_b` → **hard early contact** |

So **+0.05 is the late-contact case** (the perceived terrain sits above the
real one, the foot aims too high, and the swing runs out before contact),
and **−0.05 is the hard early-contact case** (perceived terrain sits
below the real one, the foot aims through it, and the actual surface
arrests the foot well before the scheduled touchdown).

This is consistent with the per-leg `[robust_event]` log counts collected
during the trials (see "Findings" §F0 below for the table): `+0.05` is
dominated by `late` events, `−0.05` is dominated by `late` events too but
those are mostly fall-induced (the leg is airborne after the body rolls
over), while the small `early` count at `−0.05` (14) catches the actual
hard-contact moments before the fall.

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

### F0 — `[robust_event]` log counts (Track ② step (a) data)

Per-leg event counts from `controller.log` over each 16 s trial:

| trial | early | late | notes |
|---|---|---|---|
| robON  off=0.00 | **263** | 5 | M2 baseline pattern: design-driven mild early on every swing (foot target 3 cm below natural touchdown) |
| robON  off=+0.05 | 7 | **264** | foot frame target 0.18 m (above actual 0.16 m) → almost every scheduled `t_b` arrives with the foot still in air → late dominant |
| robON  off=−0.05 | 14 | 257 | 14 hard-contact events captured BEFORE the body falls; the 257 late events are mostly fall-induced (legs airborne after roll) |
| robOFF off=0.00 | 0 | 264 | early count is 0 by construction (no robust window → `isInRobustWindow` never true). 264 late = baseline noise level: at 10 Hz MPC and `positionErrorGain=0`, the foot arrives at the scheduled `t_b` rising-edge tick with measured force still below the 5 N estimator threshold |
| robOFF off=+0.05 | 0 | 242 | same baseline rate of late events; foot eventually lands later but not at the rising-edge tick |
| robOFF off=−0.05 | 0 | 256 | same pattern; fall scenario doesn't shift the count much because robust OFF was already missing schedule-rising-edge contacts |

These counts directly support the offset-sign reading above: `+0.05` is
the late case (264 late at robust ON), `−0.05` is the early case (only
14 caught events because the body falls before many more swings can
happen). The late event count for robust OFF is roughly constant
(~240–264) regardless of offset, which is itself an interesting baseline
fact — at 10 Hz with `positionErrorGain=0` the original perceptive MPC
already misses the rising-edge of scheduled stance ~66 % of the time
even on flat ground.

### F1 — `−0.05` (hard early contact): both fall, robust ON falls earlier

| | distance covered | roll_rms |
|---|---|---|
| robust ON,  off=−0.05 | **0.77 m** | **13.13°** |
| robust OFF, off=−0.05 | 1.12 m | 7.12° |

Robust ON falls **earlier** and rolls **worse**. Mechanism:

```
robust ON  target z_foot at t_b  =  perceived_terrain_z + foot_frame_offset − d
                                 =  0.05 + 0.06 − 0.03 = 0.08 m
contact-point equivalent          =  0.02 m   (8 cm below actual ground at 0.10 m)
actual terrain z                  =  0.10 m
```

The robust-phase boundary equality drives a SOFT POSITION constraint on
the foot frame at z=0.08 — i.e., commands the contact point 8 cm below
the actual surface. The soft penalty (`w_boundary = 1000`) is strong
enough to apply meaningful joint torques toward this infeasible target.
Combined with the diagonal trot pair (one swinging, one stancing), the
asymmetric reaction force at the impossibly-low target rolls the body.

Robust OFF does **not** equivalently command the foot to perceived = 0.05.
The original perceptive MPC plans the foot trajectory using
`NormalVelocityConstraint` with `task.info: positionErrorGain = 0.0`, so
the constraint is **velocity-only** (`config.b -= positionErrorGain *
zPositionConstraint` is skipped, and `Ax` stays zero — see
[`LeggedRobotPreComputation.cpp:75-83`](../controllers/ocs2_quadruped_controller/src/interface/LeggedRobotPreComputation.cpp#L75-L83)).
There is no terminal foot-z position equality; the swing planner shapes
the trajectory through the cost, not a hard pull at `t_b`. So robust OFF
sees a milder version of the same wrong perception and reacts to it more
softly, which is why it survives further (1.12 m before fall vs 0.77 m).

The original design intent for the robust phase was "land safely
anywhere in the `[perceived − d, perceived + d]` band". With `d = 0.03`
and `|offset| = 0.05`, the actual terrain at z=0.10 lies **outside**
that band (`[0.02, 0.08]`). The phase's correctness guarantee is voided
in this regime, and the soft boundary becomes a strong nudge toward a
wrong static position. **The −0.05 sweep is therefore not a fair test
of the robust phase's design intent** — it is a stress test of what
robust phase does when its assumption (`|error| ≤ d`) is violated.

### F2 — `+0.05` (late / missed contact): both succeed, no robust-phase benefit

| | distance | pitch_rms | roll_rms |
|---|---|---|---|
| robust ON,  off=+0.05 | 1.661 | 5.32 | 1.95 |
| robust OFF, off=+0.05 | 1.695 | 5.34 | 2.10 |

Per the corrected offset-sign reading, **+0.05 is the LATE-contact
regime, not early**. The robust phase as designed is intended to
handle EARLY contact (foot may meet terrain anywhere in the
`[perceived − d, perceived + d]` band, possibly before scheduled `t_b`),
not late contact (perceived terrain sits above reality so the foot
runs out of swing trajectory before touching). So we should not have
expected a robust-phase benefit on `+0.05` to begin with — the design
doesn't promise one. The near-identical numbers are consistent with
that.

What is interesting is that **both modes survive late contact at all**.
Two plausible mechanisms (not yet verified by deeper instrumentation):

1. **Joint Kd = 6 + Kp = 0 absorbs the timing mismatch.** When the foot
   eventually finds ground (a few tens of ms after scheduled `t_b`), the
   ground reaction is absorbed by velocity damping rather than amplified
   by position PID.
2. **MPC at 10 Hz still re-plans within ~100 ms of the missed touchdown.**
   The next OCP gets a measured state in which the leg is airborne past
   its scheduled stance time; the OCP cannot consult `observation_.mode`
   in this codebase (see "Note on F2 mechanism" below) but the new
   initial state itself encodes "foot is at z=0.16, not z=0.21" and the
   OCP re-shapes the next swing/stance accordingly.

**Note on F2 mechanism (correction to an earlier draft).** An earlier
version of this report said "the next OCP solve sees the foot already
on the ground and re-plans starting from a stance-like configuration".
This overstated what the code does. `MPC_MRT_Interface::advanceMpc()`
calls `mpc_.run(currentObservation.time, currentObservation.state)` —
state is passed but `currentObservation.mode` is NOT
([`MPC_MRT_Interface.cpp:73`](https://github.com/leggedrobotics/ocs2/blob/main/ocs2_mpc/src/MPC_MRT_Interface.cpp#L73)).
The contact schedule used for constraint activation comes from
`SwitchedModelReferenceManager::getContactFlags(t)` which reads the
gait template only. So the new OCP only uses the measured state
(joint positions, base pose) — it does NOT explicitly re-tag the leg
as stance. Whatever recovery happens is purely through the kinematic
state being closer-to-stance at the next solve, not through any
explicit mode update.

### F3 — `0.00` baseline: robust ON costs 24 % distance (consistent with M2 A/B at 50 Hz)

The `0.00` baseline (no perception noise) shows robust ON covering 1.14 m
vs robust OFF's 1.50 m, with body-stability metrics roughly equal. This
matches the M2 A/B finding at 50 Hz (1.17 vs 1.57; commit `8d6b027`
Appendix C.2): the robust-phase soft penalty pulls the foot slightly
below the swing planner's nominal trajectory (target 3 cm below natural
touchdown — this is also why the F0 baseline shows 263 design-driven
early events on every swing) and slows forward progress. **The cost is
consistent and present whether or not perception noise exists.**

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

Track ② step (b) — **schedule splice only**, with WBC contact-flag
override deliberately deferred — was held so this report could be
produced cleanly. (Per `chat4.md`'s recommendation, mixing in a WBC
reactive layer would contaminate the original-vs-robust OCP comparison;
WBC override is a separate decision after the OCP-level effect is
characterized.)

The sweep result motivates further deferring step (b) at least until
**F1 is resolved by a `d` sweep**. The specific concern: if schedule
splice is added on top of a configuration where robust phase's guard
band fails to contain the actual terrain (as in the −0.05 case here),
the splice will fire on the first measured contact every swing
(sustained-contact threshold trivially satisfied), and will latch the
leg to stance after the body has already been rolled by the impact.
The reactive layer would mask, not fix, the underlying parameter
mismatch.

The cleaner sequence is:

1. Sweep `d` at fixed `−offset` (the early-contact regime). Confirm
   the regime where robust phase actually delivers its design promise
   (lower fall rate, lower foot impact velocity than original).
   Reviewer-recommended matrix: `offset ∈ {−0.02, −0.03, −0.05}` ×
   `d ∈ {0.03, 0.05, 0.07}`.
2. Treat `+offset` separately as a late-contact stress test, not a
   robust-phase benefit test.
3. (chat4 recommendation) Sweep MPC rate at `offset = 0` to confirm
   the robust phase's `t_b`-tracking effect, if any, is rate-dependent
   in the predicted direction.
4. Add multi-seed bars for the cells of interest.
5. Then decide whether step (b) (schedule splice only, no WBC
   override) is worth adding, and on what `N` consecutive-tick
   threshold.

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

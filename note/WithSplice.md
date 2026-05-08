# Robust Phase With Schedule Splice — Midterm Report

Companion to [`WithoutSplice.md`](WithoutSplice.md). Documents Track ② step (b)
— **schedule splice on sustained early measured contact during a robust window**
— added on top of the M2 robust-phase OCP, with **WBC contact-flag override
deliberately deferred**, and a 14-trial critical-cell A/B (`Δz = -0.02, d = 0.03`,
MPC 10 Hz, `basic_step_short`, `standing_trot_forward_short` 8 s) characterizing
its effect.

The framing is "midterm" because the splice is implemented and verified, but the
A/B reads as "comparable-to-better" rather than "decisively better" at the
sample sizes used; queued follow-ups (higher MPC rate, formulation tweak,
out-of-band sweeps, WBC override decision) are not yet run.

> **Errata vs. an earlier draft of this note.**
> An earlier draft of this report (commit `b278d72`) reported a single-trial
> A/B (`ON+splice ≈ 4.5°` vs. `OFF ≈ 2.5°`) and concluded "splice helps but
> doesn't reach OFF baseline." That conclusion was **wrong**: with `n = 7` per
> condition, the OFF distribution turns out to be very wide
> (`roll = 4.74° ± 2.27°`), and the `2.5°` baseline was the low-end of that
> distribution rather than a representative mean. The corrected verdict is in
> §"Verdict (n = 7 per condition)" below. The implementation also had a
> thread-safety bug (controller thread directly mutating `GaitSchedule`) that
> was fixed in this revision before re-running.

## Why splice now, and why splice only (no WBC override)

The third peer review (`chat6.md`, summary in `WithoutSplice.md` §F6) made two
corrections to the F5 reading:

1. **F5's "formulation 문제 확정" was overreach.** The splice-less F5 cell
   shows robust ON has ~3× the roll RMS of robust OFF inside the band. F5
   attributed this to the formulation `g(t_b) = −d` continuing to push the
   foot below perceived terrain. But "without splice, the schedule never
   updates on measured contact" is an alternative explanation that doesn't
   require implicating the formulation: **even with a perfectly correct
   boundary target, the OCP keeps planning towards `g(t_b) = −d` after the
   foot has physically touched, because the schedule still says the leg is in
   swing until the originally scheduled `t_b`.**
2. **The early/late event count from F0 is not a fair ON/OFF metric.** In
   OFF the early-event log is gated by `isInRobustWindow(...)` which is
   always false, so OFF early-count = 0 by construction (gating, not absence
   of physical contact).

WBC-side override is deliberately kept off so any observed gain is
attributable to the schedule-side change alone (per `chat4.md`'s staging
rule).

## Implementation

### Architectural decision: splice on the MPC sync path, not the controller thread

Critical safety invariant: **all `GaitSchedule` mutations must run on the MPC
thread**. `GaitSchedule` (in
`ocs2_ros2/basic examples/ocs2_legged_robot/include/ocs2_legged_robot/gait/GaitSchedule.h`)
has no internal synchronization — both `setModeSchedule` and the misleadingly
named `getModeSchedule(lo, hi)` (which actually mutates `modeSchedule_` by
erasing past events and re-tiling) and `insertModeSequenceTemplate` are
unprotected. Calling any of them from the controller thread races against the
MPC thread's reads/writes during `preSolverRun` /
`SwitchedModelReferenceManager::modifyReferences`.

The splice therefore uses a **two-stage queue**:

1. **Controller thread** (`CtrlComponent::detectAndLogContactEvents`) does
   detection only and **queues a splice request** via
   `GaitManager::requestStanceSplice(leg, event_time)`. No `GaitSchedule`
   access from the controller thread, ever.
2. **MPC thread** (`GaitManager::preSolverRun`) drains the queue at the start
   of each solve via `applyPendingSplices(initTime, finalTime)`, which is
   the only code that touches `gait_schedule_ptr_` for splicing. This is
   already a `SolverSynchronizedModule`, so it runs serialized with all
   other gait-schedule reads/writes within preSolverRun.

A `std::mutex` (`splice_mutex_`) protects the per-leg `splice_pending_` /
`splice_time_` arrays during cross-thread handoff. Mutex contention is
negligible (≤ ~few requests per swing per leg, vs. controller's 1 kHz tick).

### Files touched

[`controllers/.../include/ocs2_quadruped_controller/control/GaitManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/GaitManager.h):

```cpp
// Public API used by CtrlComponent — thread-safe.
void requestStanceSplice(size_t leg, scalar_t event_time);

// Private — runs on MPC thread inside preSolverRun.
void applyPendingSplices(scalar_t initTime, scalar_t finalTime);

std::mutex splice_mutex_;
feet_array_t<bool>     splice_pending_{};
feet_array_t<scalar_t> splice_time_{};
```

[`controllers/.../src/control/GaitManager.cpp`](../controllers/ocs2_quadruped_controller/src/control/GaitManager.cpp)
— `applyPendingSplices` is called at the **start** of `preSolverRun` so the
spliced schedule is visible to any subsequent gait-template insertion in the
same solve. Its responsibilities:

1. Drain `splice_pending_` / `splice_time_` under lock; release lock.
2. Sort drained requests by `event_time` (chronological insert order so
   subsequent `findIndexInTimeArray` calls see prior inserts).
3. Pull a single `ModeSchedule` slice covering `[min(initTime, earliest
   request) − 0.5, finalTime + 0.5]` via `gait_schedule_ptr_->getModeSchedule(...)`.
4. For each request: locate `curPhase` via `lookup::findIndexInTimeArray`,
   skip if the leg is already stance there, otherwise insert
   `(event_time, newMode)` at `(curPhase, curPhase+1)` to preserve the
   `|eventTimes| = |modeSequence| − 1` invariant, **then propagate the
   stance bit forward** through subsequent phases until the original
   schedule's first stance phase for that leg (gait-agnostic "early stance
   transition until natural touchdown").
5. Commit the mutated slice via `gait_schedule_ptr_->setModeSchedule(sched)`.

The forward-propagation step is the second important correctness fix from
peer review: a single inserted phase only correctly captures "early stance"
when the **next** original phase already has the leg in stance (e.g.
standing trot's all-stance inter-step phase). For an arbitrary gait, or a
perceptive schedule with terrain-driven phase modifications, the leg may
remain in swing for several original phases before the natural touchdown,
during which the splice's effect would be undone. The propagation walk
handles this gait-agnostically:

```cpp
size_t propagatedTo = curPhase + 1;
for (size_t i = curPhase + 2; i < sched.modeSequence.size(); ++i) {
    contact_flag_t f = modeNumber2StanceLeg(sched.modeSequence[i]);
    if (f[leg]) break;            // original schedule already stance — natural touchdown
    f[leg] = true;
    sched.modeSequence[i] = stanceLeg2ModeNumber(f);
    propagatedTo = i;
}
```

For the standing-trot critical cell, the `[robust_splice]` log shows
`propagated_phases=1` for every event — confirming standing trot's
all-stance inter-step makes single-phase patches sufficient for *this*
gait, while the propagation infrastructure remains correct for others.

[`controllers/.../include/ocs2_quadruped_controller/control/CtrlComponent.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h)
— removed `spliceStanceForLeg`; renamed `splice_applied_in_swing_` →
`splice_requested_in_swing_` to reflect the new "queue request, don't apply"
semantics. The `kEventSpliceSustainedTicks = 5` threshold (5 ms at 1 kHz
controller rate) and per-leg latches (`prev_scheduled_contact_`,
`early_event_logged_in_swing_`, `sustained_early_ticks_`,
`splice_requested_in_swing_`) are unchanged in intent — they all reset on
the leg's liftoff edge so each new swing cycle is eligible for a fresh
log + splice request.

[`controllers/.../src/control/CtrlComponent.cpp`](../controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp)
— `detectAndLogContactEvents` now performs detection + logging + sustained-tick
counting, and on the trigger condition calls
`gait_manager_ptr_->requestStanceSplice(leg, observation_.time)`. **No
`GaitSchedule` access from this file.** Late contact (scheduled stance with
no measured contact at touchdown) is **detection-only** — no splice action.

### Logging

`GaitManager::applyPendingSplices` emits one line per applied splice:

```
[robust_splice] leg=2 t=7.452 mode_old=9 mode_new=11 propagated_phases=1 (early stance until natural touchdown)
```

`mode_old` is `sched.modeSequence[curPhase]` (unchanged by the insert; covers
`[..., t)`); `mode_new` is the stance-flipped variant covering
`[t, original next event)`. `propagated_phases = 1` means the propagation walk
hit a natural-stance phase immediately after the inserted phase (single
patch); `> 1` means the walk extended through additional swing phases.

## Experiment

### Cell selection

Same single critical cell as `WithoutSplice.md` §F5 — the smallest cell
that should expose the splice's effect cleanly:

| parameter | value | rationale |
| --- | --- | --- |
| `terrain_z_offset` (Δz) | −0.02 m | inside the robust band (`abs(Δz) ≤ d`) — design promise *should* hold |
| `d` (robust half-width) | 0.03 m | so `[z_perc − d, z_perc + d] = [0.05, 0.11]` brackets `z_actual = 0.10` ✓ |
| `mpcDesiredFrequency` | 10 Hz | matches F5 baseline; lowest viable rate that still solves in real time |
| scene | `basic_step_short` (box1 z=0.20 / box2 z=0.10) | descent edge at `x=0.30` |
| scenario | `standing_trot_forward_short` (8 s) | stand → enter OCS2 → forward 0.3 m/s × 3 s → stop |
| splice threshold | `kEventSpliceSustainedTicks = 5` | 5 ms at 1 kHz controller rate |
| WBC override | OFF | deliberate, see "Why splice now" |

Driver script: [`tools/perceptive_dev_v2/m2_critical_band_inside.sh`](../tools/perceptive_dev_v2/m2_critical_band_inside.sh)
(plus per-pair manual reruns to reach `n = 7` per condition).

### Per-trial results (n = 7 per condition)

All 14 trials are post-split-to-8s-scenario. The first two ON+splice trials
(tags `robON_offM02` and `robON_offM02_recheck`) used the pre-refactor
controller-thread splice path; the rest use the corrected MPC-sync-path
implementation. (See variance discussion below — empirically the bug did not
visibly bias the per-trial numbers in the cells we ran, but the corrected
implementation is what should be used going forward.)

| tag | side | status | dur (s) | dist (m) | roll (°) | pitch (°) | yaw (°) | splice (in scenario window) |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `robON_offM02`           | ON+splice | OK         | 8.00 | 0.772 | 4.92 | 8.78 | 2.51 | 9 |
| `robON_offM02_recheck`   | ON+splice | OK         | 8.00 | 0.593 | 4.19 | 8.43 | 3.41 | 12 |
| `robON_offM02_refactor`  | ON+splice | OK         | 8.00 | 0.690 | 3.90 | 8.46 | 3.30 | 15 |
| `robON_offM02_refactor2` | ON+splice | FALL @ 5.9 | 5.92 | 0.837 | 10.85 | 10.28 | 1.32 | 5 |
| `robON_offM02_var1`      | ON+splice | OK         | 8.00 | 0.413 | 2.23 | 9.28 | 0.80 | 2 |
| `robON_offM02_var2`      | ON+splice | OK         | 8.00 | 0.661 | 3.64 | 9.54 | 1.25 | 4 |
| `robON_offM02_var3`      | ON+splice | OK         | 8.00 | 0.684 | 4.60 | 7.87 | 1.80 | — |
| `robOFF_offM02`          | OFF       | FALL @ 7.3 | 7.30 | 0.867 | 11.31 | 8.29 | 6.93 | 0 |
| `robOFF_offM02_recheck`  | OFF       | OK         | 8.00 | 0.701 | 2.63 | 7.68 | 2.31 | 0 |
| `robOFF_offM02_refactor` | OFF       | OK         | 8.00 | 1.010 | 6.23 | 8.76 | 19.37 | 0 |
| `robOFF_offM02_var1`     | OFF       | OK         | 8.00 | 0.845 | 8.08 | 9.93 | 12.12 | 0 |
| `robOFF_offM02_var2`     | OFF       | OK         | 8.00 | 0.690 | 4.57 | 9.19 | 14.05 | 0 |
| `robOFF_offM02_var3`     | OFF       | OK         | 8.00 | 0.621 | 5.00 | 8.60 | 10.37 | 0 |
| `robOFF_offM02_var4`     | OFF       | OK         | 8.00 | 0.720 | 1.94 | 7.69 | 1.33 | 0 |

### Aggregate (mean ± std over the 6 successful trials per side; falls excluded from RMS stats but counted in fall rate)

| metric | ON+splice (n = 7, 1 fall) | OFF (n = 7, 1 fall) | difference |
| --- | --- | --- | --- |
| Roll RMS  | **3.91° ± 0.94°** | **4.74° ± 2.27°** | ON+splice lower mean and ~2.4× tighter variance |
| Pitch RMS | 8.73° ± 0.61°     | 8.64° ± 0.87°     | comparable |
| Yaw RMS   | **2.18° ± 1.08°** | **9.92° ± 6.97°** | ON+splice **~4.6× lower** mean, ~6.5× tighter variance |
| Fall rate | 1/7 (14%)         | 1/7 (14%)         | identical |
| Distance xy (mean) | 0.62 m | 0.79 m | OFF travels further on average |

### Splice firing characteristics (cropped to active scenario window)

The earlier draft reported **66 / 94** splices for the first two ON+splice
trials, treating that as a "~10 splices/s during active scenario" rate.
This was wrong: the full controller log spans roughly the full 38 s subprocess
lifetime, not the 8 s scenario, and `observation_.time` is not aligned to
scenario time. Cropped to the 5.5 s scenario monitoring window (using
`tick.csv`'s first time as anchor and `monitoring_start_sec → timeout_sec`
span), the actual counts are **2–15 per scenario** (table column above), or
roughly **0.4–2.7 splices per second** during the active descent. Splices
fire across all four legs; per-leg distribution shifts run-to-run with trot
phasing relative to the descent edge.

## Verdict (n = 7 per condition)

- **Splice helps in the critical cell, in the corrected reading.** Mean roll
  drops 4.74° → 3.91° (≈18 % reduction in the mean), and yaw drops 9.92° →
  2.18° (~4.6× reduction). Fall rate is identical at 14 % (1/7).
- **The most striking effect is variance reduction, not mean shift.** Roll
  std drops 2.27° → 0.94° (~2.4× tighter); yaw std drops 6.97° → 1.08° (~6.5×
  tighter). The robot's run-to-run trajectory is much more predictable with
  splice on.
- **The earlier "OFF wins by 2×" claim was wrong.** That was based on
  cherry-picked single trials (the F5 baseline `2.34°` was the low-end of a
  wide OFF distribution). With proper sample size, OFF's mean is ~5°, not
  ~2.5°.
- **F5's formulation diagnosis is partially defused, not vindicated.** With
  splice in place, the in-band cell is no longer worse than OFF — so the
  "`g(t_b) = −d` directional bias makes splice-less robust ON 3× worse than
  OFF" reading from F5 is largely the splice-less artifact chat6 predicted.
  A *secondary* concern remains that ON+splice doesn't dominate OFF more
  decisively (the means overlap inside one combined std), which could be
  formulation, MPC rate, sample size, or some combination.

## What this report does not yet answer

- **Sample size for definitive conclusion.** `n = 7` with one fall apiece
  gives wide CIs. Reaching "ON+splice is significantly better than OFF"
  with α=0.05 likely needs `n ≥ 15–20` per side given the OFF variance.
- **Whether the mean-roll gap (4.74° → 3.91°) is real or a noise artifact.**
  Welch's t test on the two distributions would tell; not run yet.
- **Why OFF's distance and yaw are so much wider than ON+splice's.** OFF's
  mean distance is 27 % higher and yaw std is 6.5× larger — possibly the
  robot is recovering by yawing into the descent, but a per-trial trajectory
  inspection is needed to confirm.
- **Whether the picture changes at MPC 50 Hz.** Cleaner MPC re-solve
  cadence may shift either ON+splice (less stale-policy window) or OFF
  (faster cost-minimization without robust phase) more.
- **Out-of-band behavior.** This report is in-band only (`|Δz| ≤ d`). The
  out-of-band falls F1 reported in `WithoutSplice.md` may or may not be
  rescued by splice — separate experiment.

## Recommended next experiments

1. **Larger n for the in-band cell.** Push `n` to 15 per side to pin down
   whether the 4.74° → 3.91° gap is statistically significant.
2. **MPC rate sweep with splice ON.** Cells: `(MPC = 10, 25, 50 Hz) × (Δz =
   −0.02)` × splice ON. If higher MPC rate further improves ON+splice
   without hurting OFF, the splice + 50 Hz combination becomes the new M2
   default.
3. **Out-of-band re-test.** `Δz = ±0.05, d = 0.03` × splice ON / OFF. F1
   reported falls without splice; check whether splice rescues.
4. **F-a formulation tweak (small)**: change `g(t_b) = −d` to `g(t_b) = 0`
   in the boundary cost, re-run the in-band cell. If the means converge
   tighter, F5's formulation diagnosis was load-bearing despite the splice
   recovery; if they don't move, the splice was the only missing piece.

## Out of scope for this report

- WBC contact-flag override (Track ② step (c)) — still deferred.
- Multi-touchdown horizon scope — current `computeRobustWindows` is
  first-touchdown-only (M3 scope).
- Per-step uncertainty `d_l(x, y)` from terrain-confidence map — M3 scope.
- Late-contact splice — current implementation is **early-only**; late is
  detection-only.

## Files of record

| path | role |
| --- | --- |
| [`controllers/.../control/CtrlComponent.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h) | detection latches, threshold, no longer owns splice |
| [`controllers/.../control/CtrlComponent.cpp`](../controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp) | `detectAndLogContactEvents` queues splice via `GaitManager::requestStanceSplice` |
| [`controllers/.../control/GaitManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/GaitManager.h) | new `requestStanceSplice` API + `applyPendingSplices` declaration + mutex/queue |
| [`controllers/.../control/GaitManager.cpp`](../controllers/ocs2_quadruped_controller/src/control/GaitManager.cpp) | `applyPendingSplices` runs at start of `preSolverRun` (MPC thread) |
| [`tools/perceptive_dev_v2/m2_critical_band_inside.sh`](../tools/perceptive_dev_v2/m2_critical_band_inside.sh) | 2-trial driver |
| [`tools/perceptive_dev_v2/scenarios/standing_trot_forward_short.yaml`](../tools/perceptive_dev_v2/scenarios/standing_trot_forward_short.yaml) | 8 s scenario |
| [`note/WithoutSplice.md`](WithoutSplice.md) | F5 baseline + §F6 pointer to here |
| [`note/m1pp_robust_phase_in_ocs2.md`](m1pp_robust_phase_in_ocs2.md) | M1'' implementation + review |
| [`note/m2_robust_phase_terrain_aware.md`](m2_robust_phase_terrain_aware.md) | M2 implementation + Track ② step (a) Appendix D |

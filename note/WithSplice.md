# Robust Phase With Schedule Splice — Midterm Report

Companion to [`WithoutSplice.md`](WithoutSplice.md). This file documents the
addition of **Track ② step (b) — schedule splice on sustained measured contact
during a robust window** on top of the M2 robust-phase OCP, the design choices
made (specifically: schedule splice **only**, with WBC contact-flag override
deliberately deferred), and the first critical-cell A/B re-run that resulted.

The framing is "midterm" because:

- The splice is implemented and verified to fire correctly across all four legs.
- The single critical cell from `WithoutSplice.md` §F5 has been re-run. The
  result is a real but partial recovery — splice helps significantly but does
  not close the gap to the no-robust-phase baseline at 10 Hz MPC. Several
  follow-up experiments (MPC rate, formulation tweak, possibly WBC override)
  are queued but not yet run.

## Why splice now (and why splice only)

The third peer review (`chat6.md`, summary in `WithoutSplice.md` §F6) made two
corrections to the F5 reading:

1. **F5's "formulation 문제 확정" was overreach.** The splice-less F5 cell shows
   robust ON has 3× the roll RMS of robust OFF inside the band. F5 attributed
   this to the formulation `g(t_b) = −d` continuing to push the foot below
   perceived terrain. But "without splice, the schedule never updates on
   measured contact" is an alternative explanation that doesn't require
   implicating the formulation: **even with a perfectly correct boundary
   target, the OCP keeps planning towards `g(t_b) = −d` after the foot has
   physically touched, because the schedule still says the leg is in swing
   until the originally scheduled `t_b`.**
2. **The early/late event count from F0 is not a fair ON/OFF metric.** In OFF,
   the early-event log is gated by `isInRobustWindow(...)` which is always
   false, so OFF early-count = 0 by construction (gating, not absence of
   physical contact).

Resolving (1) means running the same critical cell with an event-triggered
schedule splice in place. WBC-side override is deliberately kept off so any
observed gain is attributable to the schedule-side change alone, not a
WBC reactive shortcut. This matches the staging rule from `chat4.md`
(no WBC mixing during OCP-attribution experiments) and the staged scope
in `WithoutSplice.md` §"Implication for Track ②".

## Implementation

### File: [`controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h)

Added one method declaration, one constant, and two per-leg latches alongside
the existing `detectAndLogContactEvents()` plumbing:

```cpp
void detectAndLogContactEvents();   // existing (Track ② step (a))
void spliceStanceForLeg(size_t leg); // new (Track ② step (b))

static constexpr int kEventSpliceSustainedTicks = 5;

feet_array_t<bool> prev_scheduled_contact_{};
feet_array_t<bool> early_event_logged_in_swing_{};
feet_array_t<int>  sustained_early_ticks_{};   // new
feet_array_t<bool> splice_applied_in_swing_{}; // new
```

The header docstring documents the splice intent next to the
`detectAndLogContactEvents` declaration, and lists the four per-leg state
arrays with their reset semantics (latches reset on liftoff = stance→swing
edge in the schedule).

### File: [`controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp`](../controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp)

Two new includes:

```cpp
#include <ocs2_core/misc/Lookup.h>                       // findIndexInTimeArray
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h> // modeNumber2StanceLeg / stanceLeg2ModeNumber
```

`detectAndLogContactEvents()` is extended (CtrlComponent.cpp:206-289) to:

1. Compute `early_now = isInRobustWindow(leg, t) ∧ measured_contact ∧ !scheduled_contact` per leg per tick.
2. Maintain `sustained_early_ticks_[leg]`: incremented while `early_now`, reset
   to 0 otherwise.
3. Reset both `splice_applied_in_swing_[leg]` and `sustained_early_ticks_[leg]`
   on the leg's liftoff edge (the same edge that already resets the
   `early_event_logged_in_swing_` latch).
4. When `early_now ∧ sustained_early_ticks_[leg] ≥ kEventSpliceSustainedTicks
   ∧ !splice_applied_in_swing_[leg]`, call `spliceStanceForLeg(leg)` and
   latch `splice_applied_in_swing_[leg]`.

The 5-tick threshold debounces against single-tick contact spikes from sensor
noise. At the controller's 1000 Hz `update_rate`, that's a 5 ms persistence
requirement before splice fires — short enough that the splice is still
useful at 10 Hz MPC (which has ~100 ms between solves) but long enough to
filter the worst sensor noise.

`spliceStanceForLeg(size_t leg)` (CtrlComponent.cpp:291-343) performs the
schedule rewrite:

```cpp
const auto& gaitSchedulePtr =
    legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule();
const scalar_t t = observation_.time;
const scalar_t timeHorizon = legged_interface_->mpcSettings().timeHorizon_;
ModeSchedule sched = gaitSchedulePtr->getModeSchedule(t - 0.5, t + timeHorizon + 0.5);

const int curPhaseInt = lookup::findIndexInTimeArray(sched.eventTimes, t);
const size_t curPhase = static_cast<size_t>(
    std::clamp<int>(curPhaseInt, 0, static_cast<int>(sched.modeSequence.size()) - 1));

contact_flag_t curFlags = modeNumber2StanceLeg(sched.modeSequence[curPhase]);
if (curFlags[leg]) return;                       // already stance — nothing to splice
curFlags[leg] = true;
const size_t newMode = stanceLeg2ModeNumber(curFlags);

sched.eventTimes.insert(sched.eventTimes.begin() + curPhase, t);
sched.modeSequence.insert(sched.modeSequence.begin() + curPhase + 1, newMode);

gaitSchedulePtr->setModeSchedule(sched);
```

Key points:

- **Slice width.** `[t − 0.5, t + horizon + 0.5]` is wide enough to keep the
  whole MPC horizon plus a small past tail. `getModeSchedule` is the
  GaitSchedule API used to materialize a finite slice from the rolling
  template.
- **Phase index lookup.** `lookup::findIndexInTimeArray` is the same lookup
  used elsewhere in the perceptive code (e.g.
  `ConvexRegionSelector.cpp:33,39,45`) and respects OCS2's exact-time
  semantics.
- **Mode bit flip.** `modeNumber2StanceLeg` decodes the current 4-bit contact
  pattern, we set the affected leg to stance, and `stanceLeg2ModeNumber`
  re-encodes. This works for any underlying gait (trot, stand,
  flying-trot, etc.) because the encoding is gait-agnostic.
- **Insertion semantics.** `ModeSchedule` invariant is
  `modeSequence[i]` applies to `[eventTimes[i-1], eventTimes[i])` with
  `|eventTimes| = |modeSequence| − 1`. Inserting at `(curPhase, curPhase+1)`
  preserves this invariant and means: phase `curPhase` keeps
  `[…, t)`, the new phase `curPhase+1` covers `[t, original next)`, and the
  rest of the sequence shifts right by one. The original `next` mode (and
  everything after) is preserved exactly.
- **`setModeSchedule` is the persistent install.** Once installed, the next
  `MPC_MRT_Interface::advanceMpc()` solve consumes the spliced schedule
  through the existing reference-manager → constraint pipeline. We do
  **not** touch the in-flight policy; we only change what the next solve
  sees.
- **Idempotence.** The early-return `if (curFlags[leg]) return;` makes splicing
  an already-stance leg a no-op. This protects against a race where the
  schedule already updated between detection and splice.
- **No WBC mutation.** The WBC continues consuming the current MPC policy at
  1 kHz; only after the next ~100 ms (at 10 Hz MPC) does the spliced
  schedule manifest in the policy the WBC is tracking. This is the
  intended scope of step (b).

### Logging

Each splice fires one `[robust_splice]` line per (leg, splice_event):

```
[robust_splice] leg=2 t=7.210 mode_old=9 mode_new=11 (splice stance for sustained early contact)
```

`mode_old` is the pre-splice current mode and `mode_new` is the
stance-flipped variant; both decode through `modeNumber2StanceLeg` if needed
for verification.

### Build

`colcon build --packages-select ocs2_quadruped_controller --symlink-install` —
clean build, no warnings introduced. Splice path compiles into the
`ros2_control_node` plugin loaded by the `ocs2_quadruped_controller`
controller plugin.

## Experiment

### Cell selection

Same single critical cell as `WithoutSplice.md` §F5 — the smallest cell
that should expose the splice's effect cleanly:

| parameter | value | rationale |
| --- | --- | --- |
| `terrain_z_offset` (Δz) | −0.02 m | inside the robust band (`|Δz| ≤ d`) — the design promise *should* hold here |
| `d` (robust half-width) | 0.03 m | so `[z_perc − d, z_perc + d] = [0.05, 0.11]` brackets `z_actual = 0.10` ✓ |
| `mpcDesiredFrequency` | 10 Hz | matches the F5 baseline; lowest viable rate that still solves in real time |
| scene | `basic_step_short` (box1 z=0.20 / box2 z=0.10) | descent edge at `x=0.30` |
| scenario | `standing_trot_forward_short` (8 s) | shortened from 16 s per user request; phases preserved (stand → enter OCS2 → forward 0.3 m/s × 3 s → stop) |
| splice threshold | `kEventSpliceSustainedTicks = 5` | 5 ms at 1 kHz controller rate |
| WBC override | OFF | deliberate, see "Why splice now" above |

Two trials per condition (initial + recheck) to surface the simulator
flakiness already characterized in `WithoutSplice.md` §F0. Driver script
unchanged from F5: [`tools/perceptive_dev_v2/m2_critical_band_inside.sh`](../tools/perceptive_dev_v2/m2_critical_band_inside.sh).

### Results

Aggregated from
[`tools/perceptive_dev_v2/results/20260509_045312_*_crit_band_robON_offM02/`](../tools/perceptive_dev_v2/results/),
[`20260509_045405_*_crit_band_robOFF_offM02/`](../tools/perceptive_dev_v2/results/),
[`20260509_045942_*_crit_band_robOFF_offM02_recheck/`](../tools/perceptive_dev_v2/results/), and
[`20260509_050048_*_crit_band_robON_offM02_recheck/`](../tools/perceptive_dev_v2/results/).

| Config | Run | Success | Roll RMS | Pitch RMS | Yaw RMS | Dist xy | Splice events |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| ON-without-splice (F5 baseline, commit 3101e1b) | 1 | ✓ | 7.27° | 8.04° | 9.65° | 0.726 m | n/a |
| OFF (F5 baseline, commit 3101e1b) | 1 | ✓ | 2.34° | 7.78° | 0.91° | 0.740 m | n/a |
| **ON+splice** | 1 | ✓ | **4.92°** | 8.78° | 2.51° | 0.772 m | **66** |
| **ON+splice** | 2 (recheck) | ✓ | **4.19°** | 8.43° | 3.41° | 0.593 m | **94** |
| OFF | 1 | ✗ fall@7.29 s (roll_limit) | 11.31° | 8.29° | 6.93° | 0.867 m | 0 |
| OFF | 2 (recheck) | ✓ | 2.63° | 7.68° | 2.31° | 0.701 m | 0 |

Notes on the table:

- The first OFF trial was a flake. Recheck restored consistency with the F5
  baseline (2.63° vs 2.34°). The OFF-recheck zero splice count also
  confirms the splice path is correctly gated by `isInRobustWindow` —
  always false in OFF, so the splice trigger condition never holds.
- ON+splice shows roll RMS in the 4–5° range across both trials, distance
  travelled roughly comparable to OFF (~0.6–0.8 m), and no falls. Yaw RMS
  drops dramatically (9.65° in F5 baseline → ~3° here).

### Splice firing characteristics

```
ON+splice run 1: leg0=24, leg1=6,  leg2=9,  leg3=27   (total 66)
ON+splice run 2: leg0=5,  leg1=26, leg2=45, leg3=18   (total 94)
```

All four legs are exercised. The leg distribution shifts run-to-run with
trot-cycle phasing relative to the descent edge — this is expected
because the descent happens at a single `x` coordinate (`x = 0.30`) and
which leg(s) are mid-swing at that x depends on the cycle phase at the
edge crossing. At ~10 splices/s averaged over the 8 s scenario, splice
triggering is **routine, not rare** — early contact happens every time a
swing foot enters the robust window over the lower step (since
`z_actual = 0.10 > z_perc − d = 0.05`, the foot meets ground roughly
half-way through the window).

## Verdict

- **Splice helps.** Roll RMS drops from 7.27° (no splice, F5) to ~4.5°
  (avg of two ON+splice runs). ~38 % reduction in the same critical cell.
  Yaw RMS drops 9.65° → ~3°.
- **Splice does NOT fully recover OFF behavior.** ON+splice still has
  ~1.7× the roll of OFF (~4.5° vs ~2.5°).
- **Chat6's hypothesis is partially confirmed.** Splice is a real and
  large effect — necessary, but not by itself sufficient to make the
  in-band cell match the no-robust-phase baseline at 10 Hz MPC.
- **Residual gap explanation (provisional).** With splice fired, there is
  still a `5 ms (detection latch) + ~100 ms (next MPC solve)` ≈ 105 ms
  window per swing during which the OCP-installed policy continues to
  push the foot down towards `z = z_perc − d`, even though the leg has
  physically touched and the schedule has been spliced to "stance from
  now". The WBC dutifully tracks that stale policy at 1 kHz until the
  next MPC solve. This window explains why the splice can't fully
  recover OFF: the formulation's directional bias (F5's diagnosis) is
  still partially active during this stale-policy interval.

## What this report does not yet answer

- **Whether higher MPC rate alone closes the residual gap.** The simplest
  follow-up: re-run the same critical cell at MPC 50 Hz with splice ON.
  If ON+splice@50Hz now matches OFF, the residual gap was indeed about
  MPC re-solve latency and the formulation can stay as-is.
- **Whether F5's formulation diagnosis is independently load-bearing.** If
  ON+splice@50Hz still gaps, then F-a (`g(t_b) = 0` instead of `−d`) is
  the next test — and the F5 diagnosis is upgraded from "provisional" to
  "necessary fix".
- **Whether splice generalizes to `|Δz| > d`.** The cell here is in-band.
  An out-of-band cell (e.g. Δz = −0.05 with d = 0.03) was already a fall
  in F1 without splice; whether splice alone rescues it is a separate
  experiment.
- **Variance characterization.** Two trials per condition is enough to
  catch a flake, not enough for a confidence interval. Multi-seed
  expansion (5–10 trials per cell) is a separate ask.

## Recommended next experiment

A two-cell follow-up, using exactly the same `m2_critical_band_inside.sh`
driver but with `--mpc-frequency 50` and the splice already in place:

| Cell | Δz | d | MPC rate | Splice | Expected if "MPC rate is the only residual" |
| --- | ---: | ---: | ---: | --- | --- |
| 1 | −0.02 | 0.03 | 50 Hz | ON | roll RMS within ~10 % of OFF (~2.5°) |
| 2 | −0.02 | 0.03 | 50 Hz | OFF | roll RMS unchanged from existing OFF (~2.5°) — control |

If cell 1 hits ~2.5°, the residual was indeed re-solve latency; the
splice + 50 Hz combination is the M2-onwards default and we can resume
the broader band-inside / band-outside sweeps from `WithoutSplice.md` §6.
If cell 1 stays at ~4.5°, F-a (formulation tweak) is the next move.

## Out of scope for this report

- WBC contact-flag override (Track ② step (c)) — still deferred.
- F-a / F-b / F-c / F-d formulation candidates from `WithoutSplice.md` §F5 —
  none implemented yet; pending the MPC 50 Hz result above.
- Multi-touchdown horizon scope — current `computeRobustWindows` is
  first-touchdown-only (M3 scope).
- Per-step uncertainty `d_l(x, y)` from terrain-confidence map — M3 scope.

## Files of record

| path | role |
| --- | --- |
| [`controllers/.../control/CtrlComponent.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h) | splice declarations, latches, threshold constant |
| [`controllers/.../control/CtrlComponent.cpp`](../controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp) | `detectAndLogContactEvents` + new `spliceStanceForLeg` |
| [`tools/perceptive_dev_v2/m2_critical_band_inside.sh`](../tools/perceptive_dev_v2/m2_critical_band_inside.sh) | 2-trial driver (ON / OFF, Δz=−0.02, d=0.03, 10 Hz) — unchanged from F5 |
| [`tools/perceptive_dev_v2/scenarios/standing_trot_forward_short.yaml`](../tools/perceptive_dev_v2/scenarios/standing_trot_forward_short.yaml) | 8 s scenario used here and in F5 |
| [`note/WithoutSplice.md`](WithoutSplice.md) | F5 baseline (no-splice critical cell) and §F6 brief pointer to this report |
| [`note/m1pp_robust_phase_in_ocs2.md`](m1pp_robust_phase_in_ocs2.md) | M1'' implementation + review |
| [`note/m2_robust_phase_terrain_aware.md`](m2_robust_phase_terrain_aware.md) | M2 implementation + Track ② step (a) Appendix D |

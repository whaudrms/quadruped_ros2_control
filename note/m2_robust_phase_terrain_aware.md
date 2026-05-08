# M2 — Robust phase with per-leg ConvexRegionSelector terrain projection

Picks up from M1''. The boundary equality `g(x_a)=+d` / `g(x_b)=-d` was previously defined
relative to a single hard-coded ground level (`task.info: robustPhase.terrain_z_M1`).
M2 replaces that scalar with the **stance-side terrain projection** of each leg's next
upcoming touchdown, pulled from the existing perceptive infrastructure
(`ConvexRegionSelector`). The OCP can now ask "land 3 cm below the actual surface where
this leg is about to touch", not "land 3 cm below an arbitrary reference height".

## What changed (one commit, `9509863`)

Single conceptual change with three small edits.

1. `RobustPhaseSettings` gains `std::string terrain_source` — `"flat"` (M1'' fallback)
   or `"convex_region"` (M2). Plumbed through `loadRobustPhaseSettings`.

2. `PerceptiveLeggedReferenceManager::computeRobustWindows` (in
   [`src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp))
   — after computing `(t_a, t_b, stancePhase)` per leg, the `convex_region` branch reads
   the stance-side projection via the **phase-index path**:

   ```cpp
   const auto perLegProjections = convexRegionSelectorPtr_->getProjections(leg);
   if (stancePhase < perLegProjections.size()) {
       const auto& proj = perLegProjections[stancePhase];
       if (proj.regionPtr != nullptr) {
           w.p_plane = proj.positionInWorld;
           // n stays e_z for now (horizontal step surfaces).
       }
   }
   ```

   We deliberately do NOT use `getProjection(leg, t_b + eps)`. At an exact event time
   `ModeSchedule::modeAtTime` returns the *lower* count (swing-side), so a time-based
   query at `t_b` would give the swing projection one phase too early. The phase-index
   path mirrors what the existing perceptive touchdown-height code already does
   ([`PerceptiveLeggedReferenceManager.cpp:443-451`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp#L443-L451)
   reads `projections[i+1].positionInWorld.z()` at the swing→stance boundary, where
   `i+1 == stancePhase`). Sharing the same index guarantees our guard plane is exactly
   the plane the swing planner is aiming for.

3. [`task.info`](../descriptions/unitree/go2_description/config/ocs2/task.info) default
   flipped: `terrain_source = convex_region`. `terrain_z_M1` retained as fallback for
   any leg whose `proj.regionPtr` happens to be `nullptr` in a given cycle (perception
   gap), so the OCP stays well-defined.

## Verification (M2 done)

Single trial on `basic_step_short` with `standing_trot_forward`. Box layout:
floor at `z=0`, box1 (upper) top at `z=0.20` (edge at `x=0.30`), box2 (lower) top at
`z=0.10` (spans `x=[0.3, 2.3]`).

Trial output:
`tools/perceptive_dev_v2/results/20260507_220657_basic_step_short_perceptive_dev_v2_robust_M2_first/`.

| metric | value |
|---|---|
| success | true |
| fall | none |
| distance_xy | 1.17 m forward |
| start z / end z | 0.317 → 0.463 (robot navigates the step but base does not fully descend; see "Body descent" below) |
| pitch_rms | 5.62° |

The relevant verification numbers come from the per-cycle `[robust_phase]` log + the
foot-z plot, not the body-pose summary.

### Per-leg `p_plane.z` evolves over time as expected

Sample (FL leg, sub-sampled to one entry per second, restricted to `active=1`
robust windows):

```
t=5.78  pz=0.2   ← leg's next touchdown is on box1 (still on upper step)
t=6.04  pz=0.2
t=7.02  pz=0.2
t=7.84  pz=0.1   ← switched to box2 (perception sees the descent target now)
t=8.01  pz=0.1
t=9.04  pz=0.1
…       pz=0.1   (stays on box2 for the rest of the trial)
```

Two distinct `pz` values appear in the **active** robust windows for FL: `0.2`
(box1 top) and `0.1` (box2 top). The phase-index path correctly picks the
stance-side projection on every cycle when a window is active. `pz = 0.0` does
also appear in the raw log, but only on cycles where `active = 0` (no upcoming
touchdown found in horizon, or the leg is currently in stance) — it is the
default-initialized field, not an active "floor null-projection fallback". So
this trial demonstrates correct projection lookup on `{0.1, 0.2}`; the
null-projection fallback path exists in code but is not exercised here.

### Pre-window target switch caught mid-descent (around t ≈ 7.8–8.4 s)

The swing-leg's `next-touchdown` projection switches **before** the next robust
window opens, not inside one: at `t = 7.84` the per-cycle log first reports
`pz = 0.1` (box2), and the corresponding active robust window for that touchdown
runs `t_a = 8.266 → t_b = 8.366`. So the transition is "during free swing,
the future touchdown target was retargeted from box1 to box2; once the robust
window for that retargeted touchdown opens, the boundary equality already aims
at box2". The foot z trace caught mid-air between the two surfaces:

```
nominal target box1 (t_b): foot_frame_z = +0.23  (= 0.20 + 0.06 - 0.03)
new      target box2 (t_b): foot_frame_z = +0.13  (= 0.10 + 0.06 - 0.03)
achieved foot_frame_z at t_b: +0.177
   → 5.3 cm below the box1 target (foot has cleared the edge, descending)
   → 4.7 cm above the box2 target (still in mid-air toward the lower step)
```

### Steady-state residuals on box2 (t > 10 s)

```
target z_foot at t_b: +0.13   achieved +0.156   residual +0.027 m
target z_foot at t_a: +0.19   achieved +0.260   residual +0.070 m
```

Same soft-penalty trade-off pattern as M1'' trial 4: terminal `t_b` pull strong, start
`t_a` under-tracks. Per `plan.md` sub-decision 1, the documented fallback if this
under-tracking matters in M3 / hardware tests is to promote the boundary to hard
`equalityConstraintPtr->add` instead of `StateSoftConstraint`.

### Body descent quality — A/B confirmed (robust phase NOT the cause)

The body z stayed near box1's stance height (`min_z = 0.317`, `start_z = 0.317`,
`end_z = 0.463`) — the robot does not gracefully transfer body weight onto box2 even
though the swing legs reach the box2 surface. `pitch_rms` ≈ 5.6° (vs 1.47° on flat in
M1'' trial 4) reflects the body straddling the step.

The chatM2review fix #4 left this attribution as "suspected base-ref / WBC issue;
needs A/B confirmation". The A/B trial has now been run (after the Stage 8/9 cleanup
commit `2c4b986`):

| metric                  | M2 (robust ON, `9509863`) | A/B (`robustPhase.enabled=false`) | Δ (off vs on) |
|---|---|---|---|
| success / fall          | True / none               | True / none                       | same |
| dist [m]                | 1.170                     | **1.573**                         | +0.40 m (+34 %) |
| start_z [m]             | 0.317                     | 0.317                             | identical |
| **end_z [m]**           | **0.463**                 | **0.459**                         | **−0.004 (≈)** |
| **min_z [m]**           | **0.317**                 | **0.317**                         | **identical** |
| pitch_rms [°]           | 5.62                      | 5.78                              | +0.16 (≈) |
| roll_rms [°]            | 1.21                      | 2.20                              | **+1.00 (worse off)** |
| base_z_std [m]          | 0.046                     | 0.049                             | ≈ |

A/B results, same `basic_step_short` / `standing_trot_forward`:
`results/20260507_220657_..._robust_M2_first/` vs
`results/20260508_022845_..._M2_AB_baseline_off/`.

What this confirms:

- **Body descent gap is independent of robust phase.** `min_z` and `end_z` are
  identical between robust ON and robust OFF; the robot fails to transfer body weight
  onto box2 in both cases. The straddling and the body-z stuck-on-box1 behavior are
  driven by the perceptive base-reference modification + WBC, not by the M2 robust
  guard. So the original (provisional) attribution was correct.
- **Robust phase actually improves roll stability.** `roll_rms` is roughly half with
  the robust phase on (1.21° vs 2.20°). Plausible mechanism: the boundary equality
  forces a more consistent foot landing height per cycle, which gives the WBC a
  steadier set of stance contacts.
- **Robust phase costs ~34 % forward progress.** `dist` drops from 1.57 m (off) to
  1.17 m (on). Plausible mechanism: the soft penalty pulling foot z toward
  `p_plane.z + offset ± d` slows the swing slightly, and the deactivated
  `NormalVelocityConstraint` removes the strong "land on the nominal touchdown
  height" signal that the swing planner normally uses.

**M2's plumbing contribution — the OCP now plans a foot trajectory that lands on box2
— is a necessary precondition for any subsequent body descent fix**, but the body
descent fix itself lives in the base-reference / WBC stack and is queued separately
(Next milestones table row #2).

So M2 verification is: **per-leg terrain projection plumbing works end-to-end; the
boundary equality is now defined relative to the (horizontal) box/step surface the leg
targets, and the perception pipeline correctly identifies the descent target one swing
before touchdown**. The "actual surface" claim is still scoped to horizontal box/step
terrain because the plane normal `n` is currently hard-wired to `e_z`; inclined surfaces
need M2.x (extract `n` from `proj.regionPtr->transformPlaneToWorld.linear().col(2)`).
Body descent quality is a separate base-reference / WBC issue (A/B verified — see the
"Body descent quality — A/B confirmed" subsection above) and is queued under "Next
milestones" rather than blocking M2.

## Files (read-or-modify map)

Single-commit change set: `9509863`. Three small edits, no new files.

| Path | Change |
|---|---|
| [`include/.../perceptive/interface/PerceptiveLeggedReferenceManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h) | Added `std::string terrain_source` to `RobustPhaseSettings` (`"flat"` or `"convex_region"`). |
| [`src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp) | `computeRobustWindows` adds the `convex_region` branch (phase-index projection lookup, null-projection flat fallback). `loadRobustPhaseSettings` parses the new field. |
| [`descriptions/unitree/go2_description/config/ocs2/task.info`](../descriptions/unitree/go2_description/config/ocs2/task.info) | Default `terrain_source = convex_region`. `terrain_z_M1` retained as fallback. |

## Next milestones — status

After M2 the in-OCP robust phase plumbing is complete enough that the Python `robust_refine`
external layer is no longer load-bearing. Original four candidates with their current
status:

| # | candidate | scope | status |
|---|---|---|---|
| 1 | Stage 8/9 cleanup | Delete `RefinedPolicyReader`, `MpcDumpRecorder`, the `mpc_one_shot_*` machinery, the `[robust_refine] override=…` per-cycle log lines; trim `tools/perceptive_dev_v2` of `--enable-refiner-swap` / oneshot scenarios | **DONE — commit `2c4b986`** (17 files, +25/−2038) |
| 2 | Body descent A/B | Address (or first attribute) the body-z gap exposed by M2 (`min_z = start_z`, `pitch_rms 5.6°` on `basic_step_short`). Touch points: `PerceptiveLeggedReferenceManager.cpp:188-294` base-reference modification, WBC weights | **A/B DONE — commit `8d6b027`** confirmed gap is *independent* of robust phase (`min_z` and `end_z` identical with robust ON vs OFF). Tuning of base-ref / WBC remains queued separately |
| ★ | **Track ② — early/late contact event handling** | `chat1.md`'s original ① → ② program. `CtrlComponent::updateState` compares `observation_.mode` (measured) to `referenceManagerPtr_->getContactFlags(t)` (scheduled); on early contact during a robust window, splice a stance phase into the schedule via `gait_schedule_ptr_->setModeSchedule(...)`. Optionally also override the WBC contact flag for the current tick. Two sub-steps: (a) detection + per-event log only (~0.5 d), (b) actual schedule splice + WBC override (~1–2 d). | **NEXT** (per the original ① → ② plan in `chat1.md`) |
| 3 | M3 — multi-touchdown + optional impulse cost | Generalize `computeRobustWindows` to record up to `M_horizon` upcoming touchdowns per leg (not just the first); optionally add the `λ = (J M⁻¹ Jᵀ)⁻¹ J v⁻` pair-impulse cost from the original `robust_refine` formulation, gated on a CppAD differentiability smoke test for `crba`+inverse | parked (after Track ②) |
| 4 | M2.x — terrain-normal extraction | Extract `n` from `proj.regionPtr->transformPlaneToWorld.linear().col(2)` so inclined surfaces use a proper terrain-normal guard rather than world `e_z` | parked (only matters with inclined scenes) |
| 5 | Body descent tuning | Now that A/B confirms the body-z gap is base-ref / WBC, tune `PerceptiveLeggedReferenceManager.cpp:188-294` (height-blend / pitch-blend / down-step commit distance) and/or WBC tracking weights | parked (orthogonal to robust-phase work) |

**Picked order**: #1 → #2 (both done) → ★ Track ② → then re-evaluate.

---

## Appendix A — Self-diagnosis vs the original ①→② plan

The original plan (recorded in `chat1.md`) split the work into two tracks, not the
M1''/M2 progression we ended up using:

> **Track ①** — event handling 끔. MPC 안에 robust phase 제약 (`g(x_a)=+d, g(x_b)=-d, ġ ≤ 0`) 만 넣고 검증. 실행 중 contact 감지해도 mode schedule 안 바꿈.
>
> **Track ②** — event handling 켬. robust window 안 measured contact → 즉시 WBC contact override + 다음 MPC solve에서 schedule을 stance로 갱신.

Mapping our actual milestones to those tracks:

| our milestone | scope | track |
|---|---|---|
| M1'' (commits `0fe5079`, `508dbed`, `d46a921`) | flat ground OCP plumbing verification | **Track ①** |
| M2 (commits `9509863`, `f7efd82`, `785f68d`) | terrain-aware via `ConvexRegionSelector` projection | **Track ①** |
| early contact event handling | (not started) | Track ② |
| late contact handling | (not started) | Track ② |

So **M1'' and M2 are two refinement steps inside Track ①**; we have not entered Track ②
at all. This matches the recommended ordering ("①→②, do ① first because ② adds too many
debug-cause sources at once") and is consistent with the `chat3.md` reviewer's reading:

> 현재 M2 구현 기준으로는 early contact가 와도 ReferenceManager/MPC가 event-triggered로
> stance 전환하지 않습니다. ... event 처리까지 하려면 [WBC contact latch + MPC schedule
> 갱신 + stance constraint 조기 활성화 + robust window 종료] 로직이 추가되어야 합니다.
> 지금 M2에는 이 부분이 없습니다.

The "Next milestones" table above lists candidate Track ① extensions
(M2.x, M3, body-descent tuning, Stage 8/9 cleanup). **Returning to the original plan
would mean queuing Track ② (the early/late contact event handling) explicitly as a
separate row** — which is what should happen if "complete the ①→② program" is the
priority, rather than further depth in Track ①.

## Appendix B — Peer review on the M2 note + applied fixes

`chatM2review.md` flagged four wording-level issues in the initial M2 note. Code is
unchanged; the fixes are doc-only and applied in the same commit as this appendix.

### Original review (verbatim)

> 1. [보고서 line 81-83](.../m2_robust_phase_terrain_aware.md:81)의 `{0.0, 0.1, 0.2}`
>    설명은 조심해야 합니다. 로그를 보면 `active=1` robust window에서 `pz=0.0`은 안 보입니다.
>    `pz=0`은 주로 inactive window의 default 값입니다. 따라서 "floor null-projection
>    fallback이 나타났다"고 쓰면 과합니다. 더 정확히는: **active robust windows에서는
>    box1 `0.2`와 box2 `0.1`이 확인됨. `0.0` fallback은 이번 trial에서 실제 active robust
>    target으로 검증되지는 않음.**
>
> 2. [line 85-96](.../m2_robust_phase_terrain_aware.md:85)의 "switched mid-window" 표현은
>    약간 부정확합니다. 로그상 FL은 `t=7.84`에 `pz=0.1`로 바뀌고, 그 window는 `t_a=8.266,
>    t_b=8.366`입니다. 즉 robust window 안에서 바뀐 게 아니라 **swing 중, robust window
>    시작 전에 future touchdown target이 box2로 바뀐 것**에 가깝습니다.
>
> 3. [line 132-135](.../m2_robust_phase_terrain_aware.md:132)의 "actual surface"는
>    horizontal step surfaces에서는 맞습니다. 다만 현재 `n`은 여전히 `e_z`라서 slope/ramp
>    까지 포함한 진짜 terrain-normal guard는 아직 아닙니다. 보고서에 "horizontal box/step
>    terrain 기준"이라고 한정하면 더 엄밀합니다.
>
> 4. Body descent를 "not robust-phase issue"라고 단정하기보다는 "primarily
>    base-reference/WBC issue로 보이지만, no-robust M2 baseline과 비교 필요" 정도가
>    안전합니다. robust foot target이 바뀌면 contact timing과 WBC tracking에도 간접 영향은
>    줄 수 있습니다.

### Applied fixes (this doc-only commit)

| # | severity | issue (one-liner) | fix |
|---|---|---|---|
| 1 | medium | "Three unique pz values: floor (null-projection fallback), box2 top, box1 top" — overstated; `pz = 0.0` only appeared on `active = 0` cycles (default-initialised field), not as an exercised fallback. | "Per-leg pz" subsection now restricts the listed values to `active = 1` cycles (`{0.1, 0.2}`), and explicitly notes that the `pz = 0.0` flat fallback path *exists in code but is not exercised in this trial*. |
| 2 | medium | "switched mid-window from box1 to box2" — actually the projection retargeted **before** the next robust window opened (FL log: `pz` changes to `0.1` at `t = 7.84`, the matching robust window runs `t_a = 8.266 → t_b = 8.366`). | Renamed the subsection to "Pre-window target switch caught mid-descent" and rewrote the description as "during free swing, the future touchdown target was retargeted from box1 to box2; once the robust window for that retargeted touchdown opens, the boundary equality already aims at box2". |
| 3 | low | "actual surface" was unconditional; `n = e_z` still, so the claim only holds for horizontal box/step surfaces. | Verification summary qualified: "boundary equality is now defined relative to the (horizontal) box/step surface the leg targets". Inclined-terrain support deferred to M2.x as before. |
| 4 | medium | "not a robust-phase issue" was definitive without an A/B baseline comparison. | "Body descent quality" subsection retitled "(suspected base-ref / WBC issue; needs A/B confirmation)". Reads "the most likely root cause is base-reference + WBC tracking, *not* the robust phase itself, but this attribution is provisional and needs an A/B trial (`robustPhase.enabled = false` on the same scene/scenario) to confirm". The "M2 plumbing as necessary precondition" point is preserved. **Status: A/B done in commit `8d6b027`, see Appendix C.** |

## Appendix C — Post-M2 progress (Next milestones #1 + #2)

Two cleanup-class items from the "Next milestones" table executed back-to-back after
the post-M2 review fixes landed. Both were prerequisites for moving on to Track ②
(early/late contact event handling per `chat1.md`'s original ① → ② plan).

### C.1 Stage 8/9 cleanup — commit `2c4b986`

`17 files changed, +25 / −2038`. The in-OCP robust phase (M1''/M2) fully replaces the
external Python `robust_refine` IPC layer and the one-shot OCP variant; with Track ②
next, leaving the dead paths in place would just add noise to debugging.

Removed C++ (deleted files):

- `controllers/.../control/RefinedPolicyReader.{h,cpp}` — polled `/dev/shm/.../out` for refined plans
- `controllers/.../control/MpcDumpRecorder.{h,cpp}` — wrote MPC `PrimalSolution` CSV per cycle

Removed C++ (gutted in place):

- `CtrlComponent` fields `refined_policy_reader_`, `refined_seq_`, `refined_mtx_`, `refined_init_time_`, `refined_time_traj_`, `refined_state_traj_`, `refined_input_traj_`, `mpc_dump_recorder_`, `mpc_one_shot_`, `mpc_one_shot_solves_`, `mpc_one_shot_done_`
- `CtrlComponent::init()` one-shot bootstrap branch (~90 lines) collapsed back to the original receding-horizon path
- MPC thread loop synchronous IPC + refined-policy polling block (~80 lines) collapsed back to plain `mpc_mrt_interface_->advanceMpc()`
- `GaitManager::primeForOneShot` (only called by one-shot init)
- `StateOCS2::run` REFINED POLICY OVERRIDE block + per-cycle `[robust_refine] override=…` status log + `tick.csv` `refined_active` column
- `StateOCS2::last_refined_log_time_` throttle field

Removed launch params (in `mujoco.launch.py`): `ocs2_dump_dir`, `ocs2_dump_max_cycles`,
`ocs2_dump_min_interval_sec`, `enable_refiner_swap`, `refined_dir`,
`refined_timeout_sec`, `mpc_one_shot`, `mpc_one_shot_solves`. CMakeLists entries for
the two deleted `.cpp` removed.

Removed tooling:

- `tools/perceptive_dev_v2/run_trial.py` CLI: `--ocs2-dump-*`, `--enable-refiner`,
  `--enable-refiner-swap`, `--refiner-python`, `--refiner-daemon-script`,
  `--refined-dir`, `--refined-timeout-sec`, `--no-wipe-refine-out`, `--mpc-one-shot`,
  `--mpc-one-shot-solves`; sidecar refiner spawn block; `/dev/shm/robust_refine` wipe
  block; `refiner_daemon.py` from `LINGERING_PROCESS_PATTERNS`
- `tools/perceptive_dev_v2/scenarios/standing_trot_oneshot.yaml`
- `tools/perceptive_dev_v2/plot_tracking_error.py` (baseline-vs-refined plot)
- `tools/perceptive_dev_v2/noise_sweep.py` (Stage 8 sweep driver)
- `tools/perceptive_dev_v2/plot_sweep_summary.py` (Stage 8 sweep summarizer)

Untouched on this branch:

- `contact_timing_uncertainty/robust_refine/` (Python codebase) — kept on the `offline`
  branch as a research archive; `replan` just stops wiring it into the controller.
- `tick_log_path` launch param + per-tick CSV emitter — still useful for the in-OCP
  robust phase plot (`plot_robust_phase.py`).

Smoke trial after cleanup (`basic_step_short` / `standing_trot_forward`):
`success=True`, no fall, `dist=1.245 m`, `pitch_rms=5.69°` — same ballpark as the M2
trial (1.17 m, 5.62°). `controller.log` shows zero hits for
`robust_refine`/`RefinedPolicy`/`refined override` (vs ~80 per trial before); 4664
`[robust_phase]` log lines confirm the Track ① path is still active. `tick.csv` header
now starts with `t,opt_x0,…` (no `refined_active` column).

### C.2 Body descent A/B — commit `8d6b027`

Closes the open A/B item from chatM2review fix #4 (Appendix B row 4). Same scene
(`basic_step_short`), same scenario (`standing_trot_forward`), same controller build,
only `task.info: robustPhase.enabled` toggled.

| metric          | M2 (robust ON, `9509863`) | A/B (`enabled=false`) | Δ (off vs on) |
|---|---|---|---|
| success / fall  | True / none               | True / none           | same |
| dist [m]        | 1.170                     | **1.573**             | +0.40 (+34 %) |
| start_z [m]     | 0.317                     | 0.317                 | identical |
| **end_z [m]**   | **0.463**                 | **0.459**             | **−0.004 (≈)** |
| **min_z [m]**   | **0.317**                 | **0.317**             | **identical** |
| pitch_rms [°]   | 5.62                      | 5.78                  | +0.16 (≈) |
| roll_rms [°]    | 1.21                      | 2.20                  | **+1.00 (worse off)** |
| base_z_std [m]  | 0.046                     | 0.049                 | ≈ |

Three things this confirms:

1. **Body descent gap is independent of robust phase.** `min_z` and `end_z` are
   identical with robust ON vs OFF. The straddling and body-z stuck-on-box1 behavior
   are driven by the perceptive base-reference modification + WBC, not by the M2
   robust guard. Original (provisional) attribution is correct.
2. **Robust phase actually improves roll stability.** `roll_rms` is roughly half with
   the robust phase on (1.21° vs 2.20°). Plausible mechanism: the boundary equality
   forces a more consistent foot landing height per cycle.
3. **Robust phase costs ~34 % forward progress.** `dist` drops from 1.57 m (off) to
   1.17 m (on). Plausible mechanism: the soft penalty pulls foot z toward
   `p_plane.z + offset ± d`, and the deactivated `NormalVelocityConstraint` removes
   the strong "land on the nominal touchdown height" signal the swing planner used.

Trial artifacts:

- `tools/perceptive_dev_v2/results/20260507_220657_..._robust_M2_first/` (ON)
- `tools/perceptive_dev_v2/results/20260508_022845_..._M2_AB_baseline_off/` (OFF)

`task.info: robustPhase.enabled` was restored to `true` in the same commit so the A/B
toggle was a one-trial flip only.

### C.3 What's next — Track ②

Per Appendix A, M1''/M2 are both Track ① (event handling off). The original ① → ②
plan in `chat1.md` queues Track ② (early/late contact event handling) immediately after
Track ① is verified. With the cleanup and the body-descent attribution out of the way,
Track ② is now next.

Two-step plan inside Track ②:

- **(a) Detection-only first.** `CtrlComponent::updateState` already has both
  `observation_.mode = estimator_->getMode()` (measured contact, sensor-derived) and
  the `referenceManagerPtr_->getContactFlags(observation_.time)` query (scheduled
  contact). Compare per-leg, log unscheduled-contact-during-robust-window events
  (`[robust_event] leg=… type=early|late t=… t_b=…`) **without** changing the
  schedule or the WBC. Goal: confirm the events happen in the expected places and
  that the detector doesn't fire spuriously on flat ground.
- **(b) Schedule splice + WBC override.** On detected early contact, build a one-leg
  modification of the active schedule that latches that leg to stance from
  `observation_.time` and call `gait_schedule_ptr_->setModeSchedule(...)`. Mirror
  the detection on late contact (scheduled stance with no measured contact). Override
  the WBC contact flag for the current tick so the leg isn't asked for swing torque.

(a) and (b) split into separate commits keeps the "where did the new behavior come
from" debugging story clean.

## Appendix D — Track ② step (a) detection-only — commit `f86da92`

`+83 / 0` in `CtrlComponent.{h,cpp}`. New private method `detectAndLogContactEvents()`
runs once per control tick from `updateState()` right after `observation_.mode` is
filled by the estimator. Compares per-leg measured contact (sensor-derived) against
the scheduled contact flags from the gait schedule and emits one `[robust_event]` log
line per (leg, type) per swing cycle on mismatches. **No schedule mutation, no WBC
contact-flag override** — those land in step (b).

### Conversion direction (peer-review correction)

`estimator_->getMode()` returns a `size_t` mode number — it is itself
`stanceLeg2ModeNumber(contact_flag_)` per [`StateEstimateBase.h:37`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/estimator/StateEstimateBase.h#L37).
For the per-leg comparison we want we therefore need the **inverse**:

```cpp
const contact_flag_t measured = modeNumber2StanceLeg(observation_.mode);
```

(The earlier draft of step (a) had `stanceLeg2ModeNumber` here, which is the wrong
direction; flagged in peer review and corrected before commit.)

### Two event types

- **early** — `refMgr.isInRobustWindow(leg, t) && measured && !scheduled`
  (foot landed before the scheduled `t_b`, while still inside the robust window)
- **late**  — schedule swing→stance rising edge for the leg, but measured contact
  still `false` at that tick (scheduled stance starts with no actual touchdown)

### Latching

- **early latch** `early_event_logged_in_swing_[leg]` resets on **liftoff**
  (stance→swing edge in the schedule) so each swing fires at most one early line.
  A `t_b`-keyed latch would re-fire every MPC cycle because the predicted touchdown
  time drifts a few ms each `preSolverRun`. Empirically a `t_b`-keyed latch produced
  one log line per MPC cycle for the same physical event; the swing-cycle latch
  collapses that to one log line per swing.
- **late** is inherently a rising-edge detector on `scheduled` so it doesn't need a
  latch.

### Verification

Two trials, post-commit `f86da92`:

| trial | scene | scenario | success | dist [m] | pitch_rms [°] | early | late |
|---|---|---|---|---|---|---|---|
| `track2_step_a_swinglatch` | `basic_step_short` | `standing_trot_forward` | True | 1.16 | 5.39 | **264** | **6** |
| `track2_step_a_flat`       | `scene` (flat)     | `standing_trot_forward` | True | 0.97 | 1.49 | **270** | **0** |

Per-leg adjacent-event spacing (leg 1, basic_step_short): ~0.60 s, matching the
trot gait period — confirms the latch fires exactly **once per swing per leg**,
which is the design intent.

### Important interpretation

The flat-scene trial fires ~270 early events too. This is **not** anomalous
early contact from terrain; it's the M2 robust-phase soft penalty doing its
job. With `g(t_b) = −d` enforced softly and `foot_frame_offset = 0.06 m`, the
contact point ends up roughly `d = 0.03 m` below the nominal touchdown
plane (M2 trial 4 measured contact-point residual ≈ 1.6 cm below ground at
`t_b`), so the foot touches `~10–15 ms` earlier than the scheduled `t_b`.
Every healthy swing therefore registers as an early-contact event.

**Implication for step (b)**: a naïve "any early-contact event triggers schedule
splice" rule would re-splice on every swing on flat ground — which would defeat
the gait scheduler entirely. Step (b) needs a threshold to distinguish "designed"
early contact (small lead, ~10 ms, repeating every swing) from anomalous early
contact (large lead because actual terrain came up sooner than planned). Two
sensible thresholds:

1. **Lead-time threshold.** Splice only when `t_b − t_event > τ_lead` for some
   `τ_lead` larger than the design-driven lead (e.g., `τ_lead = 30 ms`).
2. **Sustained-contact threshold.** Splice only when measured contact persists
   for `N` consecutive ticks at the same robust-window event. The design-driven
   case lasts only a few ticks because scheduled stance arrives soon after; an
   anomalous early contact (hit higher terrain) would persist much longer.

(2) is more robust because it doesn't need calibration of `τ_lead` per scene
and naturally adapts to gait period changes.

### Files

| Path | Change |
|---|---|
| [`include/.../control/CtrlComponent.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/control/CtrlComponent.h) | New private `detectAndLogContactEvents()` declaration; `prev_scheduled_contact_` and `early_event_logged_in_swing_` `feet_array_t<bool>` state members. |
| [`src/control/CtrlComponent.cpp`](../controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp) | Method implementation + call site in `updateState()` after `observation_.mode = estimator_->getMode()`. |

### Trial artifacts

- `tools/perceptive_dev_v2/results/20260508_161108_..._track2_step_a_swinglatch/`
- `tools/perceptive_dev_v2/results/20260508_161346_..._track2_step_a_flat/`

### What's next — step (b)

Implement schedule splice + WBC contact-flag override on detected anomalous
events, gated by the sustained-contact threshold (2) above. Touch points:

1. `CtrlComponent` keeps a per-leg `consecutive_unscheduled_contact_ticks_` counter,
   incremented while the early-contact condition holds and reset on schedule
   transition. When it crosses a threshold (e.g., `N = 3` ticks), declare an
   actionable event.
2. On actionable event, build a one-leg modification of the active mode schedule
   that latches the leg to stance from `observation_.time` and call
   `gait_schedule_ptr_->setModeSchedule(...)`. Mirror for late events.
3. WBC override: pass the measured contact flag (instead of the scheduled one)
   to the WBC for the affected leg until the next preSolverRun assimilates the
   spliced schedule.

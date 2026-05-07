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

### Body descent quality (suspected base-ref / WBC issue; needs A/B confirmation)

The body z stayed near box1's stance height (`min_z = 0.317`, `start_z = 0.317`,
`end_z = 0.463`) — the robot does not gracefully transfer body weight onto box2 even
though the swing legs reach the box2 surface. `pitch_rms` ≈ 5.6° (vs 1.47° on flat in
M1'' trial 4) reflects the body straddling the step.

The most likely root cause is base-reference + WBC tracking, *not* the robust phase
itself, but this attribution is provisional and needs an A/B trial to confirm:

- The existing perceptive base-reference modification
  ([`PerceptiveLeggedReferenceManager.cpp:188-294`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp#L188-L294))
  already governs body z descent based on per-leg supports, separately from the swing
  guard.
- The same WBC under-tracking pattern is documented in `plan.md` Context #6
  (splice-point-style discontinuity between OCP plan and WBC tracking under perception
  uncertainty); prior `--terrain basic_step_short` trials *before* the in-OCP robust
  phase existed showed similar body-z behaviour.
- BUT — the robust foot target alters contact timing (`±d` band, `ġ ≤ 0` enforcement),
  which can perturb stance reaction forces and indirectly degrade WBC tracking. So
  saying "M2 robust phase had no effect on the body descent gap" overstates what we
  measured. A direct comparison with `robustPhase.enabled = false` on the same scene
  / scenario is the only clean way to separate the two contributions, and that A/B
  has not been run yet.

What is solid is that **M2's plumbing contribution — the OCP now plans a foot
trajectory that lands on box2 — is a necessary precondition for any subsequent body
descent fix**, regardless of what is currently dragging the body down.

So M2 verification is: **per-leg terrain projection plumbing works end-to-end; the
boundary equality is now defined relative to the (horizontal) box/step surface the leg
targets, and the perception pipeline correctly identifies the descent target one swing
before touchdown**. The "actual surface" claim is still scoped to horizontal box/step
terrain because the plane normal `n` is currently hard-wired to `e_z`; inclined surfaces
need M2.x (extract `n` from `proj.regionPtr->transformPlaneToWorld.linear().col(2)`).
Body descent quality looks primarily like a base-reference / WBC-tracking issue, but
attributing it definitively requires a head-to-head comparison against a no-robust M2
baseline (`task.info: robustPhase.enabled = false` on the same scene), which has not
been run yet.

## Files (read-or-modify map)

Single-commit change set: `9509863`. Three small edits, no new files.

| Path | Change |
|---|---|
| [`include/.../perceptive/interface/PerceptiveLeggedReferenceManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h) | Added `std::string terrain_source` to `RobustPhaseSettings` (`"flat"` or `"convex_region"`). |
| [`src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp) | `computeRobustWindows` adds the `convex_region` branch (phase-index projection lookup, null-projection flat fallback). `loadRobustPhaseSettings` parses the new field. |
| [`descriptions/unitree/go2_description/config/ocs2/task.info`](../descriptions/unitree/go2_description/config/ocs2/task.info) | Default `terrain_source = convex_region`. `terrain_z_M1` retained as fallback. |

## Next milestones — choose one

After M2 the in-OCP robust phase plumbing is complete enough that the Python `robust_refine`
external layer is no longer load-bearing. The remaining work splits into four candidates,
with rough cost / impact estimates:

| # | candidate | scope | est. cost | impact / why pick this next |
|---|---|---|---|---|
| 1 | **Stage 8/9 cleanup** | Delete `RefinedPolicyReader`, `MpcDumpRecorder`, the `mpc_one_shot_*` machinery, the `[robust_refine] override=…` per-cycle log lines; trim `tools/perceptive_dev_v2` of `--enable-refiner-swap` / oneshot scenarios | ~0.5 day | Removes ongoing debug noise (the log lines still print every cycle), shrinks build, eliminates a dead failure-mode path before any hardware bring-up. **Prereq for clean M3 telemetry**. |
| 2 | **Body descent tuning** | Address the body-z gap exposed by M2 (`min_z = start_z`, `pitch_rms 5.6°` on `basic_step_short`). Touch points: `PerceptiveLeggedReferenceManager.cpp:188-294` base-reference modification (height-blend / pitch-blend / down-step commit distance) and/or WBC tracking weights | ~1–2 days, larger result variance | Most visible quality improvement. M2 already plans box2-targeting feet, so this is the bottleneck preventing the whole "graceful descent" demo. |
| 3 | **M3 — multi-touchdown + optional impulse cost** | Generalize `computeRobustWindows` to record up to `M_horizon` upcoming touchdowns per leg (not just the first); optionally add the `λ = (J M⁻¹ Jᵀ)⁻¹ J v⁻` pair-impulse cost from the original `robust_refine` formulation, gated on a CppAD differentiability smoke test for `crba`+inverse | ~2–3 days | Closes the gap with the original `robust_refine` capability set; required for any per-step `d_ℓ` perception-confidence work later. |
| 4 | **M2.x — terrain-normal extraction** | Extract `n` from `proj.regionPtr->transformPlaneToWorld.linear().col(2)` so inclined surfaces (slopes, ramps) use a proper terrain-normal guard rather than world `e_z` | ~0.5 day | Only matters when we actually test on inclined terrain; cheap to do but no immediate sim payoff on box scenes. |

**Recommended order** (driven by "what unblocks the next thing" and "what gives a visible result soon"):

1. **#1 Stage 8/9 cleanup** — small, removes noise, prereq for clean further telemetry.
2. **#2 Body descent tuning** — biggest visible quality win on `basic_step_short`; once done the whole "perceptive descent demo" actually works end-to-end.
3. **#3 M3** — once descent is robust, add multi-touchdown so the robust phase covers more than the first event in horizon.
4. **#4 M2.x** — defer until we actually have an inclined-terrain scenario to validate against.

Track ② (unscheduled-contact mode-switch in `CtrlComponent::updateState`) is orthogonal
and remains parked per `plan.md`.

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
| 4 | medium | "not a robust-phase issue" was definitive without an A/B baseline comparison. | "Body descent quality" subsection retitled "(suspected base-ref / WBC issue; needs A/B confirmation)". Reads "the most likely root cause is base-reference + WBC tracking, *not* the robust phase itself, but this attribution is provisional and needs an A/B trial (`robustPhase.enabled = false` on the same scene/scenario) to confirm". The "M2 plumbing as necessary precondition" point is preserved. |

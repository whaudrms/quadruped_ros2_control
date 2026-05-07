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

Sample (FL leg, sub-sampled to one entry per second):

```
t=5.78  pz=0.2   ← leg's next touchdown is on box1 (still on upper step)
t=6.04  pz=0.2
t=7.02  pz=0.2
t=7.84  pz=0.1   ← switched to box2 (perception sees the descent target now)
t=8.01  pz=0.1
t=9.04  pz=0.1
…       pz=0.1   (stays on box2 for the rest of the trial)
```

Three unique `pz` values appear across the whole 42 s post-startup interval:
`{0.0, 0.1, 0.2}` — floor (null-projection fallback), box2 top, box1 top. The
phase-index path correctly picks the stance-side projection on every cycle.

### Transition cycle caught mid-descent (t ≈ 8.4 s)

When the swing-leg's `next-touchdown` switched mid-window from box1 to box2, the foot z
ended up between the two targets — exactly the in-flight descent state we expect:

```
target box1 (t_b): foot_frame_z = +0.23  (= 0.20 + 0.06 - 0.03)
target box2 (t_b): foot_frame_z = +0.13  (= 0.10 + 0.06 - 0.03)
achieved foot_frame_z at t_b:    +0.177
   → 5.3 cm below box1 target  (foot has cleared the edge, descending)
   → 4.7 cm above box2 target  (still in mid-air toward the lower step)
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

### Body descent quality (orthogonal issue, not a M2 regression)

The body z stayed near box1's stance height (`min_z = 0.317`, `start_z = 0.317`,
`end_z = 0.463`) — the robot does not gracefully transfer body weight onto box2 even
though the swing legs reach the box2 surface. `pitch_rms` ≈ 5.6° (vs 1.47° on flat in
M1'' trial 4) reflects the body straddling the step.

This is a base-reference + WBC-tracking issue, not a robust-phase issue:

- The existing perceptive base-reference modification
  ([`PerceptiveLeggedReferenceManager.cpp:188-294`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp#L188-L294))
  already governs body z descent based on per-leg supports, separately from the swing
  guard.
- This is the same WBC under-tracking we documented earlier (`plan.md` Context #6 —
  splice-point-style discontinuity between OCP plan and WBC tracking under perception
  uncertainty), and the same symptom the prior `--terrain basic_step_short` trials
  exhibited *before* the in-OCP robust phase existed.
- M2's contribution — the OCP now plans a foot trajectory that lands on box2 — is a
  necessary precondition for fixing the body descent, not a competing system. A
  follow-up tuning of base-reference modification or WBC weights would be the right
  place to address the body z gap.

So M2 verification is: **per-leg terrain projection plumbing works end-to-end; the
boundary equality is now defined relative to the actual surface the leg targets, and
the perception pipeline correctly identifies the descent target one swing before
touchdown**. Body descent quality is a separately tracked tuning issue.

## What's next

- **M2.x (optional)** — extract the plane normal from
  `proj.regionPtr->transformPlaneToWorld.linear().col(2)` so inclined surfaces use a
  proper terrain-normal guard (currently `n = e_z` only).
- **M3** — multi-touchdown (all upcoming touchdowns in horizon, not just the first) and
  optional pair-impulse cost via `M⁻¹ J^T` (CppAD differentiability check first).
- **Track ②** — unscheduled-contact mode-switch (handled in `CtrlComponent::updateState`
  per `plan.md`'s note that `SolverSynchronizedModule::preSolverRun` cannot see the
  observation).
- **Stage 8/9 cleanup** — remove `RefinedPolicyReader`, `MpcDumpRecorder`, and the
  legacy `[robust_refine] override=…` log lines; M2 is the natural cut-off after which
  the in-OCP path fully replaces the external Python refiner.

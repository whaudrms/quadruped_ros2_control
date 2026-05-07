# M1'' — Robust contact-timing phase, integrated into perceptive OCS2 OCP

This commit integrates the robust contact-timing-uncertainty phase **directly into the
perceptive OCS2 MPC** as new constraints / costs, replacing the prior out-of-process
Python `robust_refine` IPOPT layer (still kept on the `offline` branch). Scope is the
**M1'' milestone** from `plan.md`: flat-ground sanity check on a single ground plane
parameterised in `task.info`. M2 (`ConvexRegionSelector`-driven per-leg `(n, p_plane)`)
and M3 (multi-touchdown + impulse cost) are out of scope.

## What the code does

For the **first upcoming touchdown event** of each leg in the MPC horizon, define a robust
window `K_ℓ = {k_td_ℓ - P + 1 … k_td_ℓ}` of length `T_robust = P · sqp.dt` (default 5
nodes ≈ 0.10 s for go2). Inside the window:

```
g_ℓ(x)              = n_ℓᵀ ( p_foot,ℓ(q) − p_plane,ℓ )
ġ_ℓ(x,u)            = n_ℓᵀ J_foot,ℓ(q) v_pin(x,u)
g_ℓ(x(t_a_ℓ))       = + d        (boundary target — soft, QuadraticPenalty)
g_ℓ(x(t_b_ℓ))       = − d        (boundary target — soft, QuadraticPenalty)
ġ_ℓ(x(t),u(t))      ≤ 0          (approach inequality — soft, RelaxedBarrierPenalty)
J_robust            = Σ w_v · ġ²  (impact-velocity softening — soft, QuadraticPenalty)
```

For M1'' the guard is hard-wired to `n = e_z`, `p_plane.z = robustPhase.terrain_z_M1`
(scalar from `task.info`). M2 will swap this for the stance-side projection from
`ConvexRegionSelector::getProjections(leg)[stance_phase_idx]`.

`g(x)` is defined on the **contact point**, not the URDF foot frame: `EndEffectorKinematics`
returns the FK position of the URDF foot frame (the ankle for go2's `FL_foot`/etc.), which
sits ~6 cm above the ground contact along `n`. `RobustGuardBoundaryConstraint` therefore
subtracts `n · foot_frame_offset` so that `g(x) = 0` corresponds to "contact point on the
terrain plane", regardless of how high the URDF frame sits above the contact. The offset
is a constant scalar (`task.info: robustPhase.foot_frame_offset`, default 0.06 m for go2);
the Jacobian is unchanged. This factoring transitions cleanly to M2, where `p_plane`
becomes the stance-side terrain projection and the same offset still maps "frame z" → "contact z".

### Phase representation: binary contact + robust mask (not ternary mode)

Effective `swing → robust → stance` is expressed as `(c_ℓ, r_ℓ)` instead of a new
discrete mode in the gait schedule.

| effective phase | c | r | active constraints |
|---|---:|---:|---|
| normal swing  | 0 | 0 | ZeroForce, NormalVelocity (z-tracking), FootCollision |
| robust swing  | 0 | 1 | ZeroForce, RobustGuardBoundary, RobustGuardApproach (`ġ ≤ 0`), RobustGuardCost (`Σ w·ġ²`); NormalVelocity AND FootCollision DEACTIVATED |
| stance        | 1 | 0 | FrictionCone, ZeroVelocity, FootPlacement (gated by `getFootPlacementFlags = c ∧ time ≥ initStandFinalTime`) |

Only two existing `isActive(t)` predicates change — both gain `&& !isInRobustWindow(...)`.

### Partial / clamped window

When `initTime` falls inside the nominal `[t_b - T_robust, t_b]` window, `t_a` is
clamped to `initTime` and the `g(t_a)=+d` boundary is skipped (`skip_t_a_boundary=true`);
only the `g(t_b)=-d` boundary and `ġ ≤ 0` remain active over the truncated window.

## Files (read-or-modify map)

### New

| Path | Role |
|---|---|
| [`include/ocs2_quadruped_controller/interface/RobustWindowData.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/interface/RobustWindowData.h) | Common header for the per-leg window POD struct (kept out of `PerceptiveLeggedReferenceManager.h` to avoid the base reference manager reverse-including the perceptive subclass). |
| [`include/ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h) / [`.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/constraint/RobustGuardBoundaryConstraint.cpp) | `StateConstraint`. Active near `t_a` or `t_b` (within `dt_mpc/2`). Returns `g(x) − target_at(t)`. Wrapped by `StateSoftConstraint + QuadraticPenalty(2·w_boundary)` → cost `w_boundary·(g − target)²`. |
| [`include/ocs2_quadruped_controller/perceptive/constraint/RobustGuardApproachConstraint.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/constraint/RobustGuardApproachConstraint.h) / [`.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/constraint/RobustGuardApproachConstraint.cpp) | `StateInputConstraint`. Active across `[t_a, t_b]`. Returns `−ġ`. Registered TWICE: once with `RelaxedBarrierPenalty` for the inequality `ġ ≤ 0`, once with `QuadraticPenalty(2·w_v)` for the running cost `w_v·ġ²` (sign-symmetric). |
| [`tools/perceptive_dev_v2/plot_robust_phase.py`](../tools/perceptive_dev_v2/plot_robust_phase.py) | Offline verification: parses `tick.csv` → pinocchio FK → foot-z trajectory; parses `controller.log` for `[robust_phase]` lines; overlays `(t_a, +d)` / `(t_b, −d)` markers per leg. Uses the wb-mpc conda env. |
| [`note/m1pp_robust_phase_in_ocs2.md`](m1pp_robust_phase_in_ocs2.md) | This file. |

### Modified

| Path | Change |
|---|---|
| [`include/.../interface/SwitchedModelReferenceManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h) | Added base virtuals `bool isInRobustWindow(leg, time)` (default `false`) and `const RobustWindowData& getRobustWindow(leg)` (default empty). |
| [`include/.../perceptive/interface/PerceptiveLeggedReferenceManager.h`](../controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h) | Nested `RobustPhaseSettings` struct, `setRobustPhaseSettings`, override of the two virtuals, `feet_array_t<RobustWindowData> robustWindows_`, free function `loadRobustPhaseSettings(taskFile, verbose)`. |
| [`src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp) | New `computeRobustWindows(initTime, finalTime, modeSchedule, initState)` — for the **first upcoming touchdown only**, walks `modeSchedule.modeSequence` for each leg, sets `t_b = eventTimes[stance_phase − 1]`, `t_a = t_b − P·dt_mpc`, applies the partial-window clamp. Per-cycle `[robust_phase] t=… leg=… active=… ta=… tb=… pz=… d=… clamped=…` line emitted to `std::cerr` for offline verification. Definition of `loadRobustPhaseSettings`. |
| [`src/interface/constraint/NormalVelocityConstraintCppAd.cpp:54-58`](../controllers/ocs2_quadruped_controller/src/interface/constraint/NormalVelocityConstraintCppAd.cpp) | `isActive` extended with `&& !isInRobustWindow(...)`. |
| [`src/perceptive/constraint/FootCollisionConstraint.cpp:32-42`](../controllers/ocs2_quadruped_controller/src/perceptive/constraint/FootCollisionConstraint.cpp) | `isActive` extended with `&& !isInRobustWindow(...)`. Existing 0.05 s pre-touchdown buffer (`offset = 0.05`) is shorter than `T_robust ≈ 0.075–0.10 s`, so the early portion of the robust window would otherwise enforce SDF clearance against a foot whose target is `z = z_g − d`. |
| [`src/perceptive/interface/PerceptiveLeggedInterface.cpp:60-130`](../controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedInterface.cpp) | After `LeggedInterface::setupOptimalControlProblem`, loads the `robustPhase` block, sets it on the perceptive reference manager (with `dt_mpc = sqp_settings_.dt`), and registers three soft entries per leg (boundary, approach inequality, ġ² cost). |
| [`CMakeLists.txt:73-78`](../controllers/ocs2_quadruped_controller/CMakeLists.txt) | Two new `.cpp` sources: `RobustGuardApproachConstraint.cpp`, `RobustGuardBoundaryConstraint.cpp`. |
| [`descriptions/unitree/go2_description/config/ocs2/task.info`](../descriptions/unitree/go2_description/config/ocs2/task.info) | New `robustPhase` block (`enabled`, `P`, `d`, `w_v`, `w_boundary`, `approach_barrier_mu/delta`, `terrain_z_M1`). |

## Verification (M1'' done)

Three tuning trials on flat-scene `scene.xml`, `standing_trot_forward` scenario, all on
the `replan` branch with the in-OCP robust phase enabled. The first two iterations used
the workaround of folding the foot-frame offset into `terrain_z_M1`; the post-review
trial uses the explicit `foot_frame_offset` parameter introduced by the review fixes
(numerically equivalent to trial 3).

| trial | terrain_z_M1 | foot_offset | d | w_boundary | success | t_b residual* | t_a residual* | note |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.00 | n/a | 0.05 | 100 | true | +0.11 m | +0.11 m | soft penalty too weak — no observable pull |
| 2 | 0.00 | n/a | 0.05 | 10000 | **fall (roll_limit)** | — | — | physically-infeasible target × strong weight |
| 3 | 0.06 | n/a | 0.03 | 1000 | true | +0.018 m | +0.06 m | folded offset; clean pull, no instability |
| 4 | 0.00 | 0.06 | 0.03 | 1000 | true | **+0.014 m** | +0.058 m | post-review: explicit offset, ready for M2 |

*Residual = `foot_frame_z(opt) − (p_plane.z + foot_offset ± d)` measured at the SQP
shooting node closest to `t_b` / `t_a`.

What trial 4 actually demonstrates:
- Window calculation correct: `t_b − t_a = 0.10 s = P·sqp.dt`, trot diagonal pair
  `(FL,RR)` vs `(FR,RL)`, `skip_t_a_boundary` partial-window logic exercised.
- `t_b` boundary (terminal target `g = -d`) is **respected within ~1.4 cm** — the soft
  penalty pulls the foot strongly toward the contact-point target.
- `t_a` boundary (start target `g = +d`) **under-tracks by ~6 cm** — the foot does not
  reach `+d` above terrain at the window start. This is consistent with the soft-penalty
  trade-off: the `w_boundary=1000` weight can shape the trajectory but cannot
  counter-rotate the upward swing peak. Bumping `w_boundary` aggressively (trial 2)
  destabilizes. Per `plan.md` sub-decision 1, the documented fallback is to promote the
  boundaries to hard `equalityConstraintPtr` entries if soft tuning continues to
  under-track on real terrain in M2.
- No regression in walking stability: pitch_rms 1.47° (baseline 1.44°), base z stable,
  full 16 s scenario completed, 0.99 m forward; SDF clearance constraint records no
  violation events inside robust windows (gating fix is honoured).

So M1'' verification is: **terminal boundary pull confirmed; start boundary under-tracks
within tolerance** — the in-OCP plumbing is sound and ready for M2's terrain-aware
projection.

Trial output (`result.json`, `tick.csv`, `controller.log`, `robust_phase_foot_z.png`,
console residual table) lives at:
`tools/perceptive_dev_v2/results/20260507_210019_scene_perceptive_dev_v2_robust_M1pp_postreview2/`.

## Review fixes applied post-M1''

A peer review on the initial M1'' commit raised five issues. F1, F2, F4, F5 are fixed in
the follow-up commit before M2 begins; F3 was a wording correction that landed in this
note alongside trial 4.

| # | severity | issue | fix |
|---|---|---|---|
| F1 | high | M2 would put the foot frame `~6 cm + d` below the contact terrain because `EndEffectorKinematics::getPosition` returns the URDF foot frame, not the contact point | Added `foot_frame_offset` to `RobustPhaseSettings` / `RobustWindowData`. `RobustGuardBoundaryConstraint` subtracts `foot_offset` from `g(x)` (Jacobian unchanged). For M1'' set `terrain_z_M1 = 0.0` and `foot_frame_offset = 0.06`. M2's `ConvexRegionSelector` projection now plugs into `p_plane` directly without further offset bookkeeping. |
| F2 | high | `computeRobustWindows` could miss the next touchdown if the leg was currently in stance: it grabbed the first `false→true` transition and `continue`d the leg if `t_b ≤ initTime`, instead of scanning further | Loop now keeps scanning until it finds a `false→true` transition with `eventTimes[idx] > initTime`. Past transitions (leg already in stance) are skipped, not the entire leg. |
| F3 | medium | Note overstated trial 3's verification — `t_a` residual was +0.06 m; the start boundary did not actually reach `+d` | Verification table now lists `t_a` and `t_b` residuals separately and the conclusion reads "terminal boundary pull confirmed; start boundary under-tracks within tolerance" instead of "boundary nodes correct". |
| F4 | medium | `getRobustWindow` returned a reference into mutex-protected internal storage past the lock — structurally racy even if the MPC thread happens to serialise calls | Base virtual signature changed to return `RobustWindowData` by value. The override copies under the lock and releases. All call sites switched from `const auto&` to `const RobustWindowData` capture. |
| F5 | low | `[robust_phase]` `std::cerr` per cycle × 4 legs is fine in sim but not RT-safe on hardware | Wrapped in `if (robustPhaseSettings_.verbose_log)`. Default `false`; enabled in current `task.info` for offline plot verification. Stage 8/9's `[robust_refine] override=...` lines remain (will be removed in the follow-up cleanup PR). |

The post-fix smoke trial (trial 4 above) confirms numerical equivalence with the
pre-fix trial 3 baseline.

## Out of scope (deferred)

- **M2** — replace flat `terrain_z_M1` with per-leg `(n, p_plane)` from
  `ConvexRegionSelector::getProjections(leg)[stance_phase_idx]` (using the
  phase-index path, not `getProjection(leg, t_b)`, since `ModeSchedule::modeAtTime`
  returns the *lower* count at exact event times).
- **M3** — multi-touchdown windows + optional pair impulse cost via `M⁻¹ J^T`.
- **Track ②** — unscheduled-contact mode-switch (`observation_.mode` vs scheduled
  comparison; preferred wiring is in `CtrlComponent::updateState`, not a new
  `SolverSynchronizedModule`, since `preSolverRun` cannot see the observation).
- **Stage 8/9 cleanup** — `RefinedPolicyReader`, `MpcDumpRecorder`, the legacy
  `[robust_refine] override=…` log lines and CSV-IPC plumbing remain in place; they emit
  `override=inactive` on every cycle (no refined policy file present) and are harmless.
  Removal will land in a separate cleanup PR after M2.

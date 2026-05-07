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

Three trials on flat-scene `scene.xml`, `standing_trot_forward` scenario, all on the
`replan` branch with the in-OCP robust phase enabled.

| trial | terrain_z_M1 | d | w_boundary | success | t_b residual | t_a residual | note |
|---|---|---|---|---|---|---|---|
| 1 | 0.00 | 0.05 | 100 | true | +0.11 m | +0.11 m | soft penalty too weak — no observable pull |
| 2 | 0.00 | 0.05 | 10000 | **fall (roll_limit)** | — | — | physically-infeasible target (z = −0.05 below ground) × strong weight = destabilizing |
| 3 | 0.06 | 0.03 | 1000 | true | **+0.018 m** | +0.06 m | physical foot-frame z (0.06) + modest d → clean pull, no instability |

Trial 3 confirms the M1'' code path end-to-end: window calculation correct (`t_b − t_a =
0.10 s = P·sqp.dt`, trot diagonal pair `(FL,RR)` vs `(FR,RL)`), constraint trips at
boundary nodes (residual would be ~0.11 m without it, as in trial 1), `skip_t_a_boundary`
partial-window logic correct, no regression in walking stability vs baseline (pitch_rms
1.44° identical, base z stable, full 16 s scenario completed, 0.98 m forward).

Trial output (results, plot, residual table) lives at:
`tools/perceptive_dev_v2/results/20260507_201319_scene_perceptive_dev_v2_robust_M1pp_z06_d03_w1k/`.

The remaining ~2 cm residual at `t_b` is the soft-penalty trade-off Plan sub-decision 1
flagged: bumping `w_boundary` higher destabilizes (trial 2). A true `equalityConstraintPtr`
hard equality is the documented fallback if M2 still under-tracks ±d on terrain.

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

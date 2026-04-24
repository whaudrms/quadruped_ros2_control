# Perceptive OCS2 요약 — MPC에 지형을 태우는 구조

`mpc_wbc_summary.md` 가 평지 baseline MPC↔WBC 협업을 정리한 것이라면, 이 문서는 그 위에 **지형 인식 계층이 어떻게 얹히는지** 한 장으로 정리. 세부는 `perceptive*.md`에 흩어져 있다.

## 1. WBC는 바뀌지 않는다

perceptive 모드가 켜져도 `WeightedWbc` / `HierarchicalWbc` / `WbcBase` 쪽 코드는 손대지 않는다. WBC는 centroidal policy $(x^*, u^*, m^*)$ 를 받아 그 순간의 관절 토크로 번역하는 역할만 계속함.

**모든 terrain-aware 변화는 MPC 이전 단계(reference/precomputation)와 MPC OCP 정의에 soft constraint를 추가하는 쪽에서 일어난다.**

## 2. 어디에 지형이 들어오는가

### (a) reference 쪽 — base pitch & z 수정

- 기존 `SwitchedModelReferenceManager::modifyReferences` : `terrainHeight = 0` 하드코딩
- perceptive : `PerceptiveLeggedReferenceManager::modifyReferences`
  - 11-node 샘플에서 `smooth_planar` layer로 지형 법선 → terrain pitch 계산
  - `pitchBlend=0.6, heightBlend=0.5`로 완만히 블렌드, `maxAbsPitch=0.25 rad`, node간 변화 clamp
  - `z_base = h_terrain(x,y) + comHeight / cos(pitch)` 적용
  - step-down guard로 급격한 z 하락 억제 (`downStepCommitDistance=0.08 m`)

### (b) reference 쪽 — swing trajectory 높이

- 기존 : `SwingTrajectoryPlanner`는 모든 발에 대해 `terrainHeight = 0`으로 lift/touch 계산
- perceptive :
  - `ConvexRegionSelector::update` 가 각 stance phase의 projection.z 를 계산
  - `PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner` 가 이 z를 `liftOffHeightSequence` / `touchDownHeightSequence`로 swing planner에 주입
  - swing 중 발 높이가 튀지 않도록 **latching** (`activeSwingHeightLatched_`) 적용

### (c) MPC OCP — soft constraint 추가

`PerceptiveLeggedInterface::setupOptimalControlProblem`이 base OCP 구성 후 `problem_ptr_->stateSoftConstraintPtr`에 **세 종류의 RelaxedBarrier-wrapped StateConstraint**를 추가:

| Constraint | 활성 조건 | 수식 |
|---|---|---|
| `FootPlacementConstraint` | stance + 초기 stance 구간 종료 후 | $A_i\mathbf{p}_{ee,i}(\mathbf{x}) + \mathbf{b}_i \ge \mathbf{0}$ (convex polygon) |
| `FootCollisionConstraint` | 현재+0.025s+−0.05s 모두 swing | $d_{\mathrm{SDF}}(\mathbf{p}_{ee,i}) - 0.01 \ge 0$ |
| `SphereSdfConstraint` | 항상 | $d_{\mathrm{SDF}}(\mathbf{p}_{\text{sphere},j}) - r_j \ge 0$ ($j$=4 calf spheres) |

## 3. 데이터 통로

```
MuJoCo XML ──► planar_terrain_publisher (standalone executable)
             ──► /convex_plane_decomposition_ros/planar_terrain (transient_local)
PlanarTerrainReceiver (SolverSynchronizedModule, preSolverRun)
  ── lock, copy → *planarTerrainPtr_
  └─ recompute SDF (max + 3·0.1 m margin) → *sdfPtr_

*planarTerrainPtr_ ─ shared ─► PerceptiveLeggedInterface
                              ├─ ConvexRegionSelector          (lock → terrain snapshot)
                              │   └─ update(modeSchedule, initTime, state, targetTraj)
                              │       per-foot: (projection, convex polygon, nominal foothold)
                              ├─ PerceptiveLeggedPrecomputation (매 MPC tick)
                              │   └─ request → per-foot (A,b) 월드 좌표계 제약
                              └─ (directly) PerceptiveLeggedReferenceManager::modifyReferences
                                  (smooth_planar layer 직접 샘플링)

*sdfPtr_ ─ shared ─► FootCollisionConstraint, SphereSdfConstraint
                     (getValue / getLinearApproximation)
```

## 4. 실행 타이밍

```
[매 ROS tick ~500Hz]                       [MPC 스레드 ~100Hz]
Ocs2QuadrupedController::update             mpc_mrt_interface_->advanceMpc()
  ├─ estimator → rbd → centroidal            ├─ solver 시작 시:
  ├─ (perceptive+LKF) 지형 높이로 z 교정      │     ├─ GaitManager::preSolverRun
  ├─ observation 갱신 → MRT에 주입            │     ├─ PlanarTerrainReceiver::preSolverRun
  ├─ perceptive 시각화/로그 발행              │     │     └─ SDF 재계산
  └─ StateOCS2::run                           │     └─ ReferenceManager::modifyReferences
        ├─ MRT updatePolicy                   │           └─ PerceptiveLeggedReferenceManager
        ├─ MRT evaluatePolicy → x*, u*, m*    │                 ├─ terrain-aware base target
        ├─ WBC updateMeasured/updateDesired   │                 ├─ ConvexRegionSelector::update
        ├─ WBC QP solve                       │                 └─ SwingTrajectoryPlanner::update
        └─ τ, q_des, dq_des → motor           └─ SQP iteration (Precomputation::request per node)
```

MPC가 한 번 돌 때마다 SDF / convex polygon / target pitch 가 전부 최신 terrain 으로 일관되게 정렬되고, SQP는 horizon 상의 각 node에서 같은 (A,b), 같은 SDF로 평가한다.

## 5. 디커플링의 장점

perceptive 계층이 전부 **OCP의 cost/soft-constraint와 reference 만을 통해** 들어가므로:

- dynamics 모델 (centroidal) 은 그대로 → 기존 CppAD 캐시, solver, WBC 유지
- 런타임에 플래그로 개별 ablation 가능 → `enable_perceptive=true`, 나머지 `=false` 로 interface만 바꾼 baseline을 만들 수 있어 실험 비교에 유리
- terrain 소스가 XML 파싱이든 실제 depth camera pipeline이든 같은 `PlanarTerrain` 메시지 형태로 나오면 controller 쪽은 동일하게 동작

## 6. 한 줄 요약

> 평지 centroidal OCP 의 **reference(base pitch·z, swing 높이)를 smooth_planar grid와 ConvexRegionSelector가 계산한 foothold projection 으로 교체**하고, **convex polygon을 월드 좌표의 선형 부등식으로 만들어 foot placement soft constraint** 로, **SDF 거리를 foot·body collision soft constraint** 로 집어넣으면, 그 위의 SQP MPC 와 그 아래 WBC는 바뀌지 않은 채로 지형을 인식하며 걷는다.

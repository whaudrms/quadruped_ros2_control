# 원본 `ocs2_perceptive_anymal` vs 우리 포트 비교

**작성일**: 2026-04-25
**배경**: step-down (박스1 0.30 → 박스2 0.20, 0.10 m drop) 이 우리 포트에서 실패 (QP 발산 → orientation safety trip). 동료는 `lastLiftoffPos_[leg].z() -= 0.08` magic 으로 땜질했는데, 원본엔 이 hack 이 없고 대신 훨씬 큰 (0.3~0.5 m) drop 도 성공. 원본 구조 를 체계적으로 분석해서 구조적 fix 를 찾는 것이 목적.

---

## TL;DR

우리 포트는 **loopshaping + 3D swing trajectory** 를 잃었고, 그 구멍을 reference manager 쪽의 heuristic 과 magic number 로 땜질함. 원본은 이 둘을 기본으로 갖추고 있어서 magic number 없이도 크게 내려가는 지형을 처리함.

---

## 1. 원본 프레임워크 런타임 구조

**Entry point**: `ocs2_anymal_loopshaping_mpc/src/AnymalLoopshapingMpcNode.cpp`

생성 체인:
1. `anymal::getAnymalInterface()` → `QuadrupedInterface` (core MPC 문제 정의)
2. `QuadrupedLoopshapingInterface(QuadrupedInterface, LoopshapingDefinition)` 로 감싸기 — **주파수 영역 필터 삽입**
3. `getSqpMpc(...)` 또는 `getDdpMpc(...)` 로 MPC 인스턴스
4. `quadrupedLoopshapingMpcNode(...)` — ROS2 integration

**Config 계층**:
- `task.info` — MPC cost/constraint weights, swing profile
- `frame_declaration.info` — link 정의
- `loopshaping.info` — 필터 파라미터 (아래 4-2 참조)

---

## 2. 클래스 매핑

| 원본 | 우리 포트 | 핵심 차이 |
|---|---|---|
| `SegmentedPlanesTerrainModel` (+ SDF class) | `PlanarTerrainReceiver` | 원본: abstract `TerrainModel` interface + plug-in. 포트: `PlanarTerrain` 직접 사용. |
| `SwingTrajectoryPlanner` (3D, `foot_planner/`) | `SwingTrajectoryPlanner` (1D) | **원본: xy+z quintic. 포트: z 만.** |
| `FootPhase` (`StancePhase`/`SwingPhase`) | `SplineCpg` (1D) | 원본: polymorphic with 3D swing. 포트: 단일 1D spline. |
| `SwingSpline3d` | 없음 | 3D waypoint 기반 spline, apex · margin 포함. |
| `selectNominalFootholdTerrain` (planner 내부) | `ConvexRegionSelector` (extracted) | 원본: swing 생성 시점에 leg-extension penalty 포함. 포트: 별도 클래스, xy 미반환. |
| `SwitchedModelModeScheduleManager` | `PerceptiveLeggedReferenceManager` | 원본: 단순히 gait + terrain update. 포트: pitchBlend/heightBlend/latching/magic offset 추가. |
| `LoopshapingRobotInterface` | **없음** | 제거됨. |

---

## 3. 원본이 step-down 을 어떻게 푸는가

### 3-1. Swing trajectory 는 3D (xy + z)

`ocs2_switched_model_interface/src/foot_planner/SwingTrajectoryPlanner.cpp:116-142`:
```
liftOff event  = {time, velocity, *currentTerrainPlane}
touchDown event = {time, velocity, *nominalFootholdsPerLeg_[leg].plane}
SwingProfile  = {swingHeight, errorGain, sdfMargin, terrainMargin, ...}
→ SwingPhase(liftOff, touchDown, profile)
   └ SwingSpline3d — x, y, z 3개 quintic spline 동시 생성
```

`SwingSpline3d` (foot_planner/SwingSpline3d.h:19-54):
- 3 개 이상의 `SwingNode3d` (time, position, velocity) 입력
- `position(t) / velocity(t) / acceleration(t)` 모두 3D 반환
- touchdown xy 가 nominal foothold 의 xy 로 자동 설정

**우리 포트는 `SwingTrajectoryPlanner::update(modeSchedule, liftOffHeights, touchDownHeights)` 로 scalar z array 만 받음 → xy 는 reference 에 없음**.

### 3-2. `selectNominalFootholdTerrain` 이 xy 를 똑똑하게 선택

원본은 swing 생성 *전에* 각 contact phase 마다:
1. `selectHeuristicFootholds` (Raibert 유사) 로 후보 xy 계산
2. 후보를 terrain model 의 closest planar region 으로 projection
3. **leg-over-extension penalty** 로 다리 길이 초과 후보 거절
4. 최종 nominal foothold (3D) 를 swing planner 에 전달

우리 포트의 `ConvexRegionSelector` 는 z projection 만 반환 → swing planner 는 xy 를 모름 → MPC 가 cost 로만 발을 맞추려 하다 실패.

### 3-3. SDF-based swing clearance (reference 레이어)

원본 `SwingProfile`:
- `sdfMidswingMargin = 0.04 m`
- `sdfStartEndMargin`
- `terrainMargin = 0.04 m`

→ `SwingPhase::getMinimumFootClearance(t)` 가 swing 경로를 terrain SDF 에 맞춰 warp. Obstacle 회피가 **reference 레벨에서** 보장됨.

우리 포트의 `FootCollisionConstraint` 는 MPC soft constraint 로 reactive 하게 가함 — 제약이 늦게 켜져서 edge 근처에서 sharp gradient 로 QP 가 폭발.

### 3-4. Loopshaping 으로 touchdown impact 흡수

`loopshaping.info`:
- Force filter: pole @ −100 rad/s (10 ms time constant), 12 개 (4 발 × 3 축)
- Joint velocity filter: pole @ −50 rad/s (20 ms)

효과: MPC 가 touchdown 순간 큰 contact force 를 명령해도 실제 출력은 필터를 거쳐 부드럽게 증가. 몸체 pitch 진동 흡수.

---

## 4. 우리 포트가 버리거나 단순화한 요소

| 항목 | 원본 | 포트 | 영향 |
|---|---|---|---|
| Loopshaping | O | **X** | touchdown force spike 미흡수 → tipping |
| 3D swing spline | O | **X (z 만)** | xy foothold adaptation 없음 |
| SwingPhase SDF clearance | O (reference 레벨) | MPC constraint (reactive) | edge 에서 constraint 가 늦게 활성 |
| TerrainModel abstraction | O | X | 유연성 ↓ (기능엔 문제 없음) |
| previousFootholdFactor (기본 0.2) | O (task.info) | pitchBlend=0.6, heightBlend=0.5, hard-coded | 튜닝 안 돌아감 |
| `lastLiftoffPos_.z() -= X` | **없음** | O (magic) | 원본은 구조로 해결, 포트는 heuristic 으로 땜질 |

---

## 5. "제대로" 고치려면 이식해야 할 것들 (priority)

### Tier 1 (core, 반드시 필요)

**1. Loopshaping 복원**
- 원본 패키지: `ocs2_quadruped_loopshaping_interface/`
- 작업: `LoopshapingRobotInterface` wrapper 를 포트에 포함. `loopshaping.info` 복사. `PerceptiveLeggedInterface` 를 wrapping.
- 기대: contact transition smoothing → pitch 진동 감소

**2. 3D swing spline 복원**
- 원본 파일: `ocs2_switched_model_interface/src/foot_planner/SwingSpline3d.h/.cpp`, `FootPhase.h`, `SwingTrajectoryPlanner.cpp`
- 작업:
  - `SwingSpline3d`, `SwingNode3d`, `FootPhase/StancePhase/SwingPhase`, `SwingProfile` 이식
  - `SwingTrajectoryPlanner` 시그니처 변경: `update(modeSchedule, liftOffHeights, touchDownHeights)` → `updateSwingMotions(initTime, finalTime, initState, modeSchedule, nominalFootholds3d)`
  - 이에 맞춰 `PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner`, `PerceptiveLeggedPrecomputation` 도 수정
- 기대: step-down 후 xy 가 다음 region 중앙으로 자동 정렬 → leg stretch 해소

### Tier 2 (robustness)

**3. SwingPhase SDF clearance 이식**
- `SwingProfile::sdfMidswingMargin`, `sdfStartEndMargin` 추가
- `SwingPhase::getMinimumFootClearance(time)` 구현
- `FootCollisionConstraint` 와 병행 또는 대체

**4. `ConvexRegionSelector` 가 xy foothold 반환**
- 현재는 z projection 만. xy + leg-over-extension penalty 까지 계산하도록 확장.

### Tier 3 (cleanup, Tier 1+2 완료 후)

**5. Reference modification hyperparameter 정리**
- `pitchBlend`, `heightBlend`, `downStepCommitDistance` 를 `task.info` 에서 로드
- `previousFootholdFactor` (원본 방식) 로 통일 고려

**6. magic offset 제거**
- `PerceptiveLeggedReferenceManager.cpp:276` 의 `lastLiftoffPos_[leg].z() -= 0.02` 삭제 (혹은 0 으로)
- Tier 1+2 가 동작하면 이 hack 불필요

---

## 6. 작업 추정

| Tier | 작업 | 기간 (rough) |
|---|---|---|
| 1 | Loopshaping + 3D swing | 2~3 일 |
| 2 | SDF clearance + xy foothold | 1~2 일 |
| 3 | config 정리 + hack 제거 | 반나절 |
| **합계** | | **3.5~5.5 일** |

---

## 7. 관련 파일 경로

**원본**:
- `/home/cora/GO2_ws/ocs2_ros2/advance examples/ocs2_perceptive_anymal/`
  - `ocs2_anymal_loopshaping_mpc/src/AnymalLoopshapingMpcNode.cpp` (entry point)
  - `ocs2_anymal_loopshaping_mpc/config/c_series/loopshaping.info` (필터 파라미터)
  - `ocs2_anymal_loopshaping_mpc/config/c_series/task.info` (cost/constraint, swing profile)
  - `ocs2_quadruped_loopshaping_interface/` (wrapper 구현)
  - `ocs2_switched_model_interface/src/foot_planner/SwingTrajectoryPlanner.cpp:92-142` (nominal foothold + swing 생성)
  - `ocs2_switched_model_interface/include/ocs2_switched_model_interface/foot_planner/SwingSpline3d.h` (3D spline)
  - `ocs2_switched_model_interface/include/ocs2_switched_model_interface/foot_planner/FootPhase.h` (SwingPhase + SwingProfile)
  - `segmented_planes_terrain_model/` (terrain 표현)

**포트**:
- `/home/cora/GO2_ws/quadruped_ros2_control/controllers/ocs2_quadruped_controller/`
  - `src/perceptive/interface/PerceptiveLeggedInterface.cpp` (MPC 조립)
  - `src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp:275-276` (magic offset)
  - `include/ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h:38-113` (1D swing)
  - `src/perceptive/interface/ConvexRegionSelector.cpp` (z-only projection)
  - `src/perceptive/synchronize/PlanarTerrainReceiver.cpp` (terrain 입력)

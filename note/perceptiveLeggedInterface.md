# PerceptiveLeggedInterface

## Summary

`LeggedInterface`를 상속하여 평지 OCP 위에 지형 인식 관련 자료구조(PlanarTerrain, SDF, sphere kinematics)와 soft constraint(foot placement / foot collision / body sphere SDF)를 얹는 클래스. dev 브랜치에서는 각 제약의 on/off가 `enable*_` 플래그로 분기된다.

## setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose)

### 1. 초기 planar terrain 생성

부팅 직후에는 외부 topic이 아직 없으므로, $5\times5\ \mathrm{m}$의 더미 평면을 만들어 self-contained 초기화.

$$
W = H = 5.0,\quad r_{\mathrm{grid}} = 0.03\ \mathrm{m}
$$

- `PlanarRegion` 1개: `transformPlaneToWorld = I`, `bbox2d = [-H/2, -W/2, +H/2, +W/2]`
- `boundaryWithInset.boundary` : 5 m × 5 m 사각형
- `boundaryWithInset.insets` : 같은 크기에서 각 변 0.01 m 안쪽으로 수축한 사각형 (foot placement 내부 영역)

### 2. grid map과 SDF

`gridMap` 은 `elevation`과 `smooth_planar` 두 layer를 가짐 (모두 0 초기화). SDF는 `elevation` layer 기준으로 첫 계산:

$$
h_{\max}^{\mathrm{SDF}} = \max\{h\} + 3\cdot 0.1,\quad d(\mathbf{p}) = \mathrm{SDF}(\mathrm{gridMap}, \mathrm{elevation}, h_{\max}^{\mathrm{SDF}})
$$

- `planarTerrainPtr_` — `std::shared_ptr<PlanarTerrain>` (외부 topic으로 교체됨)
- `signedDistanceFieldPtr_` — `std::shared_ptr<grid_map::SignedDistanceField>`
- `terrainDataMutex_` — 위 두 포인터와 grid map 데이터 보호용

### 3. Base OCP 구축

```cpp
LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
```
평지 기준 cost/constraint를 먼저 모두 구성한 뒤 perceptive soft constraint를 추가한다.

### 4. per-foot soft constraints

각 발 $i \in \{1..4\}$에 대해 `PinocchioEndEffectorKinematicsCppAd` 하나를 만들고:

**FootPlacement (if `enableFootPlacementConstraint_`)**

$$
J_{i,\text{footPlacement}} = \mathrm{RelaxedBarrier}(h_{i,\text{footPlacement}}(\mathbf{x})),\quad (\mu,\delta) = (10^{-2},\ 10^{-4})
$$

**FootCollision (if `enableFootCollisionConstraint_`)**

$$
J_{i,\text{footCollision}} = \mathrm{RelaxedBarrier}(d(\mathbf{p}_{ee,i}) - 0.01),\quad (\mu,\delta) = (10^{-2},\ 10^{-3})
$$

두 soft constraint 모두 `problem_ptr_->stateSoftConstraintPtr` 에 `<footName>_footPlacement`, `<footName>_footCollision` 이름으로 추가된다.

### 5. body collision (sphere approximation)

```cpp
std::vector<std::string> collisionLinks = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"};
maxExcesses = {0.02, 0.02, 0.02, 0.02};
pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
    *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6);
```

각 calf 링크를 sphere로 근사 (excess=0.02 m, shrink factor 0.6). sphere 위치는 `PinocchioSphereKinematics`가 centroidal state에서 forward kinematics로 얻음.

**SphereSdf (if `enableBodyCollisionConstraint_`)**

$$
J_{\text{sdf}} = \mathrm{RelaxedBarrier}\Big(\sum_j\ d(\mathbf{p}_j) - r_j\Big),\quad (\mu,\delta) = (10^{-3},\ 10^{-3})
$$

`"sdfConstraint"` 이름으로 soft constraint 추가.

## setupReferenceManager(taskFile, urdfFile, referenceFile, verbose)

1. `SwingTrajectoryPlanner` 생성 (`swing_trajectory_config`, 발 4개)
2. 모든 발을 한 번에 다루는 `EndEffectorKinematics`(`"ALL_FOOT"`) 생성
3. `ConvexRegionSelector` 생성 (planarTerrainPtr_, mutex 공유)
4. `referenceFile`에서 `comHeight` 읽기
5. `reference_manager_ptr_ = new PerceptiveLeggedReferenceManager(...)` — SwitchedModelReferenceManager를 대체
6. `setEnableReferenceModification(enableReferenceModification_)` — 플래그 전달

## setupPreComputation

기본 `LeggedRobotPreComputation` 을 `PerceptiveLeggedPrecomputation` 으로 교체. `convexRegionSelector`와 `footPlacementBoundaryMargin_`를 넘김.

## 주요 플래그/세터

`setPerceptiveDebugOptions(enableRefMod, enableFootPlacement, enableFootCollision, enableBodyCollision, boundaryMargin)` — `CtrlComponent::setupLeggedInterface`에서 런치 파라미터를 interface에 주입.

## 외부에 노출되는 포인터

- `getPlanarTerrainPtr()`, `getSignedDistanceFieldPtr()`, `getTerrainDataMutexPtr()` → `PlanarTerrainReceiver`가 사용
- `getPinocchioSphereInterfacePtr()` → `SphereVisualization`이 사용

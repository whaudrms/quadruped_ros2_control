# CtrlComponent — perceptive wiring

## Summary

`CtrlComponent`는 컨트롤러의 컴포넌트 조립소. dev 브랜치에서는 여기에 perceptive 경로 분기가 여러 군데 추가되었다. 기본 흐름(state estimator 생성, LeggedInterface 생성, SqpMpc, MRT, target manager) 외에 perceptive-specific 부분만 이 문서에서 정리.

## 1. 파라미터 선언 / 로드 (ctor + on_configure)

```cpp
node_->declare_parameter("enable_perceptive",                            enable_perceptive_);
node_->declare_parameter("enable_perceptive_reference_modification",     ...);
node_->declare_parameter("enable_perceptive_foot_placement_constraint",  ...);
node_->declare_parameter("enable_perceptive_foot_collision_constraint",  ...);
node_->declare_parameter("enable_perceptive_body_collision_constraint",  ...);
node_->declare_parameter("perceptive_foot_placement_boundary_margin",    ...);
...
loadData::loadCppDataType(reference_file_, "comHeight", perceptive_com_height_);
```

`comHeight`는 원래 `LeggedInterface`/`PerceptiveLeggedReferenceManager` 내부에서만 쓰이지만, **perceptive 모드에서 linear-kalman estimator의 base z를 지형에 붙이기 위해** 여기서도 별도 보관한다.

## 2. setupLeggedInterface — interface 분기

```cpp
if (enable_perceptive_) {
    legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).setPerceptiveDebugOptions(
        enable_perceptive_reference_modification_,
        enable_perceptive_foot_placement_constraint_,
        enable_perceptive_foot_collision_constraint_,
        enable_perceptive_body_collision_constraint_,
        perceptive_foot_placement_boundary_margin_);
} else {
    legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
}
legged_interface_->setupJointNames(joint_names_, feet_names_);
legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);
```

perceptive가 켜지면 **세부 플래그를 interface에 주입한 뒤** OCP를 setup 하기 때문에, `PerceptiveLeggedInterface::setupOptimalControlProblem` 내부에서 `enableFootPlacementConstraint_` 등이 true일 때만 실제 soft constraint가 problem에 add 된다.

### 추가 객체 생성 (perceptive only)

```cpp
footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
    *refManager.getConvexRegionSelectorPtr(), numFeet, node_);
sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
    pinocchio_interface, centroidal_model_info, *pinocchioSphereInterfacePtr, node_);
rawReferencePathPublisherPtr_ =
    node_->create_publisher<nav_msgs::msg::Path>("/perceptive_reference/raw_base_path", 1);
terrainAwareReferencePathPublisherPtr_ =
    node_->create_publisher<nav_msgs::msg::Path>("/perceptive_reference/terrain_aware_base_path", 1);
```

- `FootPlacementVisualization` — RViz MarkerArray로 각 발의 nominal foothold, projection, convex polygon을 발행.
- `SphereVisualization` — body collision sphere 들을 MarkerArray로.
- 두 `Path` publisher — `modifyReferences`에서 쌓아둔 `latestRawBasePath_` / `latestTerrainAwareBasePath_` (`modifyReferences` 11-node 샘플)를 사용해 비교 시각화용.

## 3. setupMpc — SynchronizedModule로 receiver 등록

```cpp
gait_manager_ptr_ → mpc_->solver->addSynchronizedModule
target_manager_ = TargetManager(...);

if (enable_perceptive_) {
    auto receiver = std::make_shared<PlanarTerrainReceiver>(
        node_,
        getPlanarTerrainPtr(),
        getSignedDistanceFieldPtr(),
        getTerrainDataMutexPtr(),
        "/convex_plane_decomposition_ros/planar_terrain", "elevation");
    mpc_->getSolverPtr()->addSynchronizedModule(receiver);
}
```

이 덕분에 solver가 tick 시작마다 `receiver->preSolverRun(...)`을 호출 → 최신 terrain 적용 → `ConvexRegionSelector`가 보는 snapshot과 `SphereSdfConstraint`가 보는 SDF가 일치한다.

## 4. updateState — 매 controller tick

```cpp
measured_rbd_state_ = estimator_->update(time, period);
if (enable_perceptive_ && estimator_type_ == "linear_kalman")
    alignPerceptiveBaseHeightToTerrain();
...
visualizer_->update(observation_);
if (enable_perceptive_) {
    footPlacementVisualizationPtr_->update(observation_);
    sphereVisualizationPtr_->update(observation_);
    publishPerceptiveReferencePaths();       // throttled
    logPerceptiveFootPlacementDebug();        // throttled
}
target_manager_->update(observation_);
mpc_mrt_interface_->setCurrentObservation(observation_);
```

### alignPerceptiveBaseHeightToTerrain (linear kalman + perceptive 전용)

```cpp
x = rbd[3]; y = rbd[4];
terrainH = samplePerceptiveTerrainHeight(x, y);
if (terrainH) rbd[5] = *terrainH + perceptive_com_height_;
```

- `samplePerceptiveTerrainHeight` → `ConvexRegionSelector::sampleTerrainHeight(x,y)` → `smooth_planar` layer 조회.
- linear kalman 에스티메이터는 z drift가 있어서, 지형 정보 있으면 z를 지형 + comHeight 로 강제 교정. `ground_truth` 에스티메이터는 이미 정확하므로 건너뜀.

### publishPerceptiveReferencePaths (throttled)

```cpp
if (subscription 없음) return;
if (dt < minPublishDt) return;
if (!perceptiveRefMgr->getLatestReferencePaths(raw, terrainAware)) return;
publish(raw_base_path, terrain_aware_base_path);
```

RViz에서 raw target (베이스 평면 기준)과 terrain-aware target (pitch/z 수정된 것)을 나란히 비교해 보기 위함. `minReferencePathPublishTimeDifference_`로 rate limit.

### logPerceptiveFootPlacementDebug (throttled)

```cpp
debugInfo = perceptiveRefMgr->getLatestFootPlacementDebugInfo();
RCLCPP_INFO: t=... contact=[..] active=[..] poly=[vertex counts] proj_z=[heights] init_stand_final=[..]
```

`enable_perceptive_foot_placement_constraint_`일 때만 발행. foot placement가 언제 켜졌는지, convex polygon이 제대로 만들어졌는지 (vertex count > 0) 진단용.

## 5. 새 접근 경로 (dynamic_cast)

여러 곳에서
```cpp
auto* perceptiveRefMgr = dynamic_cast<PerceptiveLeggedReferenceManager*>(
    legged_interface_->getReferenceManagerPtr().get());
if (perceptiveRefMgr == nullptr) return;
```

→ **interface가 perceptive가 아니면 silent skip**. `enable_perceptive_` 플래그만 체크하는 대신 dynamic_cast로도 가드하기 때문에, interface가 어떤 이유로 초기화 실패했을 때도 crash 없이 넘어간다.

## 관련 topic / marker

| 방향 | topic | 타입 | 내용 |
|---|---|---|---|
| 구독 | `/convex_plane_decomposition_ros/planar_terrain` | `PlanarTerrain` | 외부 terrain 입력 (`PlanarTerrainReceiver`) |
| 발행 | `/perceptive_reference/raw_base_path` | `nav_msgs/Path` | MPC가 처음 받은 base target (terrain aware 전) |
| 발행 | `/perceptive_reference/terrain_aware_base_path` | `nav_msgs/Path` | pitch/z 수정된 base target |
| 발행 (RViz) | FootPlacementVisualization, SphereVisualization MarkerArrays | — | convex region / foothold / body sphere |

## 플래그 조합 예시

| 목적 | 플래그 조합 |
|---|---|
| perceptive pipeline off, 기본 평지 | `enable_perceptive=false` |
| perceptive interface만 붙이고 동작은 평지와 동일 (디버그/측정용) | `enable_perceptive=true`, 나머지 enable_*=false |
| terrain-aware base reference만 | `enable_perceptive=true`, `enable_perceptive_reference_modification=true` |
| 완전 perceptive 모드 | 모든 `enable_perceptive_*=true`, `perceptive_foot_placement_boundary_margin≈0.02` |

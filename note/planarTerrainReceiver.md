# PlanarTerrainReceiver

## Summary

`ocs2::SolverSynchronizedModule`을 상속. ROS 토픽으로 들어오는 `convex_plane_decomposition_msgs/PlanarTerrain` 메시지를 **MPC solver가 돌기 직전**에 안전하게 `PerceptiveLeggedInterface` 의 공유 데이터(planarTerrain, SDF)로 반영한다. subscribe는 DDS 스레드에서 발생해 내부 버퍼에 저장만 하고, 실제 적용은 `preSolverRun`에서 mutex로 보호하며 수행.

## 생성자

```cpp
PlanarTerrainReceiver(node, planarTerrainPtr, signedDistanceFieldPtr,
                      terrainDataMutexPtr, mapTopic, sdfElevationLayer)
```

- `planarTerrainPtr_`, `sdfPtr_`, `terrainDataMutexPtr_` — `PerceptiveLeggedInterface`가 소유하는 포인터들. 교체가 아니라 in-place `*ptr = ...` 로 덮어쓰기 때문에 reference manager / precomputation / constraint 쪽이 잡고 있는 포인터 유효성 유지.
- `mapTopic` — `"/convex_plane_decomposition_ros/planar_terrain"` (CtrlComponent에서 하드코딩)
- `sdfElevationLayer` — `"elevation"` (SDF 재계산 기준 layer)

### QoS

```
rclcpp::QoS(1), reliable, transient_local
```

- `transient_local` : publisher(예: `planar_terrain_publisher`)가 `transient_local`로 미리 한 번 발행해두면 나중에 접속한 subscriber도 최신 msg 한 개를 받을 수 있다. 즉 controller가 나중에 올라오는 순서에서도 terrain이 전달됨.

## Subscription callback (DDS thread)

들어온 메시지를 `convex_plane_decomposition::fromMessage(msg)` 로 PlanarTerrain 구조체로 변환 → 내부 `planarTerrain_` 버퍼에 저장. 이때 `mutex_` (subscription 전용)로 보호하며 `updated_ = true`.

**NaN inpainting**:
```cpp
if (elevationData.hasNaN()) {
    inpaint = elevationData.minCoeffOfFinites();
    elevationData = elevationData.unaryExpr([=](float v) { return isfinite(v) ? v : inpaint; });
}
```
NaN 셀을 map의 최소 유한값으로 채움 → SDF 계산 및 finite diff에서 NaN 전파 방지. 경고 로그도 한 번 출력.

$$
h_{\mathrm{inpaint}} = \min\{h : h \text{ is finite}\},\quad h(\mathbf{p}) \leftarrow \begin{cases}h(\mathbf{p}), & \text{finite} \\ h_{\mathrm{inpaint}}, & \text{NaN}\end{cases}
$$

## preSolverRun(initTime, finalTime, currentState, referenceManager)

매 MPC iteration 직전 호출 (MPC solver가 `addSynchronizedModule`로 등록).

```cpp
if (updated_) {
    PlanarTerrain latest;
    {
        lock(mutex_);
        updated_ = false;
        latest = planarTerrain_;            // subscription 버퍼 → 로컬 복사
    }

    {
        lock(*terrainDataMutexPtr_);        // interface가 공유하는 mutex
        *planarTerrainPtr_ = latest;        // interface 쪽 포인터에 덮어쓰기
        maxH = elevation.maxCoeffOfFinites() + 3 * 0.1;
        sdfPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, "elevation", maxH);
    }
}
```

**두 단계 mutex**:
1. `mutex_` — subscription 스레드와 충돌 방지
2. `*terrainDataMutexPtr_` — `ConvexRegionSelector::update` 가 snapshot 뜰 때 쓰는 공유 mutex

**SDF 재계산**:
$$
h_{\max}^{\mathrm{SDF}} = \max\{h : h \text{ finite}\} + 3 \cdot 0.1,\quad \text{SDF = computed over elevation layer up to } h_{\max}^{\mathrm{SDF}}
$$

$3 \cdot 0.1 = 0.3\,\text{m}$의 상단 margin — 지형 위로 로봇이 어느 정도 뛰어오를 수 있는 거리까지 SDF를 유효하게 유지.

## 등록 위치

```cpp
// CtrlComponent::setupMpc
if (enable_perceptive_) {
    auto receiver = std::make_shared<PlanarTerrainReceiver>(
        node_,
        dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
        dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
        dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getTerrainDataMutexPtr(),
        "/convex_plane_decomposition_ros/planar_terrain", "elevation");
    mpc_->getSolverPtr()->addSynchronizedModule(receiver);
}
```

**SolverSynchronizedModule** : OCS2 solver가 시작할 때 `preSolverRun(initTime, finalTime, state, refManager)` 을 호출한다. 이 훅이 perceptive에서 terrain 업데이트를 MPC에 일관성 있게 반영하는 곳.

## Upstream / Downstream

```
[Upstream] planar_terrain_publisher or external perception pipeline
    → /convex_plane_decomposition_ros/planar_terrain (reliable, transient_local)

PlanarTerrainReceiver
    → *planarTerrainPtr_ (공유)
    → *sdfPtr_  (elevation 기반 SDF 재계산)

[Downstream]
    ├─ ConvexRegionSelector::update   — mutex lock → terrain copy → per-foot polygon 계산
    ├─ PerceptiveLeggedReferenceManager::modifyReferences
    │     └─ grid map의 smooth_planar layer로 normal / height 샘플링
    ├─ FootCollisionConstraint         — SDF 거리 기반
    └─ SphereSdfConstraint             — SDF 거리 기반
```

# Perceptive OCS2 Overview

## 1. 목표

평지 기반 baseline 파이프라인(`Keyboard input → TargetManager → TargetTrajectories → SwitchedModelReferenceManager → SwingTrajectoryPlanner(terrainHeight=0) → MPC → WBC → Motor`) 을 지형 인식 기반으로 확장하는 것.

지형 정보가 합류하는 지점
- base reference(pitch, z)를 지형 경사/높이로 수정
- swing foot 궤적의 lift-off/touch-down 높이를 발 아래 지형에 맞춤
- stance foot 위치를 planar region 내부로 제한(foot placement)
- swing foot/몸체가 장애물과 충돌하지 않도록 SDF 거리 제약

이 모든 수정은 기존 MPC decision variable이나 dynamics를 바꾸지 않고 **reference와 soft constraint를 terrain-aware로 바꾸는 방식**으로 들어간다. 즉 centroidal OCP 자체는 그대로.

## 2. enable 분기 (feature flags)

`CtrlComponent`가 런치 파라미터로 읽는 플래그 — 모두 독립적으로 on/off 가능.

| 파라미터 | 끄면 | 켜면 |
|---|---|---|
| `enable_perceptive` | `LeggedInterface` 사용 (평지) | `PerceptiveLeggedInterface` 사용 |
| `enable_perceptive_reference_modification` | target trajectory 그대로 MPC에 전달 | base pitch·z를 지형에 맞춰 수정 (`PerceptiveLeggedReferenceManager::modifyReferences`) |
| `enable_perceptive_foot_placement_constraint` | stance foot 위치 제약 없음 | `FootPlacementConstraint` soft 추가 |
| `enable_perceptive_foot_collision_constraint` | swing foot 충돌 제약 없음 | `FootCollisionConstraint` soft 추가 |
| `enable_perceptive_body_collision_constraint` | body 충돌 제약 없음 | `SphereSdfConstraint` soft 추가 (calf 4개 sphere) |
| `perceptive_foot_placement_boundary_margin` | — | polygon 제약을 내부로 수축시키는 margin [m] |

`enable_perceptive`만 켜도 `PerceptiveLeggedInterface`가 설치되지만 개별 제약은 꺼져 있어 기본 MPC와 거의 동일. 본격적인 terrain-aware 거동을 보려면 `enable_perceptive_reference_modification`까지 같이 켜야 한다.

런치 파일에서 plus 로드되는 것
- `publish_static_terrain` — `planar_terrain_publisher` executable을 같이 띄워서 MuJoCo scene 파일을 한 번 읽어 `/convex_plane_decomposition_ros/planar_terrain` 토픽으로 발행
- `terrain_scene_file` — MuJoCo XML scene 이름 (예: `basic_step`)
- `terrain_smoothing_radius` — 고도 grid map의 Gaussian smoothing 반경 [m]

## 3. 전체 흐름

```
MuJoCo scene.xml
    ↓ (parse <geom type="box"/>)
planar_terrain_publisher (standalone node)
    ↓ /convex_plane_decomposition_ros/planar_terrain
PlanarTerrainReceiver (SynchronizedModule)
    ↓ (mutex)
planarTerrainPtr_ + signedDistanceFieldPtr_ (PerceptiveLeggedInterface 소유)
    │
    ├─ ConvexRegionSelector::update(modeSchedule, initState, rawTargetTraj)
    │     → 각 stand phase마다 nominal foothold 계산 → planar region projection → convex polygon growing
    │
    ├─ PerceptiveLeggedReferenceManager::modifyReferences
    │     ├─ 11-node horizon에서 smooth_planar layer로 terrain pitch/height 계산
    │     ├─ pitchBlend=0.6, heightBlend=0.5로 blending + clamp
    │     └─ ConvexRegionSelector update + SwingTrajectoryPlanner 업데이트 (projection의 z를 lift/touch 높이로)
    │
    ├─ PerceptiveLeggedPrecomputation::request
    │     └─ convex polygon → half-space 선형제약 (A·p + b ≥ 0), foot placement 파라미터
    │
    └─ Soft constraints (problem_ptr_->stateSoftConstraintPtr)
          ├─ FootPlacementConstraint (per foot)  — uses Precomputation params
          ├─ FootCollisionConstraint (per foot)  — SDF 거리 - clearance
          └─ SphereSdfConstraint (body)          — 4 calf sphere × SDF distance - radius

MPC (평지와 동일한 centroidal OCP, 위 soft constraint와 수정된 reference만 바뀜)
    ↓
WBC (바뀐 것 없음, centroidal policy 그대로 받아서 QP)
    ↓
Motor
```

## 4. 핵심 개념 정리

- **Planar Terrain** (`convex_plane_decomposition::PlanarTerrain`) : 여러 `PlanarRegion`(평면 하나씩) + `grid_map::GridMap`(elevation + smooth_planar layer). 각 PlanarRegion은 `transformPlaneToWorld`, `boundaryWithInset.boundary`(외곽), `boundaryWithInset.insets`(안쪽 여유) 를 가진다.
- **PlanarTerrainProjection** : 월드 좌표의 한 점을 가장 가까운 planar region에 투영한 결과 — `regionPtr`, `positionInWorld`, `positionInTerrainFrame`.
- **Convex polygon growing** : projection 점 주변에서 해당 region의 inset boundary 안쪽으로만 성장하는 convex polygon을 만들어, foot placement 제약을 선형 부등식들로 표현 가능하게 함.
- **Signed Distance Field (SDF)** (`grid_map::SignedDistanceField`) : elevation grid에서 유한차분으로 계산된 3D SDF. 장애물 표면=0, 자유공간>0, 내부<0.
- **Nominal foothold** : target trajectory의 desired foot 위치에 pitch 보정(offset = tan(-pitch)·h)을 적용한 월드 좌표의 이상적 디딤 위치. 이 위치로 planar region projection → convex polygon 을 만든다.

## 5. 코드 위치 (dev 브랜치)

```
controllers/ocs2_quadruped_controller/src/perceptive/
├── interface/
│   ├── PerceptiveLeggedInterface.cpp       [perceptiveLeggedInterface.md]
│   ├── PerceptiveLeggedReferenceManager.cpp [perceptiveLeggedReferenceManager.md]
│   ├── PerceptiveLeggedPrecomputation.cpp  [perceptiveLeggedPrecomputation.md]
│   └── ConvexRegionSelector.cpp            [convexRegionSelector.md]
├── constraint/
│   ├── FootPlacementConstraint.cpp         [footPlacementConstraint.md]
│   ├── FootCollisionConstraint.cpp         [footCollisionConstraint.md]
│   └── SphereSdfConstraint.cpp             [sphereSdfConstraint.md]
├── synchronize/
│   └── PlanarTerrainReceiver.cpp           [planarTerrainReceiver.md]
├── publisher/
│   ├── StaticPlanarTerrainPublisher.cpp    [staticPlanarTerrainPublisher.md]
│   └── PlanarTerrainVisualizer.cpp
└── visualize/
    ├── FootPlacementVisualization.cpp
    └── SphereVisualization.cpp

controllers/ocs2_quadruped_controller/src/control/
└── CtrlComponent.cpp                       [perceptiveCtrlComponent.md] (perceptive 관련 부분만)
```

## 6. 한 줄 요약

> 평지 baseline OCS2에 **(a) 외부 terrain 메시지 수신/SDF 재계산 `PlanarTerrainReceiver`**, **(b) target pitch·z를 지형에 맞춰 수정하는 `PerceptiveLeggedReferenceManager::modifyReferences`**, **(c) 각 stand phase에 대해 foot이 디딜 convex region을 고르는 `ConvexRegionSelector::update` + `PerceptiveLeggedPrecomputation` 의 polygon → half-space 변환**, **(d) 이를 MPC OCP에 soft constraint로 넣는 `FootPlacementConstraint / FootCollisionConstraint / SphereSdfConstraint`** 네 계층이 올라가 있고, 이 모든 것이 `enable_perceptive_*` 플래그로 개별 on/off 된다. WBC는 수정 없음.

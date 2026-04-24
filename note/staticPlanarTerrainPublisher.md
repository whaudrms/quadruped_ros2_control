# StaticPlanarTerrainPublisher

## Summary

**Standalone ROS 2 node** (`planar_terrain_publisher`). 실제 perception pipeline(depth camera → plane decomposition) 을 대신해서, **MuJoCo scene XML 파일을 정적으로 파싱**해 거기 들어 있는 `<geom type="box"/>` 들의 윗면을 `convex_plane_decomposition_msgs::PlanarTerrain` 메시지로 변환해 발행한다. dev 브랜치에서 `ocs2_quadruped_controller` 패키지에 같이 빌드되는 두 번째 executable.

`PerceptiveLeggedInterface`의 기본 더미 평면(5×5 m)만으로는 계단/박스 같은 지형이 없으므로, 이 노드가 사전 계산된 terrain을 transient_local QoS로 한 번 publish → `PlanarTerrainReceiver` 가 구독해서 controller에 반영하는 구조.

이 파일은 [CORA] Legged Robot에는 코드 분석 문서가 없고(dev 브랜치 신규 추가), 구현 기준으로 정리.

## 실행 인자 (ROS parameter)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `scene_file` | (required) | MuJoCo `.xml` 파일 경로 |
| `topic` | `/convex_plane_decomposition_ros/planar_terrain` | 발행 토픽 |
| `frame_id` | `odom` | grid map/region frame |
| `resolution` | `0.03 m` | grid map 해상도 |
| `smoothing_radius` | `0.12 m` | Gaussian smooth 반경 |
| `publish_rate` | `0.0 Hz` | `>0` 이면 주기적으로 재발행, 아니면 최초 한 번만 |

## 파이프라인

```
scene.xml
    ↓ loadSceneDescription (regex parse)
SceneDescription { hasFloorPlane, vector<BoxSurface> }
    ↓ buildPlanarTerrain (resolution, frameId, smoothing)
convex_plane_decomposition::PlanarTerrain
    ↓ convex_plane_decomposition::toMessage
convex_plane_decomposition_msgs::PlanarTerrain  → publish (transient_local)
```

## loadSceneDescription(sceneFile)

- regex로 `<geom ... />` 태그들 찾기 (`R"(<geom\b[^>]*/>)"`). MuJoCo XML을 진짜 parser 없이 텍스트 매칭만으로 뽑아냄 — scene이 단순한 형태일 때만 동작.
- `type="plane"` 이면 `hasFloorPlane = true`
- `type="box"` 이면 `pos`, `size`, (optional) `quat` 을 읽어서 `BoxSurface` 구조체로 저장:

```cpp
struct BoxSurface {
  Eigen::Vector3d topCenterInWorld;           // centerPos + R·(0,0,size.z)
  Eigen::Matrix3d rotationPlaneToWorld;       // quat → R
  double halfX, halfY;                         // size.x, size.y (MuJoCo는 half-size!)
};
```

MuJoCo의 box `size` 는 이미 half-size. `topCenter = centerPos + R·(0, 0, size.z)` 은 top face의 월드 좌표 중심.

## makeRectangularRegion(center, R, halfX, halfY, insetMargin) → PlanarRegion

box 윗면 하나를 PlanarRegion으로 변환:
- `transformPlaneToWorld` : rotation=R, translation=center
- `bbox2d` : $(-\text{halfX}, -\text{halfY}) \to (+\text{halfX}, +\text{halfY})$
- `boundaryWithInset.boundary` : bbox와 동일한 4-꼭짓점 사각형
- `boundaryWithInset.insets` : 각 변을 `insetMargin`만큼 안쪽 (하한 `halfX·0.5`, 즉 면적이 절반 이하로는 줄어들지 않게 guard)

## buildPlanarTerrain(scene, resolution, frameId, smoothingRadius)

1. 모든 box의 top corners를 2D로 모아 **전체 bounding box** `[minX, maxX] × [minY, maxY]` 계산. 비어 있으면 기본 `[-3,3]×[-2,2]`. 각 방향으로 `floorMargin = 1.0 m` 확장.
2. `scene.hasFloorPlane` 이거나 surfaces 가 비어 있으면 **바닥 floor plane** 도 PlanarRegion으로 하나 추가 (insetMargin=0.02 m).
3. 각 box surface를 `makeRectangularRegion`으로 PlanarRegion 추가 (insetMargin = min(0.02, 0.2·halfX, 0.2·halfY), 최소 0.005 m).
4. `grid_map::GridMap` 생성:
   - layers: `elevation` (0.0 초기화), `smooth_planar` (0.0 초기화)
   - size: `(2·halfX, 2·halfY)`, resolution 지정
   - frame_id 설정
5. GridMapIterator로 **모든 cell에서 surfaceHeightAt** 계산:
   - 바닥 plane 가정 z=0
   - 각 surface에 대해 cell XY를 surface의 local frame으로 역변환 → halfX/halfY 안쪽이면 world z 계산 → 기존 최대 height와 비교해서 **max**로 갱신 (겹치는 surface 중 가장 높은 것).
6. `fillSmoothedLayer(map, "elevation", "smooth_planar", smoothingRadius)` 호출 → Gaussian smooth.

### surfaceHeightAt(surface, x, y, &z)

surface의 local 좌표 변환:
$$
\mathbf{r} = \begin{bmatrix} x - c_x \\ y - c_y \end{bmatrix},\quad
\mathbf{p}_{\text{local}} = P^{-1}\mathbf{r},\quad P = \begin{bmatrix}R_{00} & R_{01}\\ R_{10} & R_{11}\end{bmatrix}
$$
$|p_x| \le \text{halfX}$, $|p_y| \le \text{halfY}$ 이면 그 점은 surface 위.
$$
z = c_z + R_{20}\,p_x + R_{21}\,p_y
$$

### fillSmoothedLayer(map, src, dst, smoothingRadius)

각 cell에서 `radiusCells = ceil(smoothingRadius / resolution)` 범위 내 셀들에 대해 Gaussian 가중평균:
$$
\sigma = \max(r_{\text{res}},\; 0.5\cdot \text{smoothingRadius}),\quad
w = \exp\!\left(-\frac{\Delta x^2 + \Delta y^2}{2\sigma^2}\right)
$$
`smoothingRadius ≤ 0` 이면 단순 복사.

**왜 "smooth_planar" 를 따로 두나?** `PerceptiveLeggedReferenceManager::modifyReferences`에서 지형 법선을 finite difference로 계산할 때 raw `elevation`을 쓰면 계단 경계에서 미분값이 튀어 target pitch가 튀는데, Gaussian-smooth된 layer로 계산하면 연속적인 base pitch reference가 나온다.

## QoS

```cpp
rclcpp::QoS qos(1);
qos.reliable();
qos.transient_local();
publisher_ = create_publisher<PlanarTerrain>(topic, qos);
publisher_->publish(terrainMsg_);          // 최초 1회
if (publishRate > 0) timer_.repeat(publishRate, publish);
```

`PlanarTerrainReceiver` 가 `transient_local` 로 구독하므로 publisher가 먼저 떠서 한 번만 발행해도 OK.

## 한계 / 전제

- `<geom type="box"/>` 만 인식. sphere / cylinder / capsule / hfield 등은 무시.
- regex-only XML 파싱 — MuJoCo `include` 나 JS-like 표현식이 들어가면 안전하지 않다. 단순한 scene XML에만 사용.
- PlanarRegion 하나 = box의 top face 한 면. 측면이나 아래에서 접근하는 상황은 표현 불가.
- 생성된 터라 `scene.xml`과 실제 MuJoCo 시뮬 지형이 일치해야 함 — scene 수정 후에는 노드도 재시작 필요.

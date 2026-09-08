# Foot-placement constraint 독립 시각화

이 데모는 `exp.sh`, 로봇, MPC, gait schedule 없이 단일 nominal foothold에 대한
foot-placement polygon의 생성 과정을 보여 준다. SDF collision, swing trajectory,
robust constraint는 계산하지 않는다. 기존 컨트롤러 동작이나 지형 demo launch는 변경하지 않는다.

## 실행

두 터미널 모두 `/home/tony/GO2_ws`에서 `source install/setup.bash`를 실행한다.
첫 번째 터미널에서 기존 지형 공급기를 실행한다.

```bash
ros2 launch convex_plane_decomposition_ros demo.launch.py
```

두 번째 터미널에서 독립 시각화를 실행한다.

```bash
ros2 launch ocs2_quadruped_controller foot_placement_constraint_demo.launch.py
```

두 번째로 열리는 RViz는 지형과 foot-placement constraint만 표시한다.
기존 지형 demo가 실행하는 spline 노드는 그대로지만 이 뷰에는 표시하지 않는다.
RViz의 **Publish Point**로 지형을 클릭하면 nominal foothold가 바뀌며 영역을 다시 계산한다.
초기 위치는 `(0, 0, 0.4)` m이며, **F**로 선택 지점에 카메라를 맞추거나 휠로 확대할 수 있다.

기존 RViz를 사용하려면 `rviz:=false`를 추가하고, Add → By topic에서
`/foot_placement_constraint_demo`의 MarkerArray를 추가한다.
기존 `Footholds + Swing Splines` display는 끄면 된다.
Fixed Frame은 입력 PlanarTerrain의 gridmap frame과 같아야 한다(기존 demo: `odom`).
다른 frame의 클릭은 TF 변환하지 않고 경고 후 무시한다.

## 표시 의미

| 표시 | 의미 |
| --- | --- |
| 파란 점 | 사용자 지정 nominal foothold. 실제 로봇 FK로 계산한 목표점이 아님 |
| 분홍 점 | 선택된 planar region의 inset에 투영한 seed. MPC 최적화 결과가 아님 |
| 노란 선 | seed 주변에서 성장시킨 convex polygon, 추가 margin 적용 전 |
| 초록 면/선 | 최종 반공간 부등식들의 교집합을 평면 위에 표시한 영역 |
| 초록 화살표 | 남아 있는 각 경계의 안쪽 단위 법선 방향 |

마커에는 겹침을 피하기 위한 수 mm의 시각적 높이 오프셋이 있다.
이는 제약식의 높이 여유가 아니다.
설명 글씨는 표시하지 않는다. 클릭 후 margin 적용 여부는 터미널의 `shrink` 로그로 확인한다.

## 생성 과정과 수식

1. `getBestPlanarRegionAtPositionInWorld`로 nominal point와의 3D 제곱거리가
   가장 작은 투영을 선택한다. 추가 penalty는 0이다. 투영은 지형 처리기가 이미
   생성한 `boundaryWithInset.insets`를 사용하며, 단순한 수직 투영과 다를 수 있다.
2. 선택된 region의 `boundaryWithInset.boundary` 내부에서 투영점을 seed로
   `growConvexPolygonInsideShape`를 실행한다. 기본 꼭짓점 수 16, growth factor 1.05이다.
   전체 지형의 convex hull이 아니라 구멍/비볼록 외곽 안에 포함되는 국소 convex 영역이다.
3. 평면좌표 `u=(x_P,y_P)`에 대해 각 변을 안쪽 부등식 `a_i^T u+b_i >= 0`으로 변환한다.
4. 요청 margin `m`만큼 각 경계를 안쪽으로 이동한다:
   `b'_i=b_i-m||a_i||`. 모든 행에서 seed의 **비정규화 slack > 1e-6**이어야
   적용하고, 그렇지 않으면 전체 margin을 취소하여 원래 `A,b`를 사용한다.
   이는 `PerceptiveLeggedPrecomputation::tryShrinkPolygonConstraint`의 fallback과 같다.
   따라서 좁은 영역에서는 `boundary_margin=0.05`여도 5 cm가 실제 적용되지 않을 수 있다.

초록 다각형은 활성 반공간으로 원래 다각형을 차례로 clipping한 결과다.
margin 이후 일부 변이 불필요해지면 초록 다각형의 꼭짓점은 16개보다 적을 수 있지만,
원래 제약식의 행 수는 16개다. 입력 꼭짓점 수가 맞지 않으면 초록 영역을 표시하지 않으며,
교집합이 퇴화한 경우 잘못된 면을 그리는 대신 마커를 지우고 경고한다.

월드 좌표에서는 `u=S R^T(p_W-t)`, `S=[I_2 0]`이므로
`A_W=A_P S R^T`, `b_W=b'_P-A_P S R^T t`, `A_W p_W+b_W >= 0`이다.
**foot-placement 부등식 자체는 평면 법선 방향 높이를 제한하지 않는다.**
초록 면은 이 부등식의 지면 단면이며, 3D에서 닫힌 다면체나 SDF collision volume이 아니다.
실제 MPC의 stance/swing 활성화 및 soft penalty는 이 독립 데모의 범위 밖이다.

## 파라미터 비교

노드를 종료 후 다른 margin으로 재실행하여 비교한다(실행 중 ros2 param set은 지원하지 않음).

```bash
ros2 launch ocs2_quadruped_controller foot_placement_constraint_demo.launch.py boundary_margin:=0.0
ros2 launch ocs2_quadruped_controller foot_placement_constraint_demo.launch.py boundary_margin:=0.10
```

launch 인자: `terrain_topic`(기본 `/planar_terrain`), `marker_topic`, `num_vertices`,
`growth_factor`, `boundary_margin`, `nominal_x/y/z`, `rviz`.
커스텀 지형을 쓰면 RViz의 Terrain 토픽과 Fixed Frame도 해당 입력에 맞춰 변경한다.

## 코드 위치

- 독립 노드: `src/perceptive/publisher/FootPlacementConstraintVisualizer.cpp`
- RViz: `config/foot_placement_constraint_demo.rviz`
- 비교 대상: `src/perceptive/interface/ConvexRegionSelector.cpp`,
  `src/perceptive/interface/PerceptiveLeggedPrecomputation.cpp`

패키지 재빌드:

```bash
colcon build --packages-select ocs2_quadruped_controller --symlink-install --parallel-workers 1
source install/setup.bash
```

헤드리스 ROS 통합 테스트(로봇을 시작하지 않으며, localhost 전용 domain 167 사용):

```bash
/usr/bin/python3 quadruped_ros2_control/controllers/ocs2_quadruped_controller/test/foot_placement_constraint_smoke_test.py \
  build/ocs2_quadruped_controller/foot_placement_constraint_visualizer
```

`--margin 0.0`으로 margin 없는 경우도 검증할 수 있다.

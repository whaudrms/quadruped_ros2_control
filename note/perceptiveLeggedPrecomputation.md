# PerceptiveLeggedPrecomputation

## Summary

`LeggedRobotPreComputation`을 상속. 기본 precompute(dynamics Jacobian 등)를 수행한 뒤, 각 MPC iteration에서 `ConvexRegionSelector`가 만든 convex polygon을 **월드 좌표의 발 위치에 대한 선형 부등식(halfspace) 제약**으로 변환해 `footPlacementConParameters_[leg]` 에 저장. `FootPlacementConstraint`가 매 `getValue` 호출마다 이 파라미터를 읽는다.

## 생성자

```cpp
PerceptiveLeggedPrecomputation(pinocchioInterface, info, swingPlanner, settings,
                               convexRegionSelector, footPlacementBoundaryMargin)
```

`footPlacementConParameters_`를 발 개수 크기로 초기화하고 각 항목을 `makeSafeFootPlacementConstraintParameter()`로 채운다:
$$
A = \mathbf{0}_{n_v \times 3},\quad \mathbf{b} = \mathbf{1}_{n_v}
$$
→ 값이 **항상 양수** 이므로 constraint가 활성화되더라도 `h = A\mathbf{p} + b = \mathbf{1} \ge 0$ 로 자연히 만족. projection이 없는 swing leg에서 safety fallback.

## request(request, t, x, u)

1. **Short-circuit**: cost/constraint/soft-constraint 중 어느 것도 요청되지 않았으면 바로 return.
2. base precomputation 호출 (`LeggedRobotPreComputation::request`)
3. **Constraint 또는 SoftConstraint 요청이 있을 때만** per-foot 파라미터를 갱신:

```cpp
for each leg i:
    projection = convexRegionSelectorPtr_->getProjection(i, t);
    if projection.regionPtr == nullptr:  // swing
        footPlacementConParameters_[i] = safeParams;   continue;
    polygon = convexRegionSelectorPtr_->getConvexPolygon(i, t);
    if polygon.size() < 3:                 params = safe; continue;

    (polytopeA, polytopeB) = getPolygonConstraint(polygon);      // plane 좌표계 2D 제약
    if polytopeA.rows() != numVertices_:   params = safe; continue;

    (activeA, activeB) = tryShrinkPolygonConstraint(polytopeA, polytopeB,
                                                    projection.positionInTerrainFrame);

    // 2D plane-frame 제약을 3D world-frame 발 위치 제약으로 변환
    P = [[1,0,0],[0,1,0]];  // drop z
    params.a = activeA * P * R^{-1};
    params.b = activeB + activeA * (t_xy);     // t_xy = inverse translation xy
    footPlacementConParameters_[i] = params;
```

**$A$, $b$의 월드 좌표 변환** (plane → world)

plane 좌표 $\mathbf{p}_{2d}$ 에 대한 제약이 $A_{\text{poly}}\mathbf{p}_{2d} + \mathbf{b}_{\text{poly}} \ge 0$ 일 때, 월드 발 위치 $\mathbf{p}_{ee}$ 는
$$
\mathbf{p}_{ee}^{\text{plane}} = R^{-1}(\mathbf{p}_{ee} - \mathbf{t}),\quad
\mathbf{p}_{2d} = P\,\mathbf{p}_{ee}^{\text{plane}}
$$
이므로
$$
\boxed{A_{\text{world}} = A_{\text{poly}}\,P\,R^{-1},\quad \mathbf{b}_{\text{world}} = \mathbf{b}_{\text{poly}} + A_{\text{poly}}\,\mathbf{t}_{xy}}
$$
$R$, $\mathbf{t}$ 는 `projection.regionPtr->transformPlaneToWorld.inverse()` 의 linear/translation. $P$ 는 z 성분 drop.

## getPolygonConstraint(polygon) — 2D polygon → half-space

각 변 (edge) $(A, B)$ 에 대해, 바깥쪽 법선 방향을 찾아 $A_i \mathbf{p} + b_i \ge 0$ 형태로 변환.

for each triple of consecutive vertices $(p_a, p_b, p_c)$:
$$
\mathbf{A}_i = \begin{bmatrix} y_b - y_a & x_a - x_b \end{bmatrix},\quad
b_i = y_a x_b - x_a y_b
$$
이 부등식의 부호를 결정짓기 위해 같은 polygon의 다음 점 $p_c$ 를 대입해 보고, $A_i p_c + b_i < 0$ 이면 내부가 음수 쪽이라는 뜻이므로 부호 반전:
$$
A_i \leftarrow -A_i,\quad b_i \leftarrow -b_i
$$
결과: $A\,\mathbf{p} + \mathbf{b} \ge 0 \iff \mathbf{p} \in \mathrm{polygon}$.

## tryShrinkPolygonConstraint(polytopeA, polytopeB, interiorPoint, → shrunkA, shrunkB)

각 부등식의 $b$ 값에서 법선 노름만큼 `footPlacementBoundaryMargin_`을 빼서 제약을 **내부로 수축**:
$$
\tilde b_k = b_k - m \cdot \|\mathbf{a}_k\|
$$

- `m ≤ 0` 이면 수축 없이 원본 리턴 `true`.
- 수축 후 `interiorPoint`가 여전히 **strict interior** 면 `true` (수축 결과를 사용).
- 아니면 `false` 리턴 → 호출자는 원본 `polytopeA`, `polytopeB`를 그대로 사용.

이는 **boundaryMargin이 너무 커서 convex region이 사라지는 경우를 자동으로 회피**하기 위한 guard. 런치 파라미터 `perceptive_foot_placement_boundary_margin`을 0.0~0.03 m 정도로 주면 안전 margin으로 동작.

## 제공하는 접근자

- `getFootPlacementConParameters()` → `std::vector<FootPlacementConstraint::Parameter>` — `FootPlacementConstraint::getValue`가 호출.
- `getPinocchioInterface()` → `SphereSdfConstraint`가 sphere kinematics에 현재 Pinocchio 상태를 주입할 때 씀.

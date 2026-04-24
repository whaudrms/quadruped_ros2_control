# FootPlacementConstraint

## Summary

`StateConstraint` (ConstraintOrder::Linear). 각 발 $i$에 대해 **stance 중인 phase**에서 발 위치가 `ConvexRegionSelector`가 고른 convex region 내부에 있도록 선형 부등식으로 제약. 실제 값 자체는 `PerceptiveLeggedPrecomputation`이 미리 만들어둔 `(A, b)` 파라미터를 그대로 쓰기 때문에 constraint 자체는 가볍다. soft constraint(`RelaxedBarrierPenalty`)로 감싸져 cost에 들어감.

## 생성자

```cpp
FootPlacementConstraint(referenceManager, endEffectorKinematics, contactPointIndex, numVertices)
```
- `referenceManagerPtr_` — perceptive reference manager로 dynamic_cast해서 플래그 조회
- `endEffectorKinematicsPtr_` — **단일 발** kinematics clone (PerceptiveLeggedInterface에서 `getEeKinematicsPtr({footName}, footName)`로 만든 것)
- `contactPointIndex_` — 0..3
- `numVertices_` — constraint output 차원

## isActive(time)

`PerceptiveLeggedReferenceManager::getFootPlacementFlags(time)[contactPointIndex_]` 반환.

즉 **현재 stance + 초기 stance 종료 이후**일 때만 제약 걸림 (세부 조건은 `perceptiveLeggedReferenceManager.md`의 `getFootPlacementFlags` 참고).

## getValue(time, state, preComp) → R^{numVertices}

```cpp
param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[i];
return param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
```

$$
\boxed{\mathbf{h}(\mathbf{x}) = A\,\mathbf{p}_{ee,i}(\mathbf{x}) + \mathbf{b} \;\ge\; \mathbf{0}}
$$

- $A \in \mathbb{R}^{n_v\times 3}$ — polygon의 각 변에 대한 "바깥 방향" half-space 법선들
- $\mathbf{b} \in \mathbb{R}^{n_v}$ — 법선에 대한 offset
- swing phase거나 projection이 없으면 Precomputation이 safe fallback($A=0$, $b=\mathbf{1}$)을 넣어둬서 $\mathbf{h} \equiv \mathbf{1}$ → 자연히 만족.

## getLinearApproximation(time, state, preComp)

end-effector 위치 선형 근사 ($\mathbf{p}_{ee}(x) \approx \mathbf{p}_{ee}(x_0) + \frac{\partial \mathbf{p}_{ee}}{\partial x} \Delta x$) 를 이용:

$$
\frac{\partial \mathbf{h}}{\partial \mathbf{x}} = A\,\frac{\partial \mathbf{p}_{ee,i}}{\partial \mathbf{x}}
$$

- `approx.f = A * p_ee + b`
- `approx.dfdx = A * p_ee_Jac`

`StateConstraint`이므로 input에 대한 gradient 없음. Linear order라 SQP가 Jacobian만 있으면 됨.

## 어디서 Relaxed Barrier로 감싸지는가

```cpp
// PerceptiveLeggedInterface::setupOptimalControlProblem
auto placementPenalty = new RelaxedBarrierPenalty(Config(1e-2, 1e-4));   // μ, δ
auto footPlacementConstraint = new FootPlacementConstraint(...);
problem_ptr_->stateSoftConstraintPtr->add(
    footName + "_footPlacement",
    make_unique<StateSoftConstraint>(move(footPlacementConstraint), move(placementPenalty)));
```

- μ = 0.01, δ = 1e-4 — barrier가 매우 약해서, 경계 근처에서만 cost 급증.
- soft이므로 MPC가 약간 벗어나는 해 자체는 허용함 (hard equality로 쓰면 infeasible 나기 쉬움).

## 데이터 흐름 요약

```
ConvexRegionSelector::update
    → feetProjections_, convexPolygons_ (phase별)
PerceptiveLeggedPrecomputation::request (매 MPC tick)
    → footPlacementConParameters_[i] = (A_world, b_world)
FootPlacementConstraint::getValue, getLinearApproximation
    → h = A·p_ee + b,  dh/dx = A · J_ee
RelaxedBarrierPenalty
    → cost
```

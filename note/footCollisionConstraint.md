# FootCollisionConstraint

## Summary

`StateConstraint` (ConstraintOrder::Linear). 스윙 중인 발 끝이 signed distance field(SDF) 기준으로 장애물 표면보다 clearance 이상 떨어지도록 강제. scalar 부등식 하나 ($1$-dim constraint) 이지만 SDF gradient × end-effector Jacobian으로 state에 대한 선형 근사를 제공.

## 생성자

```cpp
FootCollisionConstraint(referenceManager, endEffectorKinematics, sdfPtr,
                        contactPointIndex, clearance)
```

- `sdfPtr_` — `PerceptiveLeggedInterface`에서 공유하는 `grid_map::SignedDistanceField` (terrain 갱신 시 `PlanarTerrainReceiver`가 재계산)
- `endEffectorKinematicsPtr_` — 단일 발 kinematics clone
- `contactPointIndex_` — 0..3
- `clearance_` — perceptive interface에서 `0.01 m` 로 넣음

## isActive(time)

**안전 여유 포함 swing 판정** :
$$
a(t) = \neg c_i(t) \land \neg c_i(t + 0.025) \land \neg c_i(t - 0.05)
$$

즉 현재 시각에 swing이고, 0.025 s 뒤에도 swing이고, 0.05 s 전에도 swing이어야 한다 — stance/swing 전환 근방에서는 제약을 걸지 않음 (접촉 순간 SDF 값이 0 근처가 되어 barrier가 폭발하는 것을 피하려는 장치).

## getValue(time, state, preComp) → R^1

```cpp
value(0) = sdfPtr_->getDistanceAt(
              grid_map::Position3(endEffectorKinematicsPtr_->getPosition(state).front()))
           - clearance_;
```

$$
\boxed{h(\mathbf{x}) = d_{\mathrm{SDF}}(\mathbf{p}_{ee,i}(\mathbf{x})) - d_{\mathrm{clr}} \;\ge\; 0}
$$

- $d_{\mathrm{SDF}}$ : world 좌표에서 장애물 표면까지의 signed distance. 외부 + / 내부 −.
- $d_{\mathrm{clr}} = 0.01$ m
- positive 영역에서 barrier가 부드럽게 작동, 음수로 들어가면 cost가 상승.

## getLinearApproximation(time, state, preComp)

체인룰:
$$
\frac{\partial h}{\partial \mathbf{x}} = \underbrace{\nabla d(\mathbf{p}_{ee})^\top}_{1 \times 3}\,\underbrace{\frac{\partial \mathbf{p}_{ee}}{\partial \mathbf{x}}}_{3 \times n}
$$

```cpp
approx.f = getValue(...);
approx.dfdx = sdfGrad.transpose()
            * endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front().dfdx;
```

- `sdfPtr_->getDistanceGradientAt(...)` — grid_map SDF의 3D gradient (세 방향 유한차분).
- `getPositionLinearApproximation(state).front().dfdx` — 단일 발 Jacobian $\frac{\partial \mathbf{p}_{ee}}{\partial \mathbf{q}_{\text{gen}}}$.

## Soft constraint로 감싸기

```cpp
auto collisionPenalty = new RelaxedBarrierPenalty(Config(1e-2, 1e-3));  // μ=0.01, δ=1e-3
auto footCollisionConstraint = new FootCollisionConstraint(
    *reference_manager_ptr_, *eeKinematicsPtr, sdfPtr, i, 0.01);
problem_ptr_->stateSoftConstraintPtr->add(
    footName + "_footCollision",
    make_unique<StateSoftConstraint>(move(footCollisionConstraint), move(collisionPenalty)));
```

`δ`가 `FootPlacement`(1e-4)보다 한 자리 크다 → collision barrier가 더 일찍(먼 거리에서부터) 완만히 켜짐. 충돌 회피는 거리가 여유 있을 때부터 조금씩 신호를 주는 편이 안정적이기 때문.

## 주의

- `FootCollisionConstraint` 는 `endEffectorKinematicsPtr_` 가 **front()** 만 참조 — 단일 발 kinematics로 생성되어야 함. perceptive interface에서 `getEeKinematicsPtr({footName}, footName)`으로 호출.
- SDF는 외부 terrain이 바뀔 때 `PlanarTerrainReceiver::preSolverRun`에서 재계산 → 이 constraint의 값이 자연히 갱신된다.

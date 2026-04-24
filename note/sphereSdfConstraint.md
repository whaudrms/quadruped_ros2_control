# SphereSdfConstraint

## Summary

`StateConstraint` (ConstraintOrder::Linear). 로봇 body를 여러 sphere로 근사(`PinocchioSphereInterface`), 각 sphere의 중심이 SDF 기준으로 sphere 반지름 이상의 거리를 유지하도록 하는 body collision avoidance 제약. FootCollisionConstraint와 달리 per-foot이 아니라 **전체 sphere 집합 한꺼번에** 반환 ($n_{\text{sphere}}$ -dim).

## 생성자 / 복사

```cpp
SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics,
                    std::shared_ptr<grid_map::SignedDistanceField> sdfPtr)
```
- `sphereKinematicsPtr_` — 내부적으로 clone()으로 독립 복사본 유지 (MPC 내부에서 상태 복사 시 safety)
- `numConstraints_` = `sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal()`

dev 브랜치의 `PerceptiveLeggedInterface`는 4개 calf 링크를 sphere로 근사 — `collisionLinks = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"}`, `maxExcesses = 0.02 m` each, shrink factor `0.6`.

## getValue(time, state, preComp) → R^{numConstraints}

```cpp
sphereKinematicsPtr_->setPinocchioInterface(
    cast<PerceptiveLeggedPrecomputation>(preComp).getPinocchioInterface());
auto positions = sphereKinematicsPtr_->getPosition(state);    // vector<vector3_t>
auto radii    = sphereKinematicsPtr_->getPinocchioSphereInterface().getSphereRadii();
for i in 0..numConstraints_:
    value(i) = sdfPtr_->getDistanceAt(Position3(positions[i])) - radii[i];
```

$$
\boxed{h_j(\mathbf{x}) = d_{\mathrm{SDF}}(\mathbf{p}_j(\mathbf{x})) - r_j,\quad j = 1..n_{\text{sphere}}}
$$

- `setPinocchioInterface` — precomputation이 매 iteration 현재 상태로 업데이트 해둔 Pinocchio 상태를 sphere FK에 주입 (CppAD 재계산을 피하려는 최적화).
- positive면 충돌 안 함, 0 근처면 표면 닿음, 음수면 관통.

## getLinearApproximation

각 sphere에 대한 state gradient:

$$
\frac{\partial h_j}{\partial \mathbf{x}} = \nabla d(\mathbf{p}_j)^\top \frac{\partial \mathbf{p}_j}{\partial \mathbf{x}}
$$

```cpp
auto sphereApprox = sphereKinematicsPtr_->getPositionLinearApproximation(state); // per-sphere
for i in 0..numConstraints_:
    sdfGrad = sdfPtr_->getDistanceGradientAt(Position3(positions[i]));
    approx.dfdx.row(i) = sdfGrad.transpose() * sphereApprox[i].dfdx;
```

`approx.f = getValue(...)`로 스칼라 값 배열 채우고, 각 행에 위 체인룰.

## Soft constraint로 감싸기

```cpp
// PerceptiveLeggedInterface
auto bodyCollisionPenalty = new RelaxedBarrierPenalty(Config(1e-3, 1e-3));  // μ=1e-3, δ=1e-3
auto sphereSdfConstraint = new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_);
problem_ptr_->stateSoftConstraintPtr->add(
    "sdfConstraint",
    make_unique<StateSoftConstraint>(move(sphereSdfConstraint), move(bodyCollisionPenalty)));
```

foot collision 대비 μ가 10배 작음 → body penalty는 훨씬 약하다. body 제약은 발 collision보다 완화되어 있고, sphere approximation 자체가 보수적(최대 excess로 경계 부풀림, shrink factor 0.6)이라 barrier 자체는 강하게 안 걸어도 안전한 편.

## 데이터 흐름 요약

```
PerceptiveLeggedInterface::setupOptimalControlProblem
    ├─ pinocchioSphereInterfacePtr_ ← 4 calf links, excess=0.02, shrink=0.6
    └─ SphereSdfConstraint(sphereKinematics, sdfPtr_) + RelaxedBarrier

PerceptiveLeggedPrecomputation::request (매 MPC tick)
    └─ 현재 Pinocchio 상태를 보관 → SphereSdfConstraint가 접근

SphereSdfConstraint::getValue
    └─ 4 sphere 각각 d - r
```

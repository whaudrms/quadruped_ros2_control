// Robust phase impact-velocity lower bound.
//
// Returns the scalar  h(x,u) = ġ + v_max  for use as a soft inequality:
//
//     ġ ≥ -v_max       ⇔        h ≥ 0
//
// where ġ = n_l^T J_foot,l(q) v_pin(x,u) (foot velocity along the terrain
// normal). v_max is read from RobustWindowData (loaded from
// robustPhase.v_max in task.info, default 0.6 m/s).
//
// Active across the entire robust window [t_a, t_b] (same as the
// RobustGuardApproachConstraint counterpart, which enforces the upper bound
// ġ ≤ 0). Together they form an impact-velocity envelope -v_max ≤ ġ ≤ 0,
// preventing the OCP from satisfying the boundary g(t_a)=+d, g(t_b)=-d
// with arbitrarily large descent speed.
//
// Per chat7 priority #2 — added on top of the chat7 P/d/v_max physical-
// consistency retuning (Stage 1a, P=10 → T_robust=0.20s with d=0.05).
//
// Soft constraint via RelaxedBarrierPenalty (same penalty config as
// RobustGuardApproachConstraint).

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot {

class RobustGuardVelocityLowerBoundConstraint final : public StateInputConstraint {
public:
    RobustGuardVelocityLowerBoundConstraint(const SwitchedModelReferenceManager& referenceManager,
                                            const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                            size_t contactPointIndex);

    ~RobustGuardVelocityLowerBoundConstraint() override = default;
    RobustGuardVelocityLowerBoundConstraint* clone() const override {
        return new RobustGuardVelocityLowerBoundConstraint(*this);
    }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComp) const override;

private:
    RobustGuardVelocityLowerBoundConstraint(const RobustGuardVelocityLowerBoundConstraint& rhs);

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const size_t contactPointIndex_;
};

}  // namespace ocs2::legged_robot

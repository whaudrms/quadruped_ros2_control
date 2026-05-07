// Robust phase approach constraint.
//
// Returns the scalar value -ġ where:
//   ġ = n_l^T J_foot,l(q) v_pin(x, u)
// is the foot velocity component along the terrain normal.
//
// Active across the entire robust window [t_a, t_b]. The same instance is registered TWICE:
//   1. Wrapped by StateInputSoftConstraint + RelaxedBarrierPenalty → enforces -ġ ≥ 0
//      (i.e., the foot only descends toward the terrain inside the window).
//   2. Wrapped by StateInputSoftConstraint + QuadraticPenalty(2*w_v) → adds soft cost
//      L = w_v * (-ġ)² = w_v * ġ²  (impact-velocity softening).
// Both penalties operate on the same scalar h = -ġ; QuadraticPenalty is sign-symmetric so
// the cost form is identical to the explicit ġ² formulation in the plan.

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot {

class RobustGuardApproachConstraint final : public StateInputConstraint {
public:
    RobustGuardApproachConstraint(const SwitchedModelReferenceManager& referenceManager,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  size_t contactPointIndex);

    ~RobustGuardApproachConstraint() override = default;
    RobustGuardApproachConstraint* clone() const override { return new RobustGuardApproachConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const vector_t& input,
                                                             const PreComputation& preComp) const override;

private:
    RobustGuardApproachConstraint(const RobustGuardApproachConstraint& rhs);

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const size_t contactPointIndex_;
};

}  // namespace ocs2::legged_robot

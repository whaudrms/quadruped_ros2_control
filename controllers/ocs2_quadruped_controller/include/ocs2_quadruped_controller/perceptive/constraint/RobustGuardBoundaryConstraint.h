// Robust phase guard boundary constraint.
//
// Returns the scalar guard value g(x) - target_at(t) where:
//   g(x)        = n_l^T (p_foot,l(q) - p_plane,l)
//   target_at(t) = +d  if t is closer to t_a (window start)
//                  -d  if t is closer to t_b (window end)
//
// Active only at shooting nodes within dt_mpc/2 of t_a or t_b for the leg's robust
// window (queried from PerceptiveLeggedReferenceManager via getRobustWindow / isInRobustWindow).
//
// Wrapped by StateSoftConstraint + QuadraticPenalty(2*w_boundary) to produce the soft
// equality cost L = w_boundary * (g - target)^2.
//
// Per the M1'' plan: skip the t_a boundary entirely when the window was clamped
// (skip_t_a_boundary == true) — only the t_b boundary is then enforced.

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot {

class RobustGuardBoundaryConstraint final : public StateConstraint {
public:
    RobustGuardBoundaryConstraint(const SwitchedModelReferenceManager& referenceManager,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  size_t contactPointIndex);

    ~RobustGuardBoundaryConstraint() override = default;
    RobustGuardBoundaryConstraint* clone() const override { return new RobustGuardBoundaryConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const PreComputation& preComp) const override;

private:
    RobustGuardBoundaryConstraint(const RobustGuardBoundaryConstraint& rhs);

    // Returns +d (near t_a, if !skip_t_a_boundary) or -d (near t_b). Caller must check isActive first.
    static scalar_t targetAt(scalar_t time, const RobustWindowData& w);

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const size_t contactPointIndex_;
};

}  // namespace ocs2::legged_robot

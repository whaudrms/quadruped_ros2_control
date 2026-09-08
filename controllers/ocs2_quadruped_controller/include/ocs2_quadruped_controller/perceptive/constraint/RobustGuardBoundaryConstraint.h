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
// In EqualityResidual mode this is wrapped by StateSoftConstraint +
// QuadraticPenalty(2*w_boundary) to produce the soft equality cost
// L = w_boundary * (g - target)^2.
//
// In TraversalInequality mode the returned h(x) follows OCS2's h >= 0 convention:
//   near t_a: h =  g - d >= 0
//   near t_b: h = -g - d >= 0
// EndpointSelection allows the hard start and end inequalities to be registered
// independently. A separate EqualityResidual instance always keeps the boundary
// cost active at both endpoints.
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
    enum class Formulation {
        EqualityResidual,
        TraversalInequality,
    };

    enum class EndpointSelection {
        Both,
        Start,
        End,
    };

    RobustGuardBoundaryConstraint(const SwitchedModelReferenceManager& referenceManager,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  size_t contactPointIndex,
                                  Formulation formulation = Formulation::EqualityResidual,
                                  EndpointSelection endpointSelection = EndpointSelection::Both);

    ~RobustGuardBoundaryConstraint() override = default;
    RobustGuardBoundaryConstraint* clone() const override { return new RobustGuardBoundaryConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const PreComputation& preComp) const override;

private:
    RobustGuardBoundaryConstraint(const RobustGuardBoundaryConstraint& rhs);

    // Returns the target selected for this instance. Caller must check isActive first.
    scalar_t targetAt(scalar_t time, const RobustWindowData& w) const;

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const size_t contactPointIndex_;
    const Formulation formulation_;
    const EndpointSelection endpointSelection_;
};

}  // namespace ocs2::legged_robot

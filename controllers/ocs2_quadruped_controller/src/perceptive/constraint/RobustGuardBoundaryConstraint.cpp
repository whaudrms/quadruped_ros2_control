#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h"

#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot {

RobustGuardBoundaryConstraint::RobustGuardBoundaryConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex,
    Formulation formulation,
    EndpointSelection endpointSelection)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      formulation_(formulation),
      endpointSelection_(endpointSelection) {}

RobustGuardBoundaryConstraint::RobustGuardBoundaryConstraint(const RobustGuardBoundaryConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      formulation_(rhs.formulation_),
      endpointSelection_(rhs.endpointSelection_) {}

bool RobustGuardBoundaryConstraint::isActive(scalar_t time) const {
    const auto& w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    if (!w.active) return false;

    const scalar_t halfDt = 0.5 * w.dt_mpc;
    const bool enableTa = endpointSelection_ != EndpointSelection::End;
    const bool enableTb = endpointSelection_ != EndpointSelection::Start;
    const bool nearTb = enableTb && std::abs(time - w.t_b) < halfDt;
    const bool nearTa = enableTa && !w.skip_t_a_boundary && std::abs(time - w.t_a) < halfDt;
    return nearTa || nearTb;
}

scalar_t RobustGuardBoundaryConstraint::targetAt(scalar_t time, const RobustWindowData& w) const {
    if (endpointSelection_ == EndpointSelection::Start) {
        return +w.d;
    }
    if (endpointSelection_ == EndpointSelection::End) {
        return -w.d;
    }
    if (w.skip_t_a_boundary) {
        return -w.d;  // only t_b boundary is active
    }
    const scalar_t distToTa = std::abs(time - w.t_a);
    const scalar_t distToTb = std::abs(time - w.t_b);
    return (distToTa < distToTb) ? +w.d : -w.d;
}

// g(x) is defined on the CONTACT POINT, not the URDF foot frame:
//   p_contact = p_foot - foot_frame_offset · n
//   g(x)      = n · (p_contact - p_plane) = n · (p_foot - p_plane) - foot_frame_offset
// So g = 0 means "contact point on the terrain plane" regardless of how high the foot
// frame sits above the contact (the FK foot frame for go2 is at the ankle, ~6 cm above).
// The Jacobian is unchanged since foot_frame_offset is a constant.

vector_t RobustGuardBoundaryConstraint::getValue(scalar_t time, const vector_t& state,
                                                 const PreComputation& /*preComp*/) const {
    const RobustWindowData w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const vector3_t p_foot = endEffectorKinematicsPtr_->getPosition(state).front();
    const scalar_t g = w.n.dot(p_foot - w.p_plane) - w.foot_frame_offset;
    const scalar_t target = targetAt(time, w);
    scalar_t residual = g - target;
    if (formulation_ == Formulation::TraversalInequality && target < 0.0) {
        residual = -residual;  // near t_b: -(g - (-d)) = -g - d >= 0
    }
    vector_t value(1);
    value(0) = residual;
    return value;
}

VectorFunctionLinearApproximation RobustGuardBoundaryConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& /*preComp*/) const {
    const RobustWindowData w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    const scalar_t target = targetAt(time, w);

    VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
    const scalar_t sign =
        (formulation_ == Formulation::TraversalInequality && target < 0.0) ? -1.0 : 1.0;
    approx.f(0) = sign * (w.n.dot(positionApprox.f - w.p_plane) - w.foot_frame_offset - target);
    approx.dfdx = sign * w.n.transpose() * positionApprox.dfdx;  // 1 x nx
    return approx;
}

}  // namespace ocs2::legged_robot

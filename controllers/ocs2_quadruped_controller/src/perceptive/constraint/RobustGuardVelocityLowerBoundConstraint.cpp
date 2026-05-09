#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardVelocityLowerBoundConstraint.h"

namespace ocs2::legged_robot {

RobustGuardVelocityLowerBoundConstraint::RobustGuardVelocityLowerBoundConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

RobustGuardVelocityLowerBoundConstraint::RobustGuardVelocityLowerBoundConstraint(
    const RobustGuardVelocityLowerBoundConstraint& rhs)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

bool RobustGuardVelocityLowerBoundConstraint::isActive(scalar_t time) const {
    return referenceManagerPtr_->isInRobustWindow(contactPointIndex_, time);
}

vector_t RobustGuardVelocityLowerBoundConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& /*preComp*/) const {
    const RobustWindowData w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const vector3_t v_foot = endEffectorKinematicsPtr_->getVelocity(state, input).front();
    const scalar_t g_dot = w.n.dot(v_foot);
    vector_t value(1);
    value(0) = g_dot + w.v_max;  // ġ + v_max ≥ 0  ⇔  ġ ≥ -v_max  (impact-velocity floor)
    return value;
}

VectorFunctionLinearApproximation RobustGuardVelocityLowerBoundConstraint::getLinearApproximation(
    scalar_t /*time*/, const vector_t& state, const vector_t& input,
    const PreComputation& /*preComp*/) const {
    const RobustWindowData w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();

    VectorFunctionLinearApproximation approx =
        VectorFunctionLinearApproximation::Zero(1, state.size(), input.size());
    approx.f(0) = w.n.dot(velocityApprox.f) + w.v_max;
    approx.dfdx = w.n.transpose() * velocityApprox.dfdx;  // 1 x nx
    approx.dfdu = w.n.transpose() * velocityApprox.dfdu;  // 1 x nu
    return approx;
}

}  // namespace ocs2::legged_robot

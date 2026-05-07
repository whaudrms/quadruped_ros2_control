#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h"

#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot {

RobustGuardBoundaryConstraint::RobustGuardBoundaryConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

RobustGuardBoundaryConstraint::RobustGuardBoundaryConstraint(const RobustGuardBoundaryConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

bool RobustGuardBoundaryConstraint::isActive(scalar_t time) const {
    const auto& w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    if (!w.active) return false;

    const scalar_t halfDt = 0.5 * w.dt_mpc;
    const bool nearTb = std::abs(time - w.t_b) < halfDt;
    const bool nearTa = !w.skip_t_a_boundary && std::abs(time - w.t_a) < halfDt;
    return nearTa || nearTb;
}

scalar_t RobustGuardBoundaryConstraint::targetAt(scalar_t time, const RobustWindowData& w) {
    if (w.skip_t_a_boundary) {
        return -w.d;  // only t_b boundary is active
    }
    const scalar_t distToTa = std::abs(time - w.t_a);
    const scalar_t distToTb = std::abs(time - w.t_b);
    return (distToTa < distToTb) ? +w.d : -w.d;
}

vector_t RobustGuardBoundaryConstraint::getValue(scalar_t time, const vector_t& state,
                                                 const PreComputation& /*preComp*/) const {
    const auto& w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const vector3_t p_foot = endEffectorKinematicsPtr_->getPosition(state).front();
    const scalar_t g = w.n.dot(p_foot - w.p_plane);
    const scalar_t target = targetAt(time, w);
    vector_t value(1);
    value(0) = g - target;
    return value;
}

VectorFunctionLinearApproximation RobustGuardBoundaryConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& /*preComp*/) const {
    const auto& w = referenceManagerPtr_->getRobustWindow(contactPointIndex_);
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    const scalar_t target = targetAt(time, w);

    VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
    approx.f(0) = w.n.dot(positionApprox.f - w.p_plane) - target;
    approx.dfdx = w.n.transpose() * positionApprox.dfdx;  // 1 x nx
    return approx;
}

}  // namespace ocs2::legged_robot

#include <ocs2_quadruped_controller/perceptive/cost/SwingFootTrackingCost.h>
#include <stdexcept>
namespace ocs2::legged_robot {
SwingFootTrackingCost::SwingFootTrackingCost(const SwitchedModelReferenceManager& manager,
    const SwingTrajectoryPlanner& planner, const EndEffectorKinematics<scalar_t>& kinematics, size_t leg)
    : manager_(&manager), planner_(&planner), kinematics_(kinematics.clone()), leg_(leg) {
    if (kinematics.getIds().size() != 1) throw std::invalid_argument("Swing tracking expects one foot");
}
SwingFootTrackingCost::SwingFootTrackingCost(const SwingFootTrackingCost& other)
    : StateInputCost(other), manager_(other.manager_), planner_(other.planner_),
      kinematics_(other.kinematics_->clone()), leg_(other.leg_) {}
bool SwingFootTrackingCost::isActive(scalar_t time) const {
    return planner_->config().terrainAware && !manager_->getContactFlags(time)[leg_] &&
           planner_->getTerrainSwing(leg_, time) != nullptr;
}
matrix3_t SwingFootTrackingCost::projection(scalar_t time) const {
    if (manager_->isInRobustWindow(leg_, time)) {
        const vector3_t normal = manager_->getRobustWindow(leg_).n.normalized();
        return matrix3_t::Identity() - normal * normal.transpose();
    }
    return matrix3_t::Identity();
}
scalar_t SwingFootTrackingCost::getValue(scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories&, const PreComputation&) const {
    if (!isActive(time)) return 0.0;
    const auto& reference = planner_->getTerrainSwing(leg_, time)->spline;
    const matrix3_t project = projection(time);
    const vector3_t ep = project * (kinematics_->getPosition(state).front() - reference.position(time));
    const vector3_t ev = project * (kinematics_->getVelocity(state, input).front() - reference.velocity(time));
    return 0.5 * (planner_->config().positionWeight * ep.squaredNorm() + planner_->config().velocityWeight * ev.squaredNorm());
}
ScalarFunctionQuadraticApproximation SwingFootTrackingCost::getQuadraticApproximation(scalar_t time,
    const vector_t& state, const vector_t& input, const TargetTrajectories&, const PreComputation&) const {
    auto result = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
    if (!isActive(time)) return result;
    const auto& reference = planner_->getTerrainSwing(leg_, time)->spline;
    const matrix3_t project = projection(time);
    const auto p = kinematics_->getPositionLinearApproximation(state).front();
    const auto v = kinematics_->getVelocityLinearApproximation(state, input).front();
    const scalar_t wp = planner_->config().positionWeight, wv = planner_->config().velocityWeight;
    const vector3_t ep = project * (p.f - reference.position(time));
    const vector3_t ev = project * (v.f - reference.velocity(time));
    const matrix_t px = project * p.dfdx, vx = project * v.dfdx, vu = project * v.dfdu;
    result.f = 0.5 * (wp * ep.squaredNorm() + wv * ev.squaredNorm());
    result.dfdx = wp * px.transpose() * ep + wv * vx.transpose() * ev;
    result.dfdu = wv * vu.transpose() * ev;
    // Gauss-Newton, as in the original least-squares tracking cost.
    result.dfdxx = wp * px.transpose() * px + wv * vx.transpose() * vx;
    result.dfduu = wv * vu.transpose() * vu;
    result.dfdux = wv * vu.transpose() * vx;
    return result;
}
} // namespace ocs2::legged_robot

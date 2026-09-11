#pragma once
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>
namespace ocs2::legged_robot {
// ANYmal MotionTrackingCost foot terms: 1/2 (wp*||p-pref||^2 + wv*||v-vref||^2).
// Robust phase keeps tangential tracking, leaving its normal motion to the guard.
class SwingFootTrackingCost final : public StateInputCost {
public:
    SwingFootTrackingCost(const SwitchedModelReferenceManager& manager, const SwingTrajectoryPlanner& planner,
                          const EndEffectorKinematics<scalar_t>& kinematics, size_t leg);
    SwingFootTrackingCost(const SwingFootTrackingCost& other);
    SwingFootTrackingCost* clone() const override { return new SwingFootTrackingCost(*this); }
    bool isActive(scalar_t time) const override;
    scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                     const TargetTrajectories&, const PreComputation&) const override;
    ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
        const vector_t& input, const TargetTrajectories&, const PreComputation&) const override;
private:
    matrix3_t projection(scalar_t time) const;
    const SwitchedModelReferenceManager* manager_;
    const SwingTrajectoryPlanner* planner_;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematics_;
    size_t leg_;
};
} // namespace ocs2::legged_robot

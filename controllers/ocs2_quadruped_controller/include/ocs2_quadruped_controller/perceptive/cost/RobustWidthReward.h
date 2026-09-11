#pragma once

#include <cmath>
#include <stdexcept>
#include <ocs2_core/cost/StateCost.h>
#include "ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h"

namespace ocs2::legged_robot {

// Running cost -w_d*d_i on [t_a,t_b). SQP supplies the integration timestep.
// The linear reward has an exact gradient and zero Hessian; it must not be
// represented by a squared residual, which would favor a different width.
class RobustWidthReward final : public StateCost {
 public:
    RobustWidthReward(const SwitchedModelReferenceManager& manager, size_t leg, scalar_t weight)
        : manager_(&manager), leg_(leg), weight_(weight) {
        if (!std::isfinite(weight_) || weight_ < 0.0)
            throw std::invalid_argument("robustPhase.w_d must be finite and nonnegative");
    }

    RobustWidthReward* clone() const override { return new RobustWidthReward(*this); }

    bool isActive(scalar_t time) const override {
        return active(manager_->getRobustWindow(leg_), time);
    }

    scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                      const PreComputation&) const override {
        const auto window = manager_->getRobustWindow(leg_);
        return active(window, time) ? -weight_ * state(window.d_state_index) : 0.0;
    }

    ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation&) const override {
        auto approximation = ScalarFunctionQuadraticApproximation::Zero(state.size(), 0);
        const auto window = manager_->getRobustWindow(leg_);
        if (active(window, time)) {
            approximation.f = -weight_ * state(window.d_state_index);
            approximation.dfdx(window.d_state_index) = -weight_;
        }
        return approximation;
    }

 private:
    bool active(const RobustWindowData& window, scalar_t time) const {
        return weight_ > 0.0 && window.active && window.d_state_index >= 0 &&
               time >= window.t_a && time < window.t_b;
    }

    const SwitchedModelReferenceManager* manager_;
    size_t leg_;
    scalar_t weight_;
};
}  // namespace ocs2::legged_robot

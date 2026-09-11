#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot {

// One bounded, constant width per leg and solve. A leg without an unfinished
// robust window has no boundary cost; its width remains bounded but is unused.
class RobustWidthBounds final : public StateConstraint {
 public:
    RobustWidthBounds(const PerceptiveLeggedReferenceManager& manager, size_t physicalDim)
        : StateConstraint(ConstraintOrder::Linear), manager_(&manager), physicalDim_(physicalDim) {}
    RobustWidthBounds* clone() const override { return new RobustWidthBounds(*this); }
    size_t getNumConstraints(scalar_t) const override { return 8; }
    vector_t getValue(scalar_t, const vector_t& state, const PreComputation&) const override {
        const auto& settings = manager_->getRobustPhaseSettings();
        vector_t value(8);
        for (size_t leg = 0; leg < 4; ++leg) {
            value(2 * leg) = state(physicalDim_ + leg) - settings.d_min;
            value(2 * leg + 1) = settings.d_max - state(physicalDim_ + leg);
        }
        return value;
    }
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state, const PreComputation& pc) const override {
        auto approx = VectorFunctionLinearApproximation::Zero(8, state.size(), 0);
        approx.f = getValue(time, state, pc);
        for (size_t leg = 0; leg < 4; ++leg) {
            approx.dfdx(2 * leg, physicalDim_ + leg) = 1.0;
            approx.dfdx(2 * leg + 1, physicalDim_ + leg) = -1.0;
        }
        return approx;
    }
 private:
    const PerceptiveLeggedReferenceManager* manager_;
    size_t physicalDim_;
};
}  // namespace ocs2::legged_robot

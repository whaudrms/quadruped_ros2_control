//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                     size_t contactPointIndex,
                                                     size_t numVertices)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          contactPointIndex_(contactPointIndex),
          numVertices_(numVertices)
    {
    }

    FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_),
          numVertices_(rhs.numVertices_)
    {
    }

    bool FootPlacementConstraint::isActive(scalar_t time) const
    {
        const auto* referenceManager =
            dynamic_cast<const PerceptiveLeggedReferenceManager*>(referenceManagerPtr_);
        if (referenceManager == nullptr)
        {
            return false;
        }
        const auto flags = referenceManager->getFootPlacementFlags(time);
        if (contactPointIndex_ >= flags.size())
        {
            return false;
        }
        return flags[contactPointIndex_];
    }

    vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        if (!isActive(time))
        {
            return vector_t::Zero(numVertices_);
        }
        const auto* perceptivePreComp = dynamic_cast<const PerceptiveLeggedPrecomputation*>(&preComp);
        if (perceptivePreComp == nullptr)
        {
            return vector_t::Zero(numVertices_);
        }
        const auto& params = perceptivePreComp->getFootPlacementConParameters();
        if (contactPointIndex_ >= params.size())
        {
            return vector_t::Zero(numVertices_);
        }
        const auto& param = params[contactPointIndex_];
        if (param.a.rows() == 0 || param.b.size() == 0)
        {
            return vector_t::Zero(numVertices_);
        }
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);
        if (contactPointIndex_ >= positions.size())
        {
            return vector_t::Zero(numVertices_);
        }
        return param.a * positions[contactPointIndex_] + param.b;
    }

    VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numVertices_, state.size(), 0);
        if (!isActive(time))
        {
            return approx;
        }
        const auto* perceptivePreComp = dynamic_cast<const PerceptiveLeggedPrecomputation*>(&preComp);
        if (perceptivePreComp == nullptr)
        {
            return approx;
        }
        const auto& params = perceptivePreComp->getFootPlacementConParameters();
        if (contactPointIndex_ >= params.size())
        {
            return approx;
        }
        const auto& param = params[contactPointIndex_];
        if (param.a.rows() == 0 || param.b.size() == 0)
        {
            return approx;
        }

        const auto positionApproxs = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);
        if (contactPointIndex_ >= positionApproxs.size())
        {
            return approx;
        }
        const auto& positionApprox = positionApproxs[contactPointIndex_];
        approx.f = param.a * positionApprox.f + param.b;
        approx.dfdx = param.a * positionApprox.dfdx;
        return approx;
    }
} // namespace legged

#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2::legged_robot {
// Append constant auxiliary states without changing physical-model dimensions.
void appendConstantParameters(OptimalControlProblem& problem, size_t physicalDim, size_t parameterDim);
std::unique_ptr<Initializer> parameterInitializer(const Initializer& physical, size_t physicalDim);
std::unique_ptr<EndEffectorKinematics<scalar_t>> parameterKinematics(
    const EndEffectorKinematics<scalar_t>& physical, size_t physicalDim);
}  // namespace ocs2::legged_robot

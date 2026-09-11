//
// Created by biao on 3/21/25.
//

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardApproachConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardVelocityLowerBoundConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/penalties/penalties/QuadraticPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>
#include <ocs2_quadruped_controller/perceptive/cost/SwingFootTrackingCost.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include "ocs2_quadruped_controller/perceptive/interface/ConstantParameterOcp.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustWidthBounds.h"
#include "ocs2_quadruped_controller/perceptive/cost/RobustWidthReward.h"

namespace ocs2::legged_robot
{
    void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                               const std::string& referenceFile, bool verbose)
    {
        planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
        terrainDataMutex_ = std::make_shared<std::mutex>();

        double width{5.0}, height{5.0};
        convex_plane_decomposition::PlanarRegion plannerRegion;
        plannerRegion.transformPlaneToWorld.setIdentity();
        plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2);
        convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));
        plannerRegion.boundaryWithInset.boundary = boundary;
        convex_plane_decomposition::CgalPolygonWithHoles2d insets;
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, -width / 2 + 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, -width / 2 + 0.01));
        plannerRegion.boundaryWithInset.insets.push_back(insets);
        planarTerrainPtr_->planarRegions.push_back(plannerRegion);

        constexpr const char* layer = "elevation";
        planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
        planarTerrainPtr_->gridMap.add(layer, 0);
        planarTerrainPtr_->gridMap.add("smooth_planar", 0);
        signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
        signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, layer, 0.1);

        LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

        // Load robust phase settings from task.info and apply to the perceptive reference manager.
        // dt_mpc is overridden with the actual SQP shooting interval so that
        // RobustGuardBoundaryConstraint::isActive uses the correct dt/2 tolerance.
        auto& perceptiveRefManager = dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_);
        auto robustSettings = loadRobustPhaseSettings(taskFile, verbose);
        robustSettings.dt_mpc = sqp_settings_.dt;
        perceptiveRefManager.setRobustPhaseSettings(robustSettings);
        robustSettings = perceptiveRefManager.getRobustPhaseSettings();

        // Robust phase soft penalty weights (separate from the constraint settings, since they
        // parameterize how strongly the boundary equality and ġ² cost are enforced).
        scalar_t w_boundary = 100.0;
        scalar_t w_v = 1.0;
        // Missing w_d preserves older task files; malformed weights must fail
        // explicitly rather than being swallowed by the legacy penalty loader.
        boost::property_tree::ptree rewardSettings;
        boost::property_tree::read_info(taskFile, rewardSettings);
        const scalar_t w_d = rewardSettings.get<scalar_t>("robustPhase.w_d", 0.0);
        if (!std::isfinite(w_d) || w_d < 0.0)
            throw std::invalid_argument("robustPhase.w_d must be finite and nonnegative");
        RelaxedBarrierPenalty::Config approachBarrierConfig(1e-2, 1e-3);
        if (robustSettings.enabled)
        {
            try
            {
                boost::property_tree::ptree pt;
                boost::property_tree::read_info(taskFile, pt);
                loadData::loadPtreeValue(pt, w_boundary,                  "robustPhase.w_boundary",          verbose);
                loadData::loadPtreeValue(pt, w_v,                         "robustPhase.w_v",                 verbose);
                loadData::loadPtreeValue(pt, approachBarrierConfig.mu,    "robustPhase.approach_barrier_mu", verbose);
                loadData::loadPtreeValue(pt, approachBarrierConfig.delta, "robustPhase.approach_barrier_delta", verbose);
            }
            catch (const std::exception&)
            {
                // Defaults already set above.
            }
        }

        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = modelSettings().contactNames3DoF[i];
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

            std::unique_ptr<PenaltyBase> placementPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
            std::unique_ptr<PenaltyBase> collisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

            if (reference_manager_ptr_->getSwingTrajectoryPlanner()->config().terrainAware) {
                // Register before auxiliary d states are appended so the physical-state
                // adapter also wraps this cost and pads its derivatives for SQP.
                problem_ptr_->costPtr->add(footName + "_swingFootTracking",
                    std::make_unique<SwingFootTrackingCost>(*reference_manager_ptr_,
                        *reference_manager_ptr_->getSwingTrajectoryPlanner(), *eeKinematicsPtr, i));
            }

            // For foot placement Soft Constraint
            if (enableFootPlacementConstraint_)
            {
                std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
                    new FootPlacementConstraint(*reference_manager_ptr_, *eeKinematicsPtr, i, numVertices_));
                problem_ptr_->stateSoftConstraintPtr->add(
                    footName + "_footPlacement",
                    std::make_unique<StateSoftConstraint>(std::move(footPlacementConstraint), std::move(placementPenalty)));
            }

            // For foot Collision Soft Constraint
            if (enableFootCollisionConstraint_)
            {
                std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
                    new FootCollisionConstraint(*reference_manager_ptr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i,
                                                footCollisionClearance_));
                problem_ptr_->stateSoftConstraintPtr->add(
                    footName + "_footCollision",
                    std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
            }


        }

        // For collision avoidance Soft Constraint
        scalar_t calfExcess = 0.02;

        std::vector<std::string> collisionLinks = {"FL_calf", "FR_calf", "RL_calf", "RR_calf"};
        const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess, calfExcess, calfExcess};

        pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
            *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6);

        CentroidalModelPinocchioMapping pinocchioMapping(centroidal_model_info_);
        auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
            *pinocchioSphereInterfacePtr_, pinocchioMapping);

        if (enableBodyCollisionConstraint_)
        {
            std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
                new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));
            std::unique_ptr<PenaltyBase> bodyCollisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
            problem_ptr_->stateSoftConstraintPtr->add(
                "sdfConstraint",
                std::make_unique<StateSoftConstraint>(std::move(sphereSdfConstraint), std::move(bodyCollisionPenalty)));
        }
        if (robustSettings.enabled && robustSettings.optimize_d) {
            const auto nx = centroidal_model_info_.stateDim;
            appendConstantParameters(*problem_ptr_, nx, centroidal_model_info_.numThreeDofContacts);
            initializer_ptr_ = parameterInitializer(*initializer_ptr_, nx);
            rollout_ptr_ = std::make_unique<TimeTriggeredRollout>(*problem_ptr_->dynamicsPtr, rollout_settings_);
            problem_ptr_->stateInequalityConstraintPtr->add("robustWidthBounds",
                std::make_unique<RobustWidthBounds>(perceptiveRefManager, nx));
            problem_ptr_->finalInequalityConstraintPtr->add("robustWidthBounds",
                std::make_unique<RobustWidthBounds>(perceptiveRefManager, nx));
            for (size_t leg = 0; leg < centroidal_model_info_.numThreeDofContacts; ++leg) {
                problem_ptr_->stateCostPtr->add(modelSettings().contactNames3DoF[leg] + "_robustWidthReward",
                    std::make_unique<RobustWidthReward>(perceptiveRefManager, leg, w_d));
            }
        }
        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; ++i) {
            const auto& footName = modelSettings().contactNames3DoF[i];
            auto eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);
            if (robustSettings.enabled && robustSettings.optimize_d)
                eeKinematicsPtr = parameterKinematics(*eeKinematicsPtr, centroidal_model_info_.stateDim);
            // Robust phase boundary cost plus optional hard endpoint constraints,
            // followed by the soft velocity envelope/rate cost.
            if (robustSettings.enabled)
            {
                // (1) Boundary cost is always retained:
                //     g(x_a)=+d, g(x_b)=-d via QuadraticPenalty(2*w_boundary).
                problem_ptr_->stateSoftConstraintPtr->add(
                    footName + "_robustGuardBoundary",
                    std::make_unique<StateSoftConstraint>(
                        std::make_unique<RobustGuardBoundaryConstraint>(
                            *reference_manager_ptr_, *eeKinematicsPtr, i),
                        std::make_unique<QuadraticPenalty>(2.0 * w_boundary)));

                if (robustSettings.hard_boundary_start)
                {
                    // (1a) Optional hard start boundary. This does not replace
                    // the boundary cost above, which stays active at t_a/t_b.
                    problem_ptr_->stateInequalityConstraintPtr->add(
                        footName + "_robustGuardBoundaryHardStart",
                        std::make_unique<RobustGuardBoundaryConstraint>(
                            *reference_manager_ptr_, *eeKinematicsPtr, i,
                            RobustGuardBoundaryConstraint::Formulation::TraversalInequality,
                            RobustGuardBoundaryConstraint::EndpointSelection::Start));
                }

                if (robustSettings.hard_boundary_end)
                {
                    // (1b) Optional hard end boundary. Disabled by default so
                    // physical contact is not forced below the terrain plane.
                    problem_ptr_->stateInequalityConstraintPtr->add(
                        footName + "_robustGuardBoundaryHardEnd",
                        std::make_unique<RobustGuardBoundaryConstraint>(
                            *reference_manager_ptr_, *eeKinematicsPtr, i,
                            RobustGuardBoundaryConstraint::Formulation::TraversalInequality,
                            RobustGuardBoundaryConstraint::EndpointSelection::End));
                }

                if (robustSettings.slack_boundary_start)
                {
                    // Analytically eliminated quadratic slack:
                    // min 0.5*mu*s^2, s>=0, g(t_a)-d+s>=0.
                    problem_ptr_->stateSoftConstraintPtr->add(
                        footName + "_robustGuardBoundarySlackStart",
                        std::make_unique<StateSoftConstraint>(
                            std::make_unique<RobustGuardBoundaryConstraint>(
                                *reference_manager_ptr_, *eeKinematicsPtr, i,
                                RobustGuardBoundaryConstraint::Formulation::TraversalInequality,
                                RobustGuardBoundaryConstraint::EndpointSelection::Start),
                            std::make_unique<SquaredHingePenalty>(
                                SquaredHingePenalty::Config{
                                    robustSettings.slack_boundary_weight_start, 0.0})));
                }

                if (robustSettings.slack_boundary_end)
                {
                    // Analytically eliminated quadratic slack:
                    // min 0.5*mu*s^2, s>=0, -g(t_b)-d+s>=0.
                    problem_ptr_->stateSoftConstraintPtr->add(
                        footName + "_robustGuardBoundarySlackEnd",
                        std::make_unique<StateSoftConstraint>(
                            std::make_unique<RobustGuardBoundaryConstraint>(
                                *reference_manager_ptr_, *eeKinematicsPtr, i,
                                RobustGuardBoundaryConstraint::Formulation::TraversalInequality,
                                RobustGuardBoundaryConstraint::EndpointSelection::End),
                            std::make_unique<SquaredHingePenalty>(
                                SquaredHingePenalty::Config{
                                    robustSettings.slack_boundary_weight_end, 0.0})));
                }

                // (2) Approach inequality -ġ ≥ 0 via RelaxedBarrierPenalty.
                problem_ptr_->softConstraintPtr->add(
                    footName + "_robustGuardApproach",
                    std::make_unique<StateInputSoftConstraint>(
                        std::make_unique<RobustGuardApproachConstraint>(*reference_manager_ptr_, *eeKinematicsPtr, i),
                        std::make_unique<RelaxedBarrierPenalty>(approachBarrierConfig)));

                // (3) ġ² running cost via QuadraticPenalty(2*w_v) on the same -ġ scalar (sign-symmetric).
                problem_ptr_->softConstraintPtr->add(
                    footName + "_robustGuardCost",
                    std::make_unique<StateInputSoftConstraint>(
                        std::make_unique<RobustGuardApproachConstraint>(*reference_manager_ptr_, *eeKinematicsPtr, i),
                        std::make_unique<QuadraticPenalty>(2.0 * w_v)));

                // (4) Impact-velocity lower bound  ġ + v_max ≥ 0  (i.e. ġ ≥ -v_max)
                // via RelaxedBarrierPenalty (same config as (2) approach inequality).
                // Together with (2) this forms the soft envelope -v_max ≤ ġ ≤ 0,
                // preventing the OCP from satisfying the boundary g(t_a)=+d, g(t_b)=-d
                // with arbitrarily large descent speed. Per chat7 priority #2.
                problem_ptr_->softConstraintPtr->add(
                    footName + "_robustGuardVelocityLowerBound",
                    std::make_unique<StateInputSoftConstraint>(
                        std::make_unique<RobustGuardVelocityLowerBoundConstraint>(
                            *reference_manager_ptr_, *eeKinematicsPtr, i),
                        std::make_unique<RelaxedBarrierPenalty>(approachBarrierConfig)));
            }
        }
    }

    // SwingTrajectoryPlanner, ConvexRegionSelector, PerceptiveLeggedReferenceManager
    void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                          const std::string& referenceFile, bool verbose)
    {
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");
        auto convexRegionSelector =
            std::make_unique<ConvexRegionSelector>(centroidal_model_info_, planarTerrainPtr_, terrainDataMutex_,
                                                   *eeKinematicsPtr,
                                                   numVertices_);

        scalar_t comHeight = 0;
        loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
        reference_manager_ptr_.reset(new PerceptiveLeggedReferenceManager(
            centroidal_model_info_, loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
            *eeKinematicsPtr, comHeight));
        dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_).setEnableReferenceModification(
            enableReferenceModification_);
    }

    void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/,
                                                        const std::string& /*urdfFile*/,
                                                        const std::string& /*referenceFile*/, bool /*verbose*/)
    {
        problem_ptr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_,
            *dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_).getConvexRegionSelectorPtr(),
            footPlacementBoundaryMargin_);
    }
} // namespace legged

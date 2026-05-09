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
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <memory>

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

        // Robust phase soft penalty weights (separate from the constraint settings, since they
        // parameterize how strongly the boundary equality and ġ² cost are enforced).
        scalar_t w_boundary = 100.0;
        scalar_t w_v = 1.0;
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
                                                0.01));
                problem_ptr_->stateSoftConstraintPtr->add(
                    footName + "_footCollision",
                    std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
            }

            // Robust phase: 3 entries per leg (boundary equality, approach inequality, ġ² cost).
            if (robustSettings.enabled)
            {
                // (1) Boundary g(x_a)=+d, g(x_b)=-d via QuadraticPenalty(2*w_boundary).
                problem_ptr_->stateSoftConstraintPtr->add(
                    footName + "_robustGuardBoundary",
                    std::make_unique<StateSoftConstraint>(
                        std::make_unique<RobustGuardBoundaryConstraint>(*reference_manager_ptr_, *eeKinematicsPtr, i),
                        std::make_unique<QuadraticPenalty>(2.0 * w_boundary)));

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

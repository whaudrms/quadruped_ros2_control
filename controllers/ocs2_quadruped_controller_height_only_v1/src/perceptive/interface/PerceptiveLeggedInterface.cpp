//
// Created by biao on 3/21/25.
//

#include <ocs2_core/misc/LoadData.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <memory>

namespace ocs2::legged_robot
{
    void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                               const std::string& referenceFile, bool verbose)
    {
        planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

        double floorSizeX = 5.0;
        double floorSizeY = 5.0;
        double boxCenterX = 0.45;
        double boxCenterY = 0.0;
        double boxSizeX = 0.36;
        double boxSizeY = 1.10;
        double boxHeight = 0.06;
        const double insetMargin = 0.01;

        auto tryLoad = [&](const std::string& key, double& value) {
            try
            {
                loadData::loadCppDataType(taskFile, key, value);
            }
            catch (...)
            {
            }
        };
        tryLoad("manual_planar_terrain.floor_size_x", floorSizeX);
        tryLoad("manual_planar_terrain.floor_size_y", floorSizeY);
        tryLoad("manual_planar_terrain.box_center_x", boxCenterX);
        tryLoad("manual_planar_terrain.box_center_y", boxCenterY);
        tryLoad("manual_planar_terrain.box_size_x", boxSizeX);
        tryLoad("manual_planar_terrain.box_size_y", boxSizeY);
        tryLoad("manual_planar_terrain.box_height", boxHeight);

        auto makeRectRegion = [&](double centerX, double centerY, double centerZ, double sizeX, double sizeY) {
            convex_plane_decomposition::PlanarRegion region;
            region.transformPlaneToWorld.setIdentity();
            region.transformPlaneToWorld.translation() = Eigen::Vector3d(centerX, centerY, centerZ);
            region.bbox2d =
                convex_plane_decomposition::CgalBbox2d(-sizeX / 2, -sizeY / 2, +sizeX / 2, +sizeY / 2);

            convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
            boundary.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(+sizeX / 2, +sizeY / 2));
            boundary.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(-sizeX / 2, +sizeY / 2));
            boundary.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(-sizeX / 2, -sizeY / 2));
            boundary.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(+sizeX / 2, -sizeY / 2));
            region.boundaryWithInset.boundary = boundary;

            convex_plane_decomposition::CgalPolygonWithHoles2d inset;
            inset.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(+sizeX / 2 - insetMargin, +sizeY / 2 - insetMargin));
            inset.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(-sizeX / 2 + insetMargin, +sizeY / 2 - insetMargin));
            inset.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(-sizeX / 2 + insetMargin, -sizeY / 2 + insetMargin));
            inset.outer_boundary().push_back(
                convex_plane_decomposition::CgalPoint2d(+sizeX / 2 - insetMargin, -sizeY / 2 + insetMargin));
            region.boundaryWithInset.insets.push_back(inset);
            return region;
        };

        planarTerrainPtr_->planarRegions.push_back(makeRectRegion(0.0, 0.0, 0.0, floorSizeX, floorSizeY));
        if (boxHeight > 1e-6 && boxSizeX > 1e-6 && boxSizeY > 1e-6)
        {
            planarTerrainPtr_->planarRegions.push_back(
                makeRectRegion(boxCenterX, boxCenterY, boxHeight, boxSizeX, boxSizeY));
        }

        std::string layer = "elevation_before_postprocess";
        planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
        planarTerrainPtr_->gridMap.add(layer, 0);
        planarTerrainPtr_->gridMap.add("smooth_planar", 0);
        const double boxMinX = boxCenterX - 0.5 * boxSizeX;
        const double boxMaxX = boxCenterX + 0.5 * boxSizeX;
        const double boxMinY = boxCenterY - 0.5 * boxSizeY;
        const double boxMaxY = boxCenterY + 0.5 * boxSizeY;
        for (grid_map::GridMapIterator it(planarTerrainPtr_->gridMap); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);
            grid_map::Position position;
            planarTerrainPtr_->gridMap.getPosition(index, position);
            if (boxHeight > 1e-6 && position.x() >= boxMinX && position.x() <= boxMaxX && position.y() >= boxMinY &&
                position.y() <= boxMaxY)
            {
                planarTerrainPtr_->gridMap.at(layer, index) = boxHeight;
                planarTerrainPtr_->gridMap.at("smooth_planar", index) = boxHeight;
            }
        }
        signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
        signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, layer, 0.1);

        LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = modelSettings().contactNames3DoF[i];
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

            std::unique_ptr<PenaltyBase> placementPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
            std::unique_ptr<PenaltyBase> collisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

            // Keep foot placement disabled for the terrain-stable GO2 perceptive baseline.
            // The upstream constraint path is still unstable during gait transitions on this stack.
            (void)placementPenalty;

            // For foot Collision
            std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
                new FootCollisionConstraint(*reference_manager_ptr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i,
                                            0.03));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footCollision",
                std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
        }

        // For collision avoidance
        scalar_t thighExcess = 0.025;
        scalar_t calfExcess = 0.02;

        std::vector<std::string> collisionLinks = {"LF_calf", "RF_calf", "LH_calf", "RH_calf"};
        const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess, calfExcess, calfExcess};

        pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
            *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6);

        CentroidalModelPinocchioMapping pinocchioMapping(centroidal_model_info_);
        auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
            *pinocchioSphereInterfacePtr_, pinocchioMapping);

        std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
            new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));

        //  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
        //  problem_ptr_->stateSoftConstraintPtr->add(
        //      "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
    }

    void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                          const std::string& referenceFile, bool verbose)
    {
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");
        auto convexRegionSelector =
            std::make_unique<ConvexRegionSelector>(centroidal_model_info_, planarTerrainPtr_, *eeKinematicsPtr,
                                                   numVertices_);

        scalar_t comHeight = 0;
        loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
        reference_manager_ptr_.reset(new PerceptiveLeggedReferenceManager(
            centroidal_model_info_, loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
            *eeKinematicsPtr, comHeight));
    }

    void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/,
                                                        const std::string& /*urdfFile*/,
                                                        const std::string& /*referenceFile*/, bool /*verbose*/)
    {
        problem_ptr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_,
            *dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_).getConvexRegionSelectorPtr());
    }
} // namespace legged

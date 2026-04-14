//
// Created by biao on 3/21/25.
//

#pragma once

#include <mutex>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace ocs2::legged_robot
{
    class PerceptiveLeggedInterface final : public LeggedInterface
    {
    public:
        using LeggedInterface::LeggedInterface;

        void setPerceptiveDebugOptions(bool enableReferenceModification,
                                       bool enableFootPlacementConstraint,
                                       bool enableFootCollisionConstraint,
                                       bool enableBodyCollisionConstraint,
                                       scalar_t footPlacementBoundaryMargin)
        {
            enableReferenceModification_ = enableReferenceModification;
            enableFootPlacementConstraint_ = enableFootPlacementConstraint;
            enableFootCollisionConstraint_ = enableFootCollisionConstraint;
            enableBodyCollisionConstraint_ = enableBodyCollisionConstraint;
            footPlacementBoundaryMargin_ = footPlacementBoundaryMargin;
        }

        void setupOptimalControlProblem(const std::string& taskFile,
                                        const std::string& urdfFile,
                                        const std::string& referenceFile,
                                        bool verbose) override;

        void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                   const std::string& referenceFile,
                                   bool verbose) override;

        void setupPreComputation(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile,
                                 bool verbose) override;

        std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const
        {
            return signedDistanceFieldPtr_;
        }

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const
        {
            return planarTerrainPtr_;
        }

        std::shared_ptr<std::mutex> getTerrainDataMutexPtr() const
        {
            return terrainDataMutex_;
        }

        std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const
        {
            return pinocchioSphereInterfacePtr_;
        }

        size_t getNumVertices() const { return numVertices_; }

    protected:
        size_t numVertices_ = 16;
        bool enableReferenceModification_ = true;
        bool enableFootPlacementConstraint_ = true;
        bool enableFootCollisionConstraint_ = true;
        bool enableBodyCollisionConstraint_ = false;
        scalar_t footPlacementBoundaryMargin_ = 0.05;

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
        std::shared_ptr<std::mutex> terrainDataMutex_;
        std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
    };
} // namespace ocs2::legged_robot

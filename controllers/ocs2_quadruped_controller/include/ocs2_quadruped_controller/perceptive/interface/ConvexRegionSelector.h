//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <ocs2_core/reference/ModeSchedule.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    class ConvexRegionSelector
    {
    public:
        ConvexRegionSelector(CentroidalModelInfo info,
                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> PlanarTerrainPtr,
                             std::shared_ptr<std::mutex> terrainDataMutexPtr,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numVertices);

        void update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                    const TargetTrajectories& targetTrajectories);

        convex_plane_decomposition::PlanarTerrainProjection getProjection(size_t leg, scalar_t time) const;

        convex_plane_decomposition::CgalPolygon2d getConvexPolygon(size_t leg, scalar_t time) const;

        vector3_t getNominalFootholds(size_t leg, scalar_t time) const;

        std::vector<scalar_t> getMiddleTimes(size_t leg) const { return middleTimes_[leg]; }

        std::vector<convex_plane_decomposition::PlanarTerrainProjection> getProjections(size_t leg)
        {
            return feetProjections_[leg];
        }

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() { return planarTerrainPtr_; }

        feet_array_t<scalar_t> getInitStandFinalTimes() const { return initStandFinalTime_; }

        feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

        size_t getNumVertices() const { return numVertices_; }

        std::optional<scalar_t> sampleTerrainHeight(scalar_t x, scalar_t y) const;

    private:
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

        vector3_t getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                     const TargetTrajectories& targetTrajectories);

        feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
        feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;

        feet_array_t<std::vector<vector3_t>> nominalFootholds_;
        feet_array_t<std::vector<scalar_t>> middleTimes_;

        feet_array_t<scalar_t> initStandFinalTime_;
        feet_array_t<bool> initStandFinalTimeLatched_;

        feet_array_t<std::vector<scalar_t>> timeEvents_;

        const CentroidalModelInfo info_;
        size_t numVertices_;

        convex_plane_decomposition::PlanarTerrain planarTerrain_;
        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<std::mutex> terrainDataMutexPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    };
} // namespace ocs2::legged_robot

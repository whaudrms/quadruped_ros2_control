//
// Created by biao on 3/21/25.
//

#include <ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <cstdio>

namespace ocs2::legged_robot
{
    namespace
    {
        constexpr const char* kUncertaintyLayer = "uncertainty";
        constexpr scalar_t kUncertaintyPenaltyWeight = 0.02;
        constexpr scalar_t kUncertaintyActivationDelay = 2.0;
        constexpr const char* kDebugFilePath = "/tmp/height_only_reference_debug.log";
    }

    ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                               std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                               planarTerrainPtr,
                                               const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                               size_t numVertices)
        : info_(std::move(info)),
          numVertices_(numVertices),
          planarTerrainPtr_(std::move(planarTerrainPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone())
    {
    }

    convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(
        size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return feetProjections_[leg][index];
    }

    convex_plane_decomposition::CgalPolygon2d ConvexRegionSelector::getConvexPolygon(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return convexPolygons_[leg][index];
    }

    vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return nominalFootholds_[leg][index];
    }

    scalar_t ConvexRegionSelector::getUncertaintyAtPosition(const vector3_t& positionInWorld) const
    {
        if (!planarTerrainPtr_ || !planarTerrainPtr_->gridMap.exists(kUncertaintyLayer))
        {
            return 0.0;
        }

        const grid_map::Position query(positionInWorld.x(), positionInWorld.y());
        if (!planarTerrainPtr_->gridMap.isInside(query))
        {
            return 0.0;
        }

        const auto value = planarTerrainPtr_->gridMap.atPosition(kUncertaintyLayer, query);
        return std::isfinite(value) ? std::max<scalar_t>(0.0, value) : 0.0;
    }

    void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                                      TargetTrajectories& targetTrajectories)
    {
        planarTerrain_ = *planarTerrainPtr_;
        // Need copy storage it since PlanarTerrainProjection.regionPtr is a pointer
        const auto& modeSequence = modeSchedule.modeSequence;
        const auto& eventTimes = modeSchedule.eventTimes;
        const auto contactFlagStocks = extractContactFlags(modeSequence);
        const size_t numPhases = modeSequence.size();

        // Find start and final index of time for legs
        feet_array_t<std::vector<int>> startIndices;
        feet_array_t<std::vector<int>> finalIndices;
        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            startIndices[leg] = std::vector<int>(numPhases, 0);
            finalIndices[leg] = std::vector<int>(numPhases, 0);
            // find the startTime and finalTime indices for swing feet
            for (size_t i = 0; i < numPhases; i++)
            {
                // skip if it is a stance leg
                if (contactFlagStocks[leg][i])
                {
                    std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            feetProjections_[leg].clear();
            convexPolygons_[leg].clear();
            nominalFootholds_[leg].clear();
            feetProjections_[leg].resize(numPhases);
            convexPolygons_[leg].resize(numPhases);
            nominalFootholds_[leg].resize(numPhases);
            middleTimes_[leg].clear();
            initStandFinalTime_[leg] = 0;

            scalar_t lastStandMiddleTime = NAN;
            // Stand leg foot
            for (size_t i = 0; i < numPhases; ++i)
            {
                if (contactFlagStocks[leg][i])
                {
                    const int standStartIndex = startIndices[leg][i];
                    const int standFinalIndex = finalIndices[leg][i];
                    const scalar_t standStartTime = eventTimes[standStartIndex];
                    const scalar_t standFinalTime = eventTimes[standFinalIndex];
                    const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2;

                    if (!numerics::almost_eq(standMiddleTime, lastStandMiddleTime))
                    {
                        vector3_t footPos = getNominalFoothold(leg, standMiddleTime, initState, targetTrajectories);
                        const bool useUncertaintyPenalty = standMiddleTime > (initTime + kUncertaintyActivationDelay);
                        auto penaltyFunction = [this, useUncertaintyPenalty](const vector3_t& projectedPoint)
                        {
                            if (!useUncertaintyPenalty)
                            {
                                return 0.0;
                            }
                            return kUncertaintyPenaltyWeight * getUncertaintyAtPosition(projectedPoint);
                        };
                        const auto projection = getBestPlanarRegionAtPositionInWorld(
                            footPos, planarTerrain_.planarRegions, penaltyFunction);
                        scalar_t growthFactor = 1.05;
                        const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                            projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame,
                            numVertices_, growthFactor);

                        feetProjections_[leg][i] = projection;
                        convexPolygons_[leg][i] = convexRegion;
                        nominalFootholds_[leg][i] = footPos;
                        middleTimes_[leg].push_back(standMiddleTime);

                        if (FILE* debugFile = std::fopen(kDebugFilePath, "a"))
                        {
                            const auto projected = projection.positionInWorld;
                            const scalar_t uncertainty = getUncertaintyAtPosition(projected);
                            std::fprintf(
                                debugFile,
                                "[HeightOnlySelector] init_t=%.3f stand_mid_t=%.3f leg=%zu nominal=(%.4f,%.4f,%.4f) projected=(%.4f,%.4f,%.4f) uncertainty=%.5f polygon_vertices=%zu\n",
                                static_cast<double>(initTime), static_cast<double>(standMiddleTime), leg,
                                static_cast<double>(footPos.x()), static_cast<double>(footPos.y()), static_cast<double>(footPos.z()),
                                static_cast<double>(projected.x()), static_cast<double>(projected.y()), static_cast<double>(projected.z()),
                                static_cast<double>(uncertainty), convexRegion.size());
                            std::fclose(debugFile);
                        }
                    }
                    else
                    {
                        feetProjections_[leg][i] = feetProjections_[leg][i - 1];
                        convexPolygons_[leg][i] = convexPolygons_[leg][i - 1];
                        nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
                    }

                    if (standStartTime < initTime && initTime < standFinalTime)
                    {
                        initStandFinalTime_[leg] = standFinalTime;
                    }
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            timeEvents_[leg] = eventTimes;
        }
    }

    feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(
        const std::vector<size_t>& phaseIDsStock) const
    {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool>> contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++)
        {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < info_.numThreeDofContacts; j++)
            {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }

    std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        if (!contactFlagStock[index])
        {
            return {0, 0};
        }

        // find the starting time
        int startTimesIndex = 0;
        for (int ip = index - 1; ip >= 0; ip--)
        {
            if (!contactFlagStock[ip])
            {
                startTimesIndex = ip;
                break;
            }
        }
        // find the final time
        int finalTimesIndex = numPhases - 2;
        for (size_t ip = index + 1; ip < numPhases; ip++)
        {
            if (!contactFlagStock[ip])
            {
                finalTimesIndex = ip - 1;
                break;
            }
        }
        return {startTimesIndex, finalTimesIndex};
    }

    vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories)
    {
        vector_t desiredState = targetTrajectories.getDesiredState(time);
        const auto measuredFootPositions = endEffectorKinematicsPtr_->getPosition(initState);
        const auto desiredFootPositions = endEffectorKinematicsPtr_->getPosition(desiredState);

        vector3_t nominalFoothold = desiredFootPositions[leg];
        // Keep the nominal lateral stance close to the planner's reference while
        // retaining some measured x/z anchoring for terrain contact continuity.
        nominalFoothold.x() = 0.15 * measuredFootPositions[leg].x() + 0.85 * desiredFootPositions[leg].x();
        nominalFoothold.y() = 0.10 * measuredFootPositions[leg].y() + 0.90 * desiredFootPositions[leg].y();
        nominalFoothold.z() = measuredFootPositions[leg].z();

        // Strong step-up assist for the front legs:
        // if the front foothold is approaching an elevated box region, directly place the
        // nominal foothold onto the box top so the front legs can actually target the terrain.
        constexpr size_t kFrontLeftLeg = 0;
        constexpr size_t kFrontRightLeg = 1;
        if ((leg == kFrontLeftLeg || leg == kFrontRightLeg) && planarTerrainPtr_ && planarTerrainPtr_->planarRegions.size() > 1)
        {
            const auto& boxRegion = planarTerrainPtr_->planarRegions[1];
            const scalar_t boxHeight = static_cast<scalar_t>(boxRegion.transformPlaneToWorld.translation().z());
            if (boxHeight > 1e-6)
            {
                const scalar_t boxCenterX = static_cast<scalar_t>(boxRegion.transformPlaneToWorld.translation().x());
                const scalar_t boxCenterY = static_cast<scalar_t>(boxRegion.transformPlaneToWorld.translation().y());
                const scalar_t boxMinX = boxCenterX + static_cast<scalar_t>(boxRegion.bbox2d.xmin());
                const scalar_t boxMaxX = boxCenterX + static_cast<scalar_t>(boxRegion.bbox2d.xmax());
                const scalar_t boxMinY = boxCenterY + static_cast<scalar_t>(boxRegion.bbox2d.ymin());
                const scalar_t boxMaxY = boxCenterY + static_cast<scalar_t>(boxRegion.bbox2d.ymax());

                const scalar_t triggerDistance = 0.20;
                const scalar_t insideMargin = 0.05;
                const bool withinApproachWindow =
                    nominalFoothold.x() > (boxMinX - triggerDistance) && nominalFoothold.x() < boxMaxX;
                const bool withinBoxLateralBand =
                    nominalFoothold.y() > (boxMinY + 0.03) && nominalFoothold.y() < (boxMaxY - 0.03);

                if (withinApproachWindow && withinBoxLateralBand)
                {
                    nominalFoothold.x() = boxMinX + insideMargin;
                    nominalFoothold.z() = boxHeight;
                }
            }
        }

        return nominalFoothold;
    }
} // namespace legged

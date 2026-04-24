//
// Created by biao on 3/21/25.
//

#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    namespace
    {
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> extractBasePath(
            const TargetTrajectories& targetTrajectories,
            const CentroidalModelInfo& info)
        {
            std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> basePath;
            basePath.reserve(targetTrajectories.stateTrajectory.size());

            for (const auto& state : targetTrajectories.stateTrajectory)
            {
                basePath.push_back(centroidal_model::getBasePose(state, info).head<3>());
            }

            return basePath;
        }

        scalar_t clampSymmetric(scalar_t value, scalar_t limit)
        {
            return std::clamp(value, -limit, limit);
        }

        scalar_t clampDelta(scalar_t previousValue, scalar_t candidateValue, scalar_t maxDelta)
        {
            return previousValue + std::clamp(candidateValue - previousValue, -maxDelta, maxDelta);
        }

        struct SupportHeightProfile
        {
            bool hasSupport = false;
            bool hasFrontSupport = false;
            bool hasRearSupport = false;
            scalar_t meanHeight = std::numeric_limits<scalar_t>::quiet_NaN();
            scalar_t frontHeight = std::numeric_limits<scalar_t>::quiet_NaN();
            scalar_t rearHeight = std::numeric_limits<scalar_t>::quiet_NaN();
            scalar_t anchorHeight = std::numeric_limits<scalar_t>::quiet_NaN();
        };

        SupportHeightProfile getSupportHeightProfile(const ConvexRegionSelector& convexRegionSelector,
                                                     const contact_flag_t& contactFlags, scalar_t time)
        {
            SupportHeightProfile profile;
            scalar_t supportHeightSum = 0.0;
            scalar_t frontHeightSum = 0.0;
            scalar_t rearHeightSum = 0.0;
            size_t supportCount = 0;
            size_t frontCount = 0;
            size_t rearCount = 0;

            for (size_t leg = 0; leg < contactFlags.size(); ++leg)
            {
                if (!contactFlags[leg])
                {
                    continue;
                }

                const auto projection = convexRegionSelector.getProjection(leg, time);
                if (projection.regionPtr == nullptr)
                {
                    continue;
                }

                const scalar_t height = projection.positionInWorld.z();
                supportHeightSum += height;
                ++supportCount;

                if (leg < 2)
                {
                    frontHeightSum += height;
                    ++frontCount;
                }
                else
                {
                    rearHeightSum += height;
                    ++rearCount;
                }
            }

            if (supportCount == 0)
            {
                return profile;
            }

            profile.hasSupport = true;
            profile.meanHeight = supportHeightSum / static_cast<scalar_t>(supportCount);

            if (frontCount > 0)
            {
                profile.hasFrontSupport = true;
                profile.frontHeight = frontHeightSum / static_cast<scalar_t>(frontCount);
            }

            if (rearCount > 0)
            {
                profile.hasRearSupport = true;
                profile.rearHeight = rearHeightSum / static_cast<scalar_t>(rearCount);
            }

            if (profile.hasFrontSupport && profile.hasRearSupport)
            {
                // Keep the body anchored to the higher support surface while the robot is split across a step.
                profile.anchorHeight = std::max(profile.frontHeight, profile.rearHeight);
            }
            else
            {
                profile.anchorHeight = profile.meanHeight;
            }

            return profile;
        }

        std::pair<int, int> findActiveSwingBounds(size_t initIndex, const std::vector<bool>& contactFlagStocks)
        {
            int swingStartIndex = static_cast<int>(initIndex);
            int swingFinalIndex = static_cast<int>(initIndex);

            while (swingStartIndex > 0 && !contactFlagStocks[swingStartIndex - 1])
            {
                --swingStartIndex;
            }
            while (swingFinalIndex + 1 < static_cast<int>(contactFlagStocks.size()) &&
                   !contactFlagStocks[swingFinalIndex + 1])
            {
                ++swingFinalIndex;
            }

            return {swingStartIndex, swingFinalIndex};
        }
    } // namespace

    PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info,
                                                                       std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                       std::shared_ptr<SwingTrajectoryPlanner>
                                                                       swingTrajectoryPtr,
                                                                       std::shared_ptr<ConvexRegionSelector>
                                                                       convexRegionSelectorPtr,
                                                                       const EndEffectorKinematics<scalar_t>&
                                                                       endEffectorKinematics,
                                                                       scalar_t comHeight)
        : info_(std::move(info)),
          SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
          convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          comHeight_(comHeight)
    {
        previousContactFlags_.fill(false);
        hasLatchedContactPosition_.fill(false);
        activeSwingHeightLatched_.fill(false);
        latchedSwingLiftOffHeights_.fill(0.0);
        latchedSwingTouchDownHeights_.fill(0.0);
        for (auto& position : lastLiftoffPos_)
        {
            position.setZero();
        }
    }

    void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                            const vector_t& initState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

        //copy raw target trajectories before modification
        const auto rawTargetTrajectories = targetTrajectories;

        // Footstep projections stay based on the raw foothold plan. The body reference then reads those
        // projections to avoid dropping toward the lower terrain layer while support feet are still split
        // across two different step heights.
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, rawTargetTrajectories);
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);

        if (enableReferenceModification_)
        {
            TargetTrajectories newTargetTrajectories;
            constexpr int nodeNum = 11;
            constexpr scalar_t normalSamplingStep = 0.3;
            constexpr scalar_t pitchBlend = 0.6;
            constexpr scalar_t heightBlend = 0.5;
            constexpr scalar_t maxAbsPitch = 0.25;
            constexpr scalar_t maxPitchDeltaPerNode = 0.06;
            constexpr scalar_t maxHeightDeltaPerNode = 0.03;
            constexpr scalar_t downStepHeightThreshold = 0.03;
            constexpr scalar_t downStepCommitDistance = 0.08;
            const vector_t initBasePose = centroidal_model::getBasePose(initState, info_);
            scalar_t previousPitch = initBasePose(4);
            scalar_t previousHeight = initBasePose(2);
            for (size_t i = 0; i < nodeNum; ++i)
            {
                scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
                vector_t state = targetTrajectories.getDesiredState(time);
                vector_t input = targetTrajectories.getDesiredInput(time);

                const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
                auto basePose = centroidal_model::getBasePose(state, info_);
                const scalar_t x = basePose(0);
                const scalar_t y = basePose(1);
                const scalar_t yaw = basePose(3);
                const scalar_t rawPitch = basePose(4);
                const scalar_t rawHeight = basePose(2);
                const size_t phaseIndex = static_cast<size_t>(std::min<int>(
                    lookup::findIndexInTimeArray(modeSchedule.eventTimes, time),
                    static_cast<int>(modeSchedule.modeSequence.size() - 1)));
                contact_flag_t contactFlags{};
                for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
                {
                    contactFlags[leg] = contactFlagStocks[leg][phaseIndex];
                }
                const auto supportProfile =
                    getSupportHeightProfile(*convexRegionSelectorPtr_, contactFlags, time);

                scalar_t limitedPitch = rawPitch;
                scalar_t limitedHeight = rawHeight;

                try
                {
                    grid_map::Vector3 normalVector;
                    normalVector(0) = (
                        map.atPosition("smooth_planar", grid_map::Position(x - normalSamplingStep, y)) -
                        map.atPosition("smooth_planar", grid_map::Position(x + normalSamplingStep, y))) /
                        (2 * normalSamplingStep);
                    normalVector(1) = (
                        map.atPosition("smooth_planar", grid_map::Position(x, y - normalSamplingStep)) -
                        map.atPosition("smooth_planar", grid_map::Position(x, y + normalSamplingStep))) /
                        (2 * normalSamplingStep);
                    normalVector(2) = 1;
                    normalVector.normalize();

                    matrix3_t R;
                    R << cos(yaw), -sin(yaw), 0, // clang-format off
                         sin(yaw), cos(yaw), 0,
                         0, 0, 1;  // clang-format on
                    const vector3_t normalInBase = R.transpose() * normalVector;
                    const scalar_t terrainPitch = std::atan2(normalInBase.x(), normalInBase.z());
                    limitedPitch = rawPitch + pitchBlend * (terrainPitch - rawPitch);
                    limitedPitch = clampSymmetric(limitedPitch, maxAbsPitch);
                    limitedPitch = clampDelta(previousPitch, limitedPitch, maxPitchDeltaPerNode);

                    const scalar_t safeCosPitch = std::max<scalar_t>(0.9, std::cos(limitedPitch));
                    scalar_t terrainReferenceHeight =
                        map.atPosition("smooth_planar", grid_map::Position(x, y));
                    if (supportProfile.hasSupport)
                    {
                        terrainReferenceHeight = std::max(terrainReferenceHeight, supportProfile.anchorHeight);
                    }
                    const scalar_t terrainAwareHeight =
                        terrainReferenceHeight + comHeight_ / safeCosPitch;
                    const scalar_t planarDistanceFromInit =
                        (basePose.head<2>() - initBasePose.head<2>()).norm();
                    const bool descendingToLowerTerrain =
                        terrainAwareHeight < rawHeight - downStepHeightThreshold;
                    if (descendingToLowerTerrain && planarDistanceFromInit > downStepCommitDistance)
                    {
                        limitedPitch = rawPitch;
                        limitedHeight = rawHeight;
                    }
                    else
                    {
                        limitedHeight = rawHeight + heightBlend * (terrainAwareHeight - rawHeight);
                        limitedHeight = clampDelta(previousHeight, limitedHeight, maxHeightDeltaPerNode);
                    }
                }
                catch (const std::exception&)
                {
                    limitedPitch = previousPitch;
                    limitedHeight = previousHeight;
                }

                basePose(4) = limitedPitch;
                basePose(2) = limitedHeight;
                previousPitch = limitedPitch;
                previousHeight = limitedHeight;

                newTargetTrajectories.timeTrajectory.push_back(time);
                newTargetTrajectories.stateTrajectory.push_back(state);
                newTargetTrajectories.inputTrajectory.push_back(input);
            }
            targetTrajectories = newTargetTrajectories;
        }

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);

        {
            std::lock_guard lock(latestReferenceTrajectoriesMutex_);
            latestRawBasePath_ = extractBasePath(rawTargetTrajectories, info_);
            latestTerrainAwareBasePath_ = extractBasePath(targetTrajectories, info_);
            latestFootPlacementDebugInfo_.time = initTime;
            latestFootPlacementDebugInfo_.contactFlags = getContactFlags(initTime);
            latestFootPlacementDebugInfo_.footPlacementFlags = getFootPlacementFlags(initTime);
            latestFootPlacementDebugInfo_.initStandFinalTimes = convexRegionSelectorPtr_->getInitStandFinalTimes();
            for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
            {
                const auto projection = convexRegionSelectorPtr_->getProjection(leg, initTime);
                latestFootPlacementDebugInfo_.polygonVertexCounts[leg] =
                    convexRegionSelectorPtr_->getConvexPolygon(leg, initTime).size();
                latestFootPlacementDebugInfo_.projectionHeights[leg] =
                    projection.regionPtr == nullptr ? std::numeric_limits<scalar_t>::quiet_NaN() : projection.positionInWorld.z();
            }
            hasLatestReferenceTrajectories_ = true;
        }
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            const size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);
            const bool currentContact = contactFlagStocks[leg][initIndex];
            const bool previousContact = previousContactFlags_[leg];

            auto projections = convexRegionSelectorPtr_->getProjections(leg);
            modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

            scalar_array_t liftOffHeights, touchDownHeights;
            std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], projections);

            if (!currentContact)
            {
                if (!activeSwingHeightLatched_[leg] || previousContact)
                {
                    activeSwingHeightLatched_[leg] = true;
                    latchedSwingLiftOffHeights_[leg] = liftOffHeights[initIndex];
                    latchedSwingTouchDownHeights_[leg] = touchDownHeights[initIndex];
                }

                const auto [swingStartIndex, swingFinalIndex] = findActiveSwingBounds(initIndex, contactFlagStocks[leg]);
                for (int i = swingStartIndex; i <= swingFinalIndex; ++i)
                {
                    liftOffHeights[i] = latchedSwingLiftOffHeights_[leg];
                    touchDownHeights[i] = latchedSwingTouchDownHeights_[leg];
                }
            }
            else if (!previousContact)
            {
                activeSwingHeightLatched_[leg] = false;
            }

            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;
            previousContactFlags_[leg] = currentContact;
        }
        swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    }

    void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg,
                                                             size_t initIndex,
                                                             const std::vector<bool>& contactFlagStocks,
                                                             std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections)
    {
        const bool currentContact = contactFlagStocks[initIndex];
        const bool enteringContact =
            currentContact && (!hasLatchedContactPosition_[leg] || !previousContactFlags_[leg]);

        if (enteringContact)
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            lastLiftoffPos_[leg].z() -= 0.08;
            hasLatchedContactPosition_[leg] = true;
        }

        if (currentContact && hasLatchedContactPosition_[leg])
        {
            for (int i = static_cast<int>(initIndex); i < static_cast<int>(projections.size()); ++i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
            for (int i = static_cast<int>(initIndex); i >= 0; --i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
        }
        if (hasLatchedContactPosition_[leg] && initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg])
        {
            for (int i = static_cast<int>(initIndex); i >= 0; --i)
            {
                if (contactFlagStocks[i])
                {
                    projections[i].positionInWorld = lastLiftoffPos_[leg];
                }
                if (!contactFlagStocks[i] && !contactFlagStocks[i + 1])
                {
                    break;
                }
            }
        }
        //    for (int i = 0; i < numPhases; ++i) {
        //      if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
        //    }
        //    std::cerr << std::endl;
    }

    std::pair<scalar_array_t, scalar_array_t> PerceptiveLeggedReferenceManager::getHeights(
        const std::vector<bool>& contactFlagStocks,
        const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections)
    {
        scalar_array_t liftOffHeights, touchDownHeights;
        const size_t numPhases = projections.size();

        liftOffHeights.clear();
        liftOffHeights.resize(numPhases);
        touchDownHeights.clear();
        touchDownHeights.resize(numPhases);

        for (size_t i = 1; i < numPhases; ++i)
        {
            if (!contactFlagStocks[i])
            {
                liftOffHeights[i] = contactFlagStocks[i - 1]
                                        ? projections[i - 1].positionInWorld.z()
                                        : liftOffHeights[i - 1];
            }
        }
        for (int i = numPhases - 2; i >= 0; --i)
        {
            if (!contactFlagStocks[i])
            {
                touchDownHeights[i] = contactFlagStocks[i + 1]
                                          ? projections[i + 1].positionInWorld.z()
                                          : touchDownHeights[i + 1];
            }
        }

        //  for (int i = 0; i < numPhases; ++i) {
        //    std::cerr << std::setprecision(3) << liftOffHeights[i] << "\t";
        //  }
        //  std::cerr << std::endl;
        //  for (int i = 0; i < numPhases; ++i) {
        //    std::cerr << std::setprecision(3) << contactFlagStocks[i] << "\t";
        //  }
        //  std::cerr << std::endl;

        return {liftOffHeights, touchDownHeights};
    }

    contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const
    {
        contact_flag_t flag;
        const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
        for (int i = 0; i < flag.size(); ++i)
        {
            flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
        }
        return flag;
    }

    bool PerceptiveLeggedReferenceManager::getLatestReferencePaths(
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& rawBasePath,
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& terrainAwareBasePath) const
    {
        std::lock_guard lock(latestReferenceTrajectoriesMutex_);
        if (!hasLatestReferenceTrajectories_)
        {
            return false;
        }

        rawBasePath = latestRawBasePath_;
        terrainAwareBasePath = latestTerrainAwareBasePath_;
        return true;
    }

    bool PerceptiveLeggedReferenceManager::getLatestFootPlacementDebugInfo(FootPlacementDebugInfo& debugInfo) const
    {
        std::lock_guard lock(latestReferenceTrajectoriesMutex_);
        if (!hasLatestReferenceTrajectories_)
        {
            return false;
        }

        debugInfo = latestFootPlacementDebugInfo_;
        return true;
    }
} // namespace legged

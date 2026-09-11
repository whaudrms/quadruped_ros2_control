//
// Created by biao on 3/21/25.
//

#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
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
                                                            const vector_t& mpcInitState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        const vector_t initState = mpcInitState.head(info_.stateDim);
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
        // Derive all timing from the original gait. Delay stance to robust end,
        // then restore stance from confirmed contact times. Neither operation
        // mutates GaitSchedule or shifts subsequent liftoffs.
        std::vector<RobustTouchdownTiming> robustTimings;
        if (robustPhaseSettings_.enabled) {
            robustTimings = makeRobustTouchdownTimings(modeSchedule, robustPhaseSettings_.t_a,
                                                       robustPhaseSettings_.t_b);
            modeSchedule = delayRobustTouchdowns(modeSchedule, robustTimings);
        }
        applyPendingSplices(initTime, finalTime, initState, modeSchedule);

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
            constexpr int nodeNum = 21;
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

        // Robust phase windows (per leg, first upcoming touchdown only for M1''/M2 scope).
        computeRobustWindows(initTime, finalTime, modeSchedule, robustTimings);

        // Preserve the future FL swing reference generated by this MPC update.
        // The controller-side CSV writer records each sequence exactly once,
        // retaining pre-contact snapshots even after later replans occur.
        updateLatestFootholdPlanSnapshot(initTime, finalTime, modeSchedule);

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
        if (visualizationCallback_)
        {
            visualizationCallback_(initTime, finalTime, modeSchedule);
        }
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;
        SwingTrajectoryPlanner::TerrainSwings terrainSwings;
        const auto& swingConfig = swingTrajectoryPtr_->config();

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

            if (swingConfig.terrainAware) {
                for (size_t phase = 1; phase + 1 < contactFlagStocks[leg].size(); ++phase) {
                    if (contactFlagStocks[leg][phase] || !contactFlagStocks[leg][phase - 1]) continue;
                    size_t touchdown = phase + 1;
                    while (touchdown < contactFlagStocks[leg].size() && !contactFlagStocks[leg][touchdown]) ++touchdown;
                    if (touchdown == contactFlagStocks[leg].size()) break;
                    const scalar_t start = modeSchedule.eventTimes[phase - 1];
                    const scalar_t end = modeSchedule.eventTimes[touchdown - 1];
                    const auto& from = projections[phase - 1];
                    const auto& to = projections[touchdown];
                    if (!from.regionPtr || !to.regionPtr) continue;
                    const vector3_t fromNormal = from.regionPtr->transformPlaneToWorld.linear().col(2);
                    const vector3_t toNormal = to.regionPtr->transformPlaneToWorld.linear().col(2);
                    // Current/last contact anchor was measured in FK then lowered along world Z.
                    // Future contacts use the selected surface normal and the physical foot radius.
                    const bool activeSwing = phase <= initIndex && initIndex < touchdown;
                    const bool fromCurrentStance = currentContact && phase > initIndex &&
                        std::all_of(contactFlagStocks[leg].begin() + initIndex,
                                    contactFlagStocks[leg].begin() + phase, [](bool contact) { return contact; });
                    const bool measuredAnchor = hasLatchedContactPosition_[leg] && (activeSwing || fromCurrentStance);
                    const vector3_t startPosition = measuredAnchor
                        ? vector3_t(lastLiftoffPos_[leg] + swingConfig.footRadius * vector3_t::UnitZ())
                        : vector3_t(from.positionInWorld + swingConfig.footRadius * fromNormal);
                    const vector3_t endPosition = to.positionInWorld + swingConfig.footRadius * toNormal;
                    auto trajectory = std::make_shared<TerrainSwing>(start, end, startPosition, endPosition,
                        fromNormal, toNormal, swingConfig.liftOffVelocity, swingConfig.touchDownVelocity,
                        swingConfig.swingHeight, swingConfig.swingTimeScale,
                        convexRegionSelectorPtr_->getHeightProfileAlongLine(startPosition, endPosition));
                    terrainSwings[leg].push_back(std::move(trajectory));
                    phase = touchdown - 1;
                }
            }

            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;
            previousContactFlags_[leg] = currentContact;
        }
        latestTouchDownHeightSequence_ = touchDownHeightSequence;
        swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
        swingTrajectoryPtr_->setTerrainSwings(std::move(terrainSwings));
    }

    void PerceptiveLeggedReferenceManager::updateLatestFootholdPlanSnapshot(
        const scalar_t initTime, const scalar_t finalTime, const ModeSchedule& modeSchedule)
    {
        constexpr size_t kFlLeg = 0;
        const auto& eventTimes = modeSchedule.eventTimes;
        const auto& modeSequence = modeSchedule.modeSequence;
        if (modeSequence.empty())
        {
            return;
        }

        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSequence);
        const auto& flContact = contactFlagStocks[kFlLeg];
        const int initPhaseInt = lookup::findIndexInTimeArray(eventTimes, initTime);
        const size_t initPhase = static_cast<size_t>(std::clamp<int>(
            initPhaseInt, 0, static_cast<int>(modeSequence.size()) - 1));

        // Select the first active or future FL swing that ends after initTime.
        size_t swingPhase = modeSequence.size();
        for (size_t phase = initPhase; phase < flContact.size(); ++phase)
        {
            if (!flContact[phase])
            {
                swingPhase = phase;
                break;
            }
        }
        if (swingPhase == modeSequence.size())
        {
            return;
        }

        size_t swingStartPhase = swingPhase;
        while (swingStartPhase > 0 && !flContact[swingStartPhase - 1])
        {
            --swingStartPhase;
        }
        size_t stancePhase = swingPhase + 1;
        while (stancePhase < flContact.size() && !flContact[stancePhase])
        {
            ++stancePhase;
        }
        if (stancePhase >= flContact.size() || swingStartPhase == 0)
        {
            return;
        }

        const size_t liftOffEventIndex = swingStartPhase - 1;
        const size_t touchDownEventIndex = stancePhase - 1;
        if (liftOffEventIndex >= eventTimes.size() || touchDownEventIndex >= eventTimes.size())
        {
            return;
        }

        const scalar_t liftOffTime = eventTimes[liftOffEventIndex];
        const scalar_t touchDownTime = eventTimes[touchDownEventIndex];
        if (touchDownTime <= initTime || touchDownTime > finalTime || touchDownTime <= liftOffTime)
        {
            return;
        }
        if (kFlLeg >= latestTouchDownHeightSequence_.size() ||
            swingPhase >= latestTouchDownHeightSequence_[kFlLeg].size())
        {
            return;
        }

        FootholdPlanSnapshot snapshot;
        snapshot.solveTime = initTime;
        snapshot.horizonEndTime = finalTime;
        snapshot.liftOffTime = liftOffTime;
        snapshot.touchDownTime = touchDownTime;
        snapshot.touchDownHeight = latestTouchDownHeightSequence_[kFlLeg][swingPhase];
        snapshot.robustEnabled = robustPhaseSettings_.enabled;
        snapshot.robustWindow = getRobustWindow(kFlLeg);

        const scalar_t sampleDt = std::max<scalar_t>(robustPhaseSettings_.dt_mpc, 1e-4);
        for (scalar_t sampleTime = liftOffTime; sampleTime < touchDownTime; sampleTime += sampleDt)
        {
            snapshot.sampleTimes.push_back(sampleTime);
            snapshot.zReferences.push_back(
                swingTrajectoryPtr_->getZpositionConstraint(kFlLeg, sampleTime));
            snapshot.zVelocityReferences.push_back(
                swingTrajectoryPtr_->getZvelocityConstraint(kFlLeg, sampleTime));
        }
        snapshot.sampleTimes.push_back(touchDownTime);
        // At an exact mode boundary lookup may select the stance-side spline.
        // Preserve the swing-side terminal node explicitly instead.
        snapshot.zReferences.push_back(snapshot.touchDownHeight);
        snapshot.zVelocityReferences.push_back(0.0);

        // Keep the existing CSV/plot contract: logged z_ref + guard offset is
        // the FK foot-frame reference, even when radius and guard offset differ.
        if (const auto* swing = swingTrajectoryPtr_->getTerrainSwing(kFlLeg, 0.5 * (liftOffTime + touchDownTime))) {
            snapshot.touchDownHeight = swing->endPosition.z() - robustPhaseSettings_.foot_frame_offset;
            for (size_t k = 0; k < snapshot.sampleTimes.size(); ++k) {
                snapshot.zReferences[k] = swing->spline.position(snapshot.sampleTimes[k]).z() - robustPhaseSettings_.foot_frame_offset;
                snapshot.zVelocityReferences[k] = swing->spline.velocity(snapshot.sampleTimes[k]).z();
            }
        }

        {
            std::lock_guard lock(footholdPlanSnapshotMutex_);
            snapshot.sequence = ++footholdPlanSnapshotSequence_;
            latestFootholdPlanSnapshot_ = std::move(snapshot);
            footholdPlanSnapshotHistory_.push_back(latestFootholdPlanSnapshot_);
            constexpr size_t kMaximumSnapshotHistory = 256;
            if (footholdPlanSnapshotHistory_.size() > kMaximumSnapshotHistory)
            {
                footholdPlanSnapshotHistory_.pop_front();
            }
            hasLatestFootholdPlanSnapshot_ = true;
        }
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

        if (enteringContact || (currentContact && swingTrajectoryPtr_->config().terrainAware))
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            // Legacy height arrays use a contact-surface anchor. The 3D planner
            // restores this physical radius and therefore starts at measured FK.
            lastLiftoffPos_[leg].z() -= swingTrajectoryPtr_->config().footRadius;
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

    void PerceptiveLeggedReferenceManager::computeRobustWindows(
        scalar_t initTime, scalar_t finalTime, const ModeSchedule& modeSchedule,
        const std::vector<RobustTouchdownTiming>& timings)
    {
        feet_array_t<RobustWindowData> windows{};
        if (!robustPhaseSettings_.enabled)
        {
            std::lock_guard lock(robustWindowsMutex_);
            robustWindows_ = windows;
            return;
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
        {
            // Select the first unfinished window, even when nominal touchdown
            // is already in the past but its delayed robust end is still ahead.
            const RobustTouchdownTiming* selected = nullptr;
            for (const auto& timing : timings) {
                if (timing.leg != leg || timing.end <= initTime || timing.end > finalTime) continue;
                const bool contacted = std::any_of(robustContactOverrides_.begin(), robustContactOverrides_.end(),
                    [&](const RobustContactOverride& contact) {
                        return contact.leg == leg && contact.touchdownTime == timing.end;
                    });
                if (!contacted && (!selected || timing.end < selected->end)) selected = &timing;
            }
            if (!selected) continue;

            auto& w = windows[leg];
            w.active = true;
            w.d = robustPhaseSettings_.d;
            w.d_state_index = robustPhaseSettings_.optimize_d ? static_cast<int>(info_.stateDim + leg) : -1;
            w.n = vector3_t::UnitZ();
            w.foot_frame_offset = robustPhaseSettings_.foot_frame_offset;
            // Keep absolute endpoints stable across MPC replans. Only skip the
            // elapsed start boundary, never shrink the denominator for v_max.
            w.t_a = selected->start;
            w.t_b = selected->end;
            w.t_nominal = selected->nominalTouchdown;
            w.skip_t_a_boundary = w.t_a < initTime;
            w.dt_mpc = robustPhaseSettings_.dt_mpc;
            w.v_max = robustPhaseVelocityLimit(robustPhaseSettings_.d_max, w.t_a, w.t_b);
            const size_t stancePhase = static_cast<size_t>(
                std::upper_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), w.t_b) -
                modeSchedule.eventTimes.begin());
            w.k_a_phase_index = stancePhase - 1;
            w.k_b_phase_index = stancePhase;

            // M1'' default: flat-ground guard plane.
            w.p_plane = vector3_t::Zero();
            w.p_plane.z() = robustPhaseSettings_.terrain_z_M1;

            // M2 path: pull stance-side terrain projection from ConvexRegionSelector.
            // Use the phase-index path (NOT getProjection(leg, t_b + eps)) so we share
            // exactly the same projection the existing perceptive touchdown-height code uses
            // (PerceptiveLeggedReferenceManager.cpp:443-451 reads projections[i+1].positionInWorld.z()
            // at the swing→stance boundary, where i+1 == stancePhase).
            if (robustPhaseSettings_.terrain_source == "convex_region")
            {
                const auto perLegProjections = convexRegionSelectorPtr_->getProjections(leg);
                if (stancePhase < perLegProjections.size())
                {
                    const auto& proj = perLegProjections[stancePhase];
                    if (proj.regionPtr != nullptr)
                    {
                        w.p_plane = proj.positionInWorld;
                        // n stays e_z for now (horizontal step surfaces). M2.x will extract
                        // the plane normal from proj.regionPtr->transformPlaneToWorld for
                        // inclined terrain.
                    }
                    // else: leave the flat fallback (terrain_z_M1) intact for this leg.
                }
            }
        }

        {
            std::lock_guard lock(robustWindowsMutex_);
            robustWindows_ = windows;
        }

        // Verification log — one line per MPC cycle, gated by robustPhase.verbose_log
        // to keep the RT control thread quiet on real hardware. Off by default.
        // Format (machine-parseable, used by tools/perceptive_dev_v2/plot_robust_phase.py):
        //   [robust_phase] t=... leg=N active=0/1 ta=... tb=... pz=... d=... clamped=0/1
        if (robustPhaseSettings_.verbose_log)
        {
            for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
            {
                const auto& w = windows[leg];
                std::cerr << "[robust_phase] t=" << initTime
                          << " leg=" << leg
                          << " active=" << (w.active ? 1 : 0)
                          << " ta=" << w.t_a
                          << " tb=" << w.t_b
                          << " pz=" << w.p_plane.z()
                          << " d=" << w.d
                          << " offset=" << w.foot_frame_offset
                          << " clamped=" << (w.skip_t_a_boundary ? 1 : 0)
                          << " t_nominal=" << w.t_nominal
                          << " v_max=" << w.v_max
                          << "\n";
            }
        }
    }

    void PerceptiveLeggedReferenceManager::requestRobustContactSplice(const size_t leg, const scalar_t event_time,
                                                                    const RobustWindowData& window)
    {
        if (!robustPhaseSettings_.enabled || !robustPhaseSettings_.enable_splice) return;
        // Use the first contact tick's snapshot, not a window recomputed during
        // debounce. A delayed request must never target a later swing.
        if (leg >= contact_flag_t{}.size() || !window.active || !std::isfinite(event_time) ||
            event_time < window.t_a || event_time >= window.t_b) return;
        const RobustContactOverride request{leg, event_time, window.t_a, window.t_b};
        std::lock_guard lk(splice_mutex_);
        for (auto& pending : robustContactSplicePending_) {
            if (pending.leg == leg && pending.touchdownTime == window.t_b) {
                if (event_time < pending.eventTime) pending = request;
                return;
            }
        }
        robustContactSplicePending_.push_back(request);
    }

    void PerceptiveLeggedReferenceManager::applyPendingSplices(const scalar_t initTime,
                                                              const scalar_t finalTime,
                                                              const vector_t& initState,
                                                              ModeSchedule& modeSchedule)
    {
        std::vector<RobustContactOverride> pending;
        {
            std::lock_guard lk(splice_mutex_);
            pending.swap(robustContactSplicePending_);
        }
        if (!robustPhaseSettings_.enabled || !robustPhaseSettings_.enable_splice) {
            robustContactOverrides_.clear();
            return;
        }

        // Retain recent history for projection/swing planning. Reject overrides
        // whose configured touchdown disappeared after a gait change. Validation
        // uses the delayed schedule, never another contact's overlay.
        const scalar_t historyStart = initTime - (finalTime - initTime);
        const auto obsolete = [&](const RobustContactOverride& contact) {
            return contact.touchdownTime <= historyStart ||
                   !isValidRobustContactOverride(modeSchedule, contact);
        };
        robustContactOverrides_.erase(
            std::remove_if(robustContactOverrides_.begin(), robustContactOverrides_.end(), obsolete),
            robustContactOverrides_.end());

        for (const auto& request : pending) {
            if (obsolete(request)) {
                std::cerr << "[robust_contact_splice_rejected] leg=" << request.leg
                          << " t=" << request.eventTime << " reason=stale_or_invalid_window\n";
                continue;
            }
            auto existing = std::find_if(robustContactOverrides_.begin(), robustContactOverrides_.end(),
                [&](const RobustContactOverride& contact) {
                    return contact.leg == request.leg && contact.touchdownTime == request.touchdownTime;
                });
            if (existing != robustContactOverrides_.end()) {
                if (existing->eventTime <= request.eventTime) continue;
                *existing = request;
            } else {
                robustContactOverrides_.push_back(request);
            }

            // Diagnostic displacement is evaluated at apply time, not contact time.
            scalar_t gEvent = std::numeric_limits<scalar_t>::quiet_NaN();
            const auto window = getRobustWindow(request.leg);
            if (window.active && window.t_b == request.touchdownTime) {
                const auto position = endEffectorKinematicsPtr_->getPosition(initState).at(request.leg);
                gEvent = window.n.dot(position - window.p_plane) - window.foot_frame_offset;
            }
            std::cerr << "[robust_contact_splice] leg=" << request.leg
                      << " t=" << request.eventTime << " splice_t=" << request.eventTime
                      << " t_b=" << request.touchdownTime << " g_event=" << gEvent
                      << " transition=robust->stance stage=mpc_schedule apply_t=" << initTime
                      << " (stance overlay; nominal gait unchanged)\n";
        }

        // No time snapping, cross-leg batching, or whole-phase merging. The
        // configured touchdown and every subsequent liftoff retain their times.
        modeSchedule = applyRobustContactOverrides(modeSchedule, robustContactOverrides_);
    }

    bool PerceptiveLeggedReferenceManager::isInRobustWindow(size_t leg, scalar_t time) const
    {
        std::lock_guard lock(robustWindowsMutex_);
        if (leg >= robustWindows_.size())
        {
            return false;
        }
        const auto& w = robustWindows_[leg];
        return w.active && time >= w.t_a && time <= w.t_b;
    }

    RobustWindowData PerceptiveLeggedReferenceManager::getRobustWindow(size_t leg) const
    {
        std::lock_guard lock(robustWindowsMutex_);
        if (leg >= robustWindows_.size())
        {
            return RobustWindowData{};
        }
        return robustWindows_[leg];  // value copy under lock — safe to release
    }

    PerceptiveLeggedReferenceManager::RobustPhaseSettings loadRobustPhaseSettings(
        const std::string& taskFile, bool verbose)
    {
        PerceptiveLeggedReferenceManager::RobustPhaseSettings s;
        boost::property_tree::ptree pt;
        try
        {
            boost::property_tree::read_info(taskFile, pt);
        }
        catch (const std::exception& e)
        {
            if (verbose)
            {
                std::cerr << "[loadRobustPhaseSettings] failed to read task file: " << e.what() << std::endl;
            }
            return s;
        }

        const std::string prefix = "robustPhase.";
        if (verbose)
        {
            std::cerr << "\n #### Robust Phase Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, s.enabled,           prefix + "enabled",           verbose);
        loadData::loadPtreeValue(pt, s.t_a,               prefix + "t_a",               verbose);
        loadData::loadPtreeValue(pt, s.t_b,               prefix + "t_b",               verbose);
        loadData::loadPtreeValue(pt, s.optimize_d, prefix + "optimize_d", verbose);
        loadData::loadPtreeValue(pt, s.d_min, prefix + "d_min", verbose);
        loadData::loadPtreeValue(pt, s.d_max, prefix + "d_max", verbose);
        loadData::loadPtreeValue(pt, s.d,                 prefix + "d",                 verbose);
        loadData::loadPtreeValue(pt, s.hard_boundary_start, prefix + "hard_boundary_start", verbose);
        loadData::loadPtreeValue(pt, s.hard_boundary_end,   prefix + "hard_boundary_end",   verbose);
        loadData::loadPtreeValue(pt, s.slack_boundary_start, prefix + "slack_boundary_start", verbose);
        loadData::loadPtreeValue(pt, s.slack_boundary_end,   prefix + "slack_boundary_end",   verbose);
        loadData::loadPtreeValue(pt, s.slack_boundary_weight_start,
                                 prefix + "slack_boundary_weight_start", verbose);
        loadData::loadPtreeValue(pt, s.slack_boundary_weight_end,
                                 prefix + "slack_boundary_weight_end", verbose);
        loadData::loadPtreeValue(pt, s.terrain_source,    prefix + "terrain_source",    verbose);
        loadData::loadPtreeValue(pt, s.terrain_z_M1,      prefix + "terrain_z_M1",      verbose);
        loadData::loadPtreeValue(pt, s.foot_frame_offset, prefix + "foot_frame_offset", verbose);
        if (pt.get_child_optional(prefix + "P") || pt.get_child_optional(prefix + "v_max")) {
            std::cerr << "[loadRobustPhaseSettings] legacy P/v_max ignored; use t_a/t_b offsets; "
                         "v_max is computed as 2*d_max/(end-start)\n";
        }
        loadData::loadPtreeValue(pt, s.enable_splice,     prefix + "enable_splice",     verbose);
        loadData::loadPtreeValue(pt, s.verbose_log,       prefix + "verbose_log",       verbose);
        validateRobustPhaseTiming(s.t_a, s.t_b, s.d);
        if (s.hard_boundary_start && s.slack_boundary_start)
        {
            throw std::invalid_argument(
                "robustPhase start boundary cannot be both hard and slack");
        }
        if (s.hard_boundary_end && s.slack_boundary_end)
        {
            throw std::invalid_argument(
                "robustPhase end boundary cannot be both hard and slack");
        }
        if (s.slack_boundary_weight_start <= 0.0 || s.slack_boundary_weight_end <= 0.0)
        {
            throw std::invalid_argument(
                "robustPhase slack boundary weights must be positive");
        }
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }
        return s;
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

    bool PerceptiveLeggedReferenceManager::getLatestFootholdPlanSnapshot(
        FootholdPlanSnapshot& snapshot) const
    {
        std::lock_guard lock(footholdPlanSnapshotMutex_);
        if (!hasLatestFootholdPlanSnapshot_)
        {
            return false;
        }

        snapshot = latestFootholdPlanSnapshot_;
        return true;
    }

    bool PerceptiveLeggedReferenceManager::getFootholdPlanSnapshot(
        const scalar_t solveTime, FootholdPlanSnapshot& snapshot) const
    {
        std::lock_guard lock(footholdPlanSnapshotMutex_);
        if (footholdPlanSnapshotHistory_.empty())
        {
            return false;
        }

        const auto closest = std::min_element(
            footholdPlanSnapshotHistory_.begin(), footholdPlanSnapshotHistory_.end(),
            [solveTime](const auto& lhs, const auto& rhs)
            {
                return std::abs(lhs.solveTime - solveTime) < std::abs(rhs.solveTime - solveTime);
            });
        const scalar_t tolerance = std::max<scalar_t>(2.0 * robustPhaseSettings_.dt_mpc, 0.05);
        if (closest == footholdPlanSnapshotHistory_.end() ||
            std::abs(closest->solveTime - solveTime) > tolerance)
        {
            return false;
        }

        snapshot = *closest;
        return true;
    }
} // namespace legged

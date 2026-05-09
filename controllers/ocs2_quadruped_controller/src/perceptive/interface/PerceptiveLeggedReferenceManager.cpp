//
// Created by biao on 3/21/25.
//

#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>
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
                                                            const vector_t& initState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        // Drain any pending splice requests BEFORE we read the schedule below.
        // OCS2 SolverBase::preRun calls referenceManagerPtr_->preSolverRun (this
        // path) FIRST, then synchronized modules. So the only way for a splice
        // to manifest in the SAME solve cycle is to apply it here, before the
        // line-180 getModeSchedule() read that feeds convexRegionSelector,
        // swing planner, and computeRobustWindows. Splicing later (e.g., from
        // GaitManager::preSolverRun, the previous home for this code) means a
        // one-cycle latency where the reference work runs on stale schedule.
        applyPendingSplices(initTime, finalTime, initState);

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
        computeRobustWindows(initTime, finalTime, modeSchedule, initState);

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
            // 0.08 m is a perceptive-planner-side touchdown projection bias used
            // to anchor the swing planner's lift-off / touchdown heights at the
            // actual contact point (FK foot frame is at the ankle, ~0.08 m
            // above contact for go2). This is independent of robust phase's
            // RobustGuardBoundaryConstraint::foot_frame_offset (loaded from
            // task.info, separate physical quantity used in g(x) = n·(p_foot −
            // p_plane) − foot_frame_offset). Do not blindly equate the two:
            // touching this number alters the perceptive planner's stair-down
            // behavior (0.02 caused stair-down failures in earlier sweeps).
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

    void PerceptiveLeggedReferenceManager::computeRobustWindows(scalar_t initTime, scalar_t finalTime,
                                                                const ModeSchedule& modeSchedule,
                                                                const vector_t& initState)
    {
        feet_array_t<RobustWindowData> windows{};
        if (!robustPhaseSettings_.enabled)
        {
            std::lock_guard lock(robustWindowsMutex_);
            robustWindows_ = windows;
            return;
        }

        const auto& eventTimes = modeSchedule.eventTimes;
        const auto& modeSequence = modeSchedule.modeSequence;
        const size_t numPhases = modeSequence.size();
        if (numPhases == 0)
        {
            std::lock_guard lock(robustWindowsMutex_);
            robustWindows_ = windows;
            return;
        }

        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSequence);
        const int initPhaseInt = lookup::findIndexInTimeArray(eventTimes, initTime);
        const size_t initPhase = static_cast<size_t>(std::clamp<int>(initPhaseInt, 0, static_cast<int>(numPhases) - 1));

        const scalar_t T_robust = static_cast<scalar_t>(robustPhaseSettings_.P) * robustPhaseSettings_.dt_mpc;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
        {
            RobustWindowData& w = windows[leg];
            w.active = false;
            w.d = robustPhaseSettings_.d;
            w.n = vector3_t::UnitZ();   // M1''/M2-flat: world-z guard
            w.foot_frame_offset = robustPhaseSettings_.foot_frame_offset;

            const auto& flags = contactFlagStocks[leg];

            // Find the first FUTURE swing→stance transition for this leg.
            // We can't just take the first false→true transition starting from initPhase
            // because the leg may currently be in stance (flags[initPhase]==true) — in that
            // case the transition at initPhase was already in the past, and we need to
            // scan further to locate the next swing→stance after the upcoming swing.
            //
            // So we keep scanning while either (a) the candidate event time is ≤ initTime,
            // or (b) no transition is found yet. (a) catches both "leg is in stance now"
            // and "the next transition straddles initTime".
            size_t stancePhase = numPhases;  // sentinel: not found
            for (size_t i = std::max<size_t>(initPhase, 1); i < numPhases; ++i)
            {
                if (!(flags[i] && !flags[i - 1]))
                {
                    continue;
                }
                const size_t eventIdx = i - 1;
                if (eventIdx >= eventTimes.size())
                {
                    continue;
                }
                if (eventTimes[eventIdx] <= initTime)
                {
                    continue;  // past transition (leg already in stance) — keep scanning
                }
                stancePhase = i;
                break;
            }
            if (stancePhase == numPhases)
            {
                continue;  // no upcoming touchdown for this leg in this horizon
            }

            const size_t swingPhase = stancePhase - 1;
            const size_t eventIdx = stancePhase - 1;
            const scalar_t t_b = eventTimes[eventIdx];
            if (t_b > finalTime)
            {
                continue;  // touchdown outside MPC horizon
            }

            scalar_t t_a = t_b - T_robust;
            const bool clamped = (t_a < initTime);
            if (clamped)
            {
                t_a = initTime;
            }

            w.active = true;
            w.skip_t_a_boundary = clamped;
            w.dt_mpc = robustPhaseSettings_.dt_mpc;
            w.t_a = t_a;
            w.t_b = t_b;
            w.k_a_phase_index = swingPhase;
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
                          << "\n";
            }
        }
    }

    void PerceptiveLeggedReferenceManager::requestRobustContactSplice(const size_t leg, const scalar_t event_time)
    {
        // Robust-window contact event splice is a robust-phase feature — must
        // be a no-op when robust phase is disabled, so OFF trials remain a
        // clean control.
        if (!robustPhaseSettings_.enabled) return;
        if (leg >= robust_contact_splice_pending_.size()) return;
        std::lock_guard lk(splice_mutex_);
        // If a request for this leg is already pending, anchor on the EARLIEST
        // event_time — the moment of first measured contact in the window.
        if (robust_contact_splice_pending_[leg] && event_time >= robust_contact_splice_time_[leg]) return;
        robust_contact_splice_pending_[leg] = true;
        robust_contact_splice_time_[leg] = event_time;
    }

    void PerceptiveLeggedReferenceManager::applyPendingSplices(const scalar_t initTime,
                                                                const scalar_t finalTime,
                                                                const vector_t& initState)
    {
        // Drain queue under lock, then operate without lock.
        feet_array_t<bool>     pending{};
        feet_array_t<scalar_t> time{};
        {
            std::lock_guard lk(splice_mutex_);
            for (size_t i = 0; i < robust_contact_splice_pending_.size(); ++i) {
                pending[i] = robust_contact_splice_pending_[i];
                time[i]    = robust_contact_splice_time_[i];
                robust_contact_splice_pending_[i] = false;
            }
        }

        // Collect (t, leg) requests in chronological order so subsequent inserts
        // see the prior inserts. (Ordering matters because findIndexInTimeArray
        // depends on the current contents of eventTimes.)
        std::vector<std::pair<scalar_t, size_t>> requests;
        for (size_t leg = 0; leg < pending.size(); ++leg) {
            if (pending[leg]) requests.emplace_back(time[leg], leg);
        }
        if (requests.empty()) return;
        std::sort(requests.begin(), requests.end());

        // Pull a wide enough slice to cover any splice request; getModeSchedule
        // is allowed to mutate internal state because we are on the MPC thread,
        // serialized with all other GaitSchedule reads/writes (this method runs
        // BEFORE getModeSchedule on line 180 of modifyReferences).
        const scalar_t loBound = std::min(initTime, requests.front().first) - 0.5;
        const scalar_t hiBound = finalTime + 0.5;
        ModeSchedule sched = getGaitSchedule()->getModeSchedule(loBound, hiBound);

        const scalar_t dt_mpc = robustPhaseSettings_.dt_mpc;
        const scalar_t kMergeToNominalThreshold = 2.0 * dt_mpc;  // ~0.04 s at default dt=0.02

        for (const auto& [t, leg] : requests) {
            const int curPhaseInt = lookup::findIndexInTimeArray(sched.eventTimes, t);
            const size_t curPhase = static_cast<size_t>(
                std::clamp<int>(curPhaseInt, 0, static_cast<int>(sched.modeSequence.size()) - 1));

            contact_flag_t curFlags = modeNumber2StanceLeg(sched.modeSequence[curPhase]);
            if (curFlags[leg]) continue;  // already stance — splice would be a no-op

            // === Engineering "merge-to-nominal-touchdown" guard ===
            // If t is within 2*dt_mpc of the leg's next nominal event (the
            // already-scheduled t_b), skip the splice. Inserting a sub-2-shoot
            // phase destabilizes SQP shooting and WBC mode transitions; the
            // gait would have reached stance at t_b in a fraction of a dt_mpc
            // anyway. NOT part of the paper's strict event-triggered MPC —
            // purely a numerical guard.
            if (curPhase < sched.eventTimes.size()) {
                const scalar_t nextEventTime = sched.eventTimes[curPhase];
                const scalar_t timeToNext = nextEventTime - t;
                if (timeToNext > 0.0 && timeToNext < kMergeToNominalThreshold) {
                    std::cerr << "[robust_splice_skip] leg=" << leg << " t=" << t
                              << " next=" << nextEventTime
                              << " (within " << kMergeToNominalThreshold << "s of nominal — merged)\n";
                    continue;
                }
            }

            curFlags[leg] = true;
            const size_t newMode = stanceLeg2ModeNumber(curFlags);

            // Insert stance-flipped phase at t; new mode covers [t, original next event).
            // ModeSchedule invariant: |eventTimes| = |modeSequence| - 1, with
            // modeSequence[i] applying to [eventTimes[i-1], eventTimes[i]).
            sched.eventTimes.insert(sched.eventTimes.begin() + curPhase, t);
            sched.modeSequence.insert(sched.modeSequence.begin() + curPhase + 1, newMode);

            // Forward propagation: walk forward and force leg=stance for any
            // consecutive original swing phases until we hit a phase where the
            // schedule already has the leg in stance (= the gait template's
            // natural next touchdown takes over). Without this, the original
            // gait pattern flips the leg back to swing at the very next event,
            // undoing the splice — only correct for gaits where the very next
            // phase already has the leg in stance (e.g. standing trot's
            // all-stance inter-step phase, where propagated_phases==1).
            size_t propagatedTo = curPhase + 1;
            for (size_t i = curPhase + 2; i < sched.modeSequence.size(); ++i) {
                contact_flag_t f = modeNumber2StanceLeg(sched.modeSequence[i]);
                if (f[leg]) break;          // original schedule already stance
                f[leg] = true;
                sched.modeSequence[i] = stanceLeg2ModeNumber(f);
                propagatedTo = i;
            }

            // g_event = n · (p_foot − p_plane) − foot_frame_offset, evaluated
            // at splice time using the previous cycle's robust window data
            // (the band the controller was reasoning against when it saw the
            // contact). Sign tells us where in [-d, +d] the contact landed:
            //   g_event > 0  → high-side hit (paper "early")
            //   g_event < 0  → low-side  hit (paper "late")
            // |g_event| ≤ d in the well-behaved case.
            scalar_t g_event = std::numeric_limits<scalar_t>::quiet_NaN();
            {
                std::lock_guard lock(robustWindowsMutex_);
                if (leg < robustWindows_.size()) {
                    const auto& w = robustWindows_[leg];
                    if (w.active) {
                        const vector3_t p_foot =
                            endEffectorKinematicsPtr_->getPosition(initState).at(leg);
                        g_event = w.n.dot(p_foot - w.p_plane) - w.foot_frame_offset;
                    }
                }
            }

            std::cerr << "[robust_contact_splice] leg=" << leg << " t=" << t
                      << " g_event=" << g_event
                      << " (>0 high-side / <0 low-side)"
                      << " mode_old=" << sched.modeSequence[curPhase]
                      << " mode_new=" << newMode
                      << " propagated_phases=" << (propagatedTo - curPhase) << "\n";
        }

        getGaitSchedule()->setModeSchedule(sched);
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
        loadData::loadPtreeValue(pt, s.P,                 prefix + "P",                 verbose);
        loadData::loadPtreeValue(pt, s.d,                 prefix + "d",                 verbose);
        loadData::loadPtreeValue(pt, s.terrain_source,    prefix + "terrain_source",    verbose);
        loadData::loadPtreeValue(pt, s.terrain_z_M1,      prefix + "terrain_z_M1",      verbose);
        loadData::loadPtreeValue(pt, s.foot_frame_offset, prefix + "foot_frame_offset", verbose);
        loadData::loadPtreeValue(pt, s.verbose_log,       prefix + "verbose_log",       verbose);
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
} // namespace legged

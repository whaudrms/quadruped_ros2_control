//
// Created by biao on 3/21/25.
//

#include <utility>
#include <cstdio>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    namespace
    {
        constexpr const char* kUncertaintyLayer = "uncertainty";
        constexpr const char* kDebugFilePath = "/tmp/height_only_reference_debug.log";
        constexpr scalar_t kDebugLogPeriod = 0.2;
    }

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
    }

    void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                            const vector_t& initState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
        const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
        const scalar_t queryTime = initTime + 0.5 * timeHorizon;
        const vector_t queryState = targetTrajectories.getDesiredState(queryTime);
        const vector_t queryPos = centroidal_model::getBasePose(queryState, info_).head(3);
        const grid_map::Position query(queryPos.x(), queryPos.y());
        const bool hasUncertainty = map.exists(kUncertaintyLayer) && map.isInside(query);
        const scalar_t uncertainty =
            hasUncertainty ? std::clamp(static_cast<scalar_t>(map.atPosition(kUncertaintyLayer, query)), scalar_t(0.0),
                                        scalar_t(1.0))
                           : scalar_t(0.0);

        static scalar_t lastDebugLogTime = -1.0;
        if (lastDebugLogTime < 0.0 || queryTime - lastDebugLogTime >= kDebugLogPeriod)
        {
            if (FILE* debugFile = std::fopen(kDebugFilePath, "a"))
            {
                std::fprintf(
                    debugFile,
                    "[HeightOnlyRef] init_t=%.3f query_t=%.3f pass_through=1 pos=(%.4f,%.4f,%.4f) uncertainty=%.5f smooth_height=%.5f com_height=%.5f\n",
                    static_cast<double>(initTime), static_cast<double>(queryTime),
                    static_cast<double>(queryPos.x()), static_cast<double>(queryPos.y()), static_cast<double>(queryPos.z()),
                    static_cast<double>(uncertainty),
                    map.isInside(query) ? static_cast<double>(map.atPosition("smooth_planar", query)) : 0.0,
                    static_cast<double>(comHeight_));
                std::fclose(debugFile);
            }
            lastDebugLogTime = queryTime;
        }

        // Footstep
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

            const auto terrainProjections = convexRegionSelectorPtr_->getProjections(leg);
            auto projections = terrainProjections;
            modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

            scalar_array_t liftOffHeights, touchDownHeights;
            // Heights must be computed from the original terrain projections.
            // `modifyProjections()` rewrites stance projections with the current
            // foot pose to preserve continuity, which is useful for execution but
            // would otherwise erase the intended terrain touchdown height.
            std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], terrainProjections);

            // For front-leg step-up motions, preserve an elevated touchdown target
            // directly from the terrain projection so the swing arc clears the box edge.
            if (leg < 2) {
                for (size_t phase = 0; phase < terrainProjections.size(); ++phase) {
                    if (!contactFlagStocks[leg][phase]) {
                        const scalar_t projectedHeight = terrainProjections[phase].positionInWorld.z();
                        if (projectedHeight > 0.02) {
                            touchDownHeights[phase] = std::max(touchDownHeights[phase], projectedHeight);
                        }
                    }
                }
            }
            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;

            if (!projections.empty())
            {
                if (FILE* debugFile = std::fopen(kDebugFilePath, "a"))
                {
                    const auto sampleIndex = std::min(initIndex, projections.size() - 1);
                    const auto projected = projections[sampleIndex].positionInWorld;
                    scalar_t maxTouchdown = 0.0;
                    int firstNonzeroTouchdown = -1;
                    scalar_t maxProjected = 0.0;
                    int firstSwingPhase = -1;
                    scalar_t firstSwingProjected = 0.0;
                    int firstTouchdownCandidate = -1;
                    scalar_t firstTouchdownProjected = 0.0;
                    for (size_t idx = 0; idx < touchDownHeights.size(); ++idx) {
                        if (touchDownHeights[idx] > maxTouchdown) {
                            maxTouchdown = touchDownHeights[idx];
                        }
                        if (firstNonzeroTouchdown < 0 && touchDownHeights[idx] > 0.0) {
                            firstNonzeroTouchdown = static_cast<int>(idx);
                        }
                        if (idx < terrainProjections.size()) {
                            const scalar_t projectedHeight = terrainProjections[idx].positionInWorld.z();
                            if (projectedHeight > maxProjected) {
                                maxProjected = projectedHeight;
                            }
                            if (firstSwingPhase < 0 && !contactFlagStocks[leg][idx]) {
                                firstSwingPhase = static_cast<int>(idx);
                                firstSwingProjected = projectedHeight;
                            }
                            if (idx + 1 < terrainProjections.size() && firstTouchdownCandidate < 0 &&
                                !contactFlagStocks[leg][idx] && contactFlagStocks[leg][idx + 1]) {
                                firstTouchdownCandidate = static_cast<int>(idx);
                                firstTouchdownProjected = terrainProjections[idx + 1].positionInWorld.z();
                            }
                        }
                    }
                    std::fprintf(
                        debugFile,
                        "[HeightOnlySwing] init_t=%.3f leg=%zu init_index=%zu projected=(%.4f,%.4f,%.4f) liftoff_h=%.5f touchdown_h=%.5f max_touchdown_h=%.5f first_nonzero_touchdown_idx=%d max_projected_z=%.5f first_swing_phase=%d first_swing_projected_z=%.5f first_touchdown_candidate=%d first_touchdown_projected_z=%.5f\n",
                        static_cast<double>(initTime), leg, sampleIndex,
                        static_cast<double>(projected.x()), static_cast<double>(projected.y()), static_cast<double>(projected.z()),
                        liftOffHeights.empty() ? 0.0 : static_cast<double>(liftOffHeights[sampleIndex]),
                        touchDownHeights.empty() ? 0.0 : static_cast<double>(touchDownHeights[sampleIndex]),
                        static_cast<double>(maxTouchdown), firstNonzeroTouchdown,
                        static_cast<double>(maxProjected), firstSwingPhase, static_cast<double>(firstSwingProjected),
                        firstTouchdownCandidate, static_cast<double>(firstTouchdownProjected));
                    std::fclose(debugFile);
                }
            }
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
        if (contactFlagStocks[initIndex])
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            for (int i = initIndex; i < projections.size(); ++i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
            for (int i = initIndex; i >= 0; --i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
        }
        if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg])
        {
            for (int i = initIndex; i >= 0; --i)
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
} // namespace legged

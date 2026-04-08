//
// Created by biao on 3/21/25.
//

#include <cmath>
#include <iostream>
#include <utility>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
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

        TargetTrajectories newTargetTrajectories;
        int nodeNum = 11;
        for (size_t i = 0; i < nodeNum; ++i)
        {
            scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
            vector_t state = targetTrajectories.getDesiredState(time);
            vector_t input = targetTrajectories.getDesiredInput(time);
            const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
            vector_t pos = centroidal_model::getBasePose(state, info_).head(3);
            const scalar_t desiredPlanarSpeed = state.head(2).norm();
            const scalar_t speedAlpha =
                std::clamp((desiredPlanarSpeed - scalar_t(0.02)) / scalar_t(0.08), scalar_t(0.0), scalar_t(1.0));
            const scalar_t holdBlend = scalar_t(0.25) + scalar_t(0.75) * speedAlpha;

            // Base Orientation
            scalar_t step = 0.3;
            const scalar_t heightXp = map.atPosition("smooth_planar", pos + grid_map::Position(step, 0));
            const scalar_t heightXm = map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0));
            const scalar_t heightYp = map.atPosition("smooth_planar", pos + grid_map::Position(0, step));
            const scalar_t heightYm = map.atPosition("smooth_planar", pos + grid_map::Position(0, -step));
            const scalar_t gradX = (heightXp - heightXm) / (2 * step);
            const scalar_t gradY = (heightYp - heightYm) / (2 * step);
            const scalar_t slopeNorm = std::sqrt(gradX * gradX + gradY * gradY);
            grid_map::Vector3 normalVector;
            normalVector(0) = -gradX;
            normalVector(1) = -gradY;
            normalVector(2) = 1;
            normalVector.normalize();
            matrix3_t R;
            scalar_t z = centroidal_model::getBasePose(state, info_)(3);
            R << cos(z), -sin(z), 0,
                 sin(z), cos(z), 0,
                 0, 0, 1;
            vector_t v = R.transpose() * normalVector;
            const scalar_t currentPitch = centroidal_model::getBasePose(state, info_)(4);
            const scalar_t terrainPitch = atan(v.x() / v.z());
            const scalar_t warmupDuration = 1.5;
            const scalar_t warmupAlpha = std::clamp((time - initTime) / warmupDuration, scalar_t(0.0), scalar_t(1.0));
            const scalar_t terrainPitchWeight = scalar_t(0.08) * warmupAlpha * holdBlend;
            const scalar_t blendedPitch =
                terrainPitchWeight * terrainPitch + (scalar_t(1.0) - terrainPitchWeight) * currentPitch;
            const scalar_t pitchLimit = (scalar_t(0.02) + warmupAlpha * scalar_t(0.015)) *
                (scalar_t(0.6) + scalar_t(0.4) * speedAlpha);
            const scalar_t limitedPitch = std::clamp(blendedPitch, -pitchLimit, pitchLimit);
            centroidal_model::getBasePose(state, info_)(4) = limitedPitch;

            // Base Z Position
            const scalar_t terrainHeight =
                map.atPosition("smooth_planar", pos) + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4)) - 0.01;
            const scalar_t baseZBefore = centroidal_model::getBasePose(state, info_)(2);
            const scalar_t zStepLimit =
                (scalar_t(0.03) + warmupAlpha * scalar_t(0.05)) * (scalar_t(0.55) + scalar_t(0.45) * speedAlpha);
            const scalar_t limitedTerrainHeight = std::min(baseZBefore + zStepLimit, terrainHeight);
            const scalar_t terrainHeightBlend = scalar_t(0.35) + scalar_t(0.65) * speedAlpha;
            const scalar_t blendedTerrainHeight =
                baseZBefore + terrainHeightBlend * (limitedTerrainHeight - baseZBefore);
            centroidal_model::getBasePose(state, info_)(2) =
                std::max(centroidal_model::getBasePose(state, info_)(2), blendedTerrainHeight);
            const scalar_t baseZAfter = centroidal_model::getBasePose(state, info_)(2);

            // Static uphill hold: when command is near zero, shift the body slightly uphill
            // so the support polygon does not sit behind the COM on sloped terrain patches.
            if (slopeNorm > scalar_t(1e-4))
            {
                const scalar_t uphillBiasScale = std::clamp(slopeNorm / scalar_t(0.08), scalar_t(0.0), scalar_t(1.0));
                const scalar_t uphillBias = scalar_t(0.015) * (scalar_t(1.0) - speedAlpha) * uphillBiasScale;
                centroidal_model::getBasePose(state, info_)(0) += uphillBias * gradX / slopeNorm;
                centroidal_model::getBasePose(state, info_)(1) += uphillBias * gradY / slopeNorm;
            }

            const bool suspiciousPitch = std::abs(terrainPitch) > scalar_t(0.03) || std::abs(blendedPitch - currentPitch) > scalar_t(0.01);
            const bool suspiciousHeight = (baseZAfter - baseZBefore) > scalar_t(0.015);
            if ((suspiciousPitch || suspiciousHeight) && ((debugCounter_++ % 25) == 0))
            {
                std::cerr << "[PerceptiveRefDebug] t=" << time
                          << " pos=(" << pos.x() << "," << pos.y() << "," << pos.z() << ")"
                          << " terrainPitch=" << terrainPitch
                          << " currentPitch=" << currentPitch
                          << " blendedPitch=" << blendedPitch
                          << " warmupAlpha=" << warmupAlpha
                          << " desiredPlanarSpeed=" << desiredPlanarSpeed
                          << " speedAlpha=" << speedAlpha
                          << " holdBlend=" << holdBlend
                          << " slopeNorm=" << slopeNorm
                          << " terrainPitchWeight=" << terrainPitchWeight
                          << " limitedPitch=" << limitedPitch
                          << " terrainHeight=" << terrainHeight
                          << " zStepLimit=" << zStepLimit
                          << " limitedTerrainHeight=" << limitedTerrainHeight
                          << " blendedTerrainHeight=" << blendedTerrainHeight
                          << " baseZBefore=" << baseZBefore
                          << " baseZAfter=" << baseZAfter
                          << std::endl;
            }

            newTargetTrajectories.timeTrajectory.push_back(time);
            newTargetTrajectories.stateTrajectory.push_back(state);
            newTargetTrajectories.inputTrajectory.push_back(input);
        }
        targetTrajectories = newTargetTrajectories;

        // Footstep
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

            auto projections = convexRegionSelectorPtr_->getProjections(leg);
            modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

            scalar_array_t liftOffHeights, touchDownHeights;
            std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], projections);
            scalar_array_t maxHeights(liftOffHeights.size());
            const scalar_t warmupDuration = 1.5;
            if ((leg == 2 || leg == 3) && initTime > warmupDuration)
            {
                for (size_t i = 0; i < touchDownHeights.size(); ++i)
                {
                    if (!contactFlagStocks[leg][i])
                    {
                        touchDownHeights[i] += 0.004;
                    }
                }
            }
            for (size_t i = 0; i < maxHeights.size(); ++i)
            {
                maxHeights[i] = std::max(liftOffHeights[i], touchDownHeights[i]);
                if (!contactFlagStocks[leg][i] && initTime > warmupDuration)
                {
                    const scalar_t upStep = touchDownHeights[i] - liftOffHeights[i];
                    if (upStep > scalar_t(0.01))
                    {
                        const scalar_t upStepBoost = std::min(scalar_t(0.008), scalar_t(0.5) * upStep);
                        maxHeights[i] += upStepBoost;
                    }
                }
            }
            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;
            maxHeightSequence[leg] = maxHeights;
        }
        swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
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

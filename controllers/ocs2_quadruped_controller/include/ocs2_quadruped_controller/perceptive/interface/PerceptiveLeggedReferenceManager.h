//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>
#include <mutex>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"

#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot
{
    class PerceptiveLeggedReferenceManager : public SwitchedModelReferenceManager
    {
    public:
        struct FootPlacementDebugInfo
        {
            scalar_t time = 0.0;
            contact_flag_t contactFlags{};
            contact_flag_t footPlacementFlags{};
            std::array<size_t, 4> polygonVertexCounts{};
            feet_array_t<scalar_t> projectionHeights{};
            feet_array_t<scalar_t> initStandFinalTimes{};
        };

        PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                         std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                         scalar_t comHeight);

        const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

        contact_flag_t getFootPlacementFlags(scalar_t time) const;

        void setEnableReferenceModification(bool enable) { enableReferenceModification_ = enable; }

        // Robust phase configuration (loaded from task.info `robustPhase` block by
        // PerceptiveLeggedInterface). Applied during modifyReferences().
        struct RobustPhaseSettings {
            bool      enabled = false;
            int       P = 5;            // window length in nodes
            scalar_t  d = 0.05;         // uncertainty half-width [m]
            // Terrain plane source for the per-leg robust window:
            //   "flat"          M1'' — n = e_z, p_plane.z = terrain_z_M1
            //   "convex_region" M2   — n = e_z (for now), p_plane = stance-side
            //                          projection from ConvexRegionSelector::getProjections(leg)
            //                          at the stance phase index (NOT getProjection(t_b),
            //                          which returns the swing-side at exact event boundaries).
            // Falls back to "flat" for any leg whose projection is null in convex_region mode.
            std::string terrain_source = "flat";
            scalar_t  terrain_z_M1 = 0.0;       // M1'' flat-ground guard reference
            scalar_t  foot_frame_offset = 0.0;  // FK foot-frame z above contact point along n
            scalar_t  dt_mpc = 0.015;           // SQP shooting interval [s]
            bool      verbose_log = false;      // emit per-cycle [robust_phase] std::cerr
        };
        void setRobustPhaseSettings(const RobustPhaseSettings& settings) { robustPhaseSettings_ = settings; }

        // Overrides from SwitchedModelReferenceManager.
        bool isInRobustWindow(size_t leg, scalar_t time) const override;
        RobustWindowData getRobustWindow(size_t leg) const override;

        // Track ② step (b/c) — splice requests routed through the reference
        // manager so that the actual ModeSchedule mutation runs INSIDE
        // modifyReferences AT THE START, BEFORE getGaitSchedule()->getModeSchedule(...)
        // on line 180. (Previously these lived in GaitManager::preSolverRun, which
        // ran AFTER modifyReferences per OCS2 SolverBase::preRun ordering — meaning
        // splice mutations were invisible to the same solve's reference work.)
        //
        // Both methods are callable from any thread (typically controller-thread
        // event detector in CtrlComponent::detectAndLogContactEvents). Queued
        // requests drain at the start of the next modifyReferences call.
        //
        //   requestStanceSplice — early contact: leg measured stance during
        //     scheduled swing; flips leg to stance from event_time forward, with
        //     forward propagation through subsequent swing phases until the gait
        //     template's natural touchdown.
        //   requestSwingSplice — late contact: leg scheduled stance but no
        //     measured contact; flips leg back to swing from event_time forward
        //     (touchdown delay), no forward propagation (gait template's natural
        //     cycle restores stance at next nominal touchdown).
        void requestStanceSplice(size_t leg, scalar_t event_time);
        void requestSwingSplice (size_t leg, scalar_t event_time);

        bool getLatestReferencePaths(
            std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& rawBasePath,
            std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& terrainAwareBasePath) const;

        bool getLatestFootPlacementDebugInfo(FootPlacementDebugInfo& debugInfo) const;

    protected:
        void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                              TargetTrajectories& targetTrajectories,
                              ModeSchedule& modeSchedule) override;

        virtual void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                  ModeSchedule& modeSchedule);

        void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                               const std::vector<bool>& contactFlagStocks,
                               std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

        std::pair<scalar_array_t, scalar_array_t> getHeights(const std::vector<bool>& contactFlagStocks,
                                                             const std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections);

        // Computes (per leg) the first-touchdown robust window for the current MPC cycle.
        // Called from modifyReferences() right after convexRegionSelectorPtr_->update(...).
        // For M1'': hard-coded n = e_z, p_plane.z = robustPhaseSettings_.terrain_z_M1.
        // For M2: pull (n, p_plane) from ConvexRegionSelector stance-side projection.
        void computeRobustWindows(scalar_t initTime, scalar_t finalTime, const ModeSchedule& modeSchedule,
                                  const vector_t& initState);

        // Drains pending splice requests and applies them to gait_schedule_ptr_.
        // Called at the START of modifyReferences (MPC thread, BEFORE the
        // line-180 getGaitSchedule()->getModeSchedule(...) read), so that the
        // subsequent reference-manager work (terrain projection, swing planner,
        // robust windows) all see the spliced schedule in the SAME solve cycle.
        // dt_mpc is needed for the near-touchdown guard (skip splice if event
        // time is within ~2*dt_mpc of nominal next event for that leg).
        void applyPendingSplices(scalar_t initTime, scalar_t finalTime);

        const CentroidalModelInfo info_;
        feet_array_t<vector3_t> lastLiftoffPos_;
        contact_flag_t previousContactFlags_{};
        feet_array_t<bool> hasLatchedContactPosition_{};
        feet_array_t<bool> activeSwingHeightLatched_{};
        feet_array_t<scalar_t> latchedSwingLiftOffHeights_{};
        feet_array_t<scalar_t> latchedSwingTouchDownHeights_{};

        std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        scalar_t comHeight_;
        bool enableReferenceModification_ = true;

        // Robust phase state (one window per leg, recomputed each MPC cycle).
        RobustPhaseSettings robustPhaseSettings_{};
        mutable std::mutex robustWindowsMutex_;
        feet_array_t<RobustWindowData> robustWindows_{};

        // Splice request queue (drained at start of modifyReferences). Two
        // independent queues: stance (early) and swing (late). Per-leg, at
        // most one pending request per type at a time (later request with
        // a later time is dropped — we anchor on the EARLIEST event for
        // that leg).
        std::mutex splice_mutex_;
        feet_array_t<bool>     stance_splice_pending_{};
        feet_array_t<scalar_t> stance_splice_time_{};
        feet_array_t<bool>     swing_splice_pending_{};
        feet_array_t<scalar_t> swing_splice_time_{};

        mutable std::mutex latestReferenceTrajectoriesMutex_;
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> latestRawBasePath_;
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> latestTerrainAwareBasePath_;
        FootPlacementDebugInfo latestFootPlacementDebugInfo_;
        bool hasLatestReferenceTrajectories_ = false;
    };

    // Free function: parse `robustPhase` block from task.info.
    // Defaults: enabled=false, P=5, d=0.05, terrain_z_M1=0.0, dt_mpc=0.015.
    // The caller is expected to override dt_mpc with the actual `sqp.dt` from task.info.
    PerceptiveLeggedReferenceManager::RobustPhaseSettings loadRobustPhaseSettings(
        const std::string& taskFile, bool verbose);
} // namespace legged

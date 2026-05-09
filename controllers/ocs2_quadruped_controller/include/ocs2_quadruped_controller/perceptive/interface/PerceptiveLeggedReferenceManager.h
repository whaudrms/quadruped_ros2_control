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
            // Impact-velocity lower bound: -v_max ≤ ġ. Used by
            // RobustGuardVelocityLowerBoundConstraint. Per chat7 priority #2;
            // must satisfy T_robust ≥ 2d/v_max (feasibility floor). Stored
            // per-window in RobustWindowData.v_max so constraints don't need
            // a separate settings handle.
            scalar_t  v_max = 0.6;
            bool      verbose_log = false;      // emit per-cycle [robust_phase] std::cerr
        };
        void setRobustPhaseSettings(const RobustPhaseSettings& settings) { robustPhaseSettings_ = settings; }

        // Overrides from SwitchedModelReferenceManager.
        bool isInRobustWindow(size_t leg, scalar_t time) const override;
        RobustWindowData getRobustWindow(size_t leg) const override;

        // Track ② — paper-faithful "robust-window contact event" splice.
        //
        // Concept (per Trajectory Optimization under Contact Timing Uncertainties,
        // §IV-V): inside the robust window t ∈ [t_a, t_b], the foot's normal
        // displacement g(x) traverses the band [+d, -d]. A measured contact in
        // this window — regardless of whether g_event is high-side (+d-ish, the
        // "early" case) or low-side (-d-ish, the "late" case) — is the event
        // that triggers a stance splice and an MPC replan. There is NO separate
        // "late→swing splice" in the paper's robust phase; that would be a
        // missed-touchdown fallback (out-of-band, t > t_b with no contact),
        // which is intentionally NOT implemented here.
        //
        // Callable from any thread (typically controller-thread event detector
        // in CtrlComponent::detectAndLogContactEvents). Queued requests drain
        // at the start of the next modifyReferences call.
        //
        //   requestRobustContactSplice — measured contact inside [t_a, t_b]
        //     while the schedule still says swing; flips leg to stance from
        //     event_time forward, with forward propagation through subsequent
        //     swing phases until the gait template's natural touchdown.
        //
        // Splice ordering: this method's effect is visible in the SAME MPC
        // solve because applyPendingSplices runs at the top of modifyReferences,
        // BEFORE the line-180 getModeSchedule(...) read that feeds terrain
        // projection / swing planner / robust-window computation.
        // (Previously this lived in GaitManager::preSolverRun, which runs AFTER
        // modifyReferences per OCS2 SolverBase::preRun:77-82 ordering — splice
        // mutations were invisible to the same solve's reference work.)
        void requestRobustContactSplice(size_t leg, scalar_t event_time);

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

        // Drains pending robust-contact-event splice requests and applies them
        // to gait_schedule_ptr_. Called at the START of modifyReferences (MPC
        // thread, BEFORE the line-180 getGaitSchedule()->getModeSchedule(...)
        // read), so the subsequent reference-manager work (terrain projection,
        // swing planner, robust windows) all see the spliced schedule in the
        // SAME solve cycle.
        //
        // initState is used to compute g_event = n·(p_foot − p_plane) −
        // foot_frame_offset at splice time for diagnostic logging. g_event > 0
        // means high-side hit (paper "early"), < 0 means low-side hit (paper
        // "late"); both go through the same splice path.
        //
        // Includes a "merge-to-nominal-touchdown" engineering guard: if the
        // event time is within ~2*dt_mpc of the leg's next nominal event,
        // skip the splice — inserting a sub-2-shoot phase destabilizes SQP
        // and WBC mode transitions. This is NOT part of the paper's robust
        // OCP — purely a numerical merge guard. Strict event-triggered MPC
        // would always insert; we trade a tiny modeling deviation for SQP
        // stability.
        void applyPendingSplices(scalar_t initTime, scalar_t finalTime, const vector_t& initState);

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

        // Robust-window contact event splice queue (drained at start of
        // modifyReferences). Per leg, at most one pending request at a time
        // (later request with a later time is dropped — we anchor on the
        // EARLIEST event for that leg, the moment of first measured contact).
        std::mutex splice_mutex_;
        feet_array_t<bool>     robust_contact_splice_pending_{};
        feet_array_t<scalar_t> robust_contact_splice_time_{};

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

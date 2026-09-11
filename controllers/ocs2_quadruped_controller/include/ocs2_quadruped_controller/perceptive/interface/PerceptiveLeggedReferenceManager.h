//
// Created by biao on 3/21/25.
//

#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/RobustPhaseTiming.h"

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

        // Immutable copy of one MPC reference update for the first upcoming
        // FL swing. StateOCS2 appends every new snapshot to a long-form CSV,
        // so a post-contact replan cannot overwrite the pre-contact plan.
        struct FootholdPlanSnapshot
        {
            uint64_t sequence = 0;
            scalar_t solveTime = 0.0;
            scalar_t horizonEndTime = 0.0;
            scalar_t liftOffTime = 0.0;
            scalar_t touchDownTime = 0.0;
            scalar_t touchDownHeight = 0.0;
            bool robustEnabled = false;
            RobustWindowData robustWindow{};
            scalar_array_t sampleTimes;
            scalar_array_t zReferences;
            scalar_array_t zVelocityReferences;
        };

        PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                         std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                         scalar_t comHeight);

        const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

        contact_flag_t getFootPlacementFlags(scalar_t time) const;

        void setEnableReferenceModification(bool enable) { enableReferenceModification_ = enable; }

        // Register before starting MPC. Called on the MPC thread after both
        // terrain selection and swing splines have been updated.
        void setVisualizationCallback(std::function<void(scalar_t, scalar_t, const ModeSchedule&)> callback)
        {
            visualizationCallback_ = std::move(callback);
        }

        // Robust phase configuration (loaded from task.info `robustPhase` block by
        // PerceptiveLeggedInterface). Applied during modifyReferences().
        struct RobustPhaseSettings {
            bool      enabled = false;
            scalar_t  t_a = 0.05;       // seconds BEFORE nominal touchdown
            scalar_t  t_b = 0.05;       // seconds AFTER nominal touchdown
            scalar_t  d = 0.05;         // fixed width / optimized-width initial guess [m]
            bool      optimize_d = false;
            scalar_t  d_min = 0.02;
            scalar_t  d_max = 0.05;
            // Add hard one-sided endpoint constraints independently alongside
            // the boundary cost, which always remains active at both endpoints.
            // Must be configured before startup.
            bool      hard_boundary_start = false;  // g(t_a) >= d
            bool      hard_boundary_end = false;    // g(t_b) <= -d
            // Quadratic-slack alternatives for the same one-sided inequalities.
            // For h >= 0, StateSoftConstraint + SquaredHingePenalty(mu, 0)
            // is equivalent to min_{s>=0} 0.5*mu*s^2 subject to h+s>=0.
            // Hard and slack may not both be enabled for the same endpoint.
            bool      slack_boundary_start = false;
            bool      slack_boundary_end = false;
            scalar_t  slack_boundary_weight_start = 200.0;
            scalar_t  slack_boundary_weight_end = 200.0;
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
            // v_max is derived per window as 2*d_max/(absolute end - start).
            // It stays constant as the MPC horizon advances into that window.
            // Ablation: when false, robust OCP (boundary, ġ envelope, ġ²
            // cost) stays active but the event-triggered schedule splice is
            // disabled — requestRobustContactSplice becomes a no-op. Useful
            // to isolate splice-induced gait disruption (per-leg splice can
            // create non-trot 3-leg stance modes) from the OCP-side robust
            // formulation. Default true preserves committed behavior.
            bool      enable_splice = true;
            bool      verbose_log = false;      // emit per-cycle [robust_phase] std::cerr
        };
        void setRobustPhaseSettings(const RobustPhaseSettings& settings) {
            validateRobustPhaseTiming(settings.t_a, settings.t_b, settings.d);
            if (!std::isfinite(settings.d_min) || !std::isfinite(settings.d_max) ||
                settings.d_min <= 0.0 || settings.d_min > settings.d_max ||
                settings.d < settings.d_min || settings.d > settings.d_max) {
                throw std::invalid_argument("robustPhase requires 0 < d_min <= d <= d_max (finite)");
            }
            robustPhaseSettings_ = settings;
            // Equal bounds are the fixed-width problem; avoid redundant zero-width inequalities.
            if (settings.d_min == settings.d_max) robustPhaseSettings_.optimize_d = false;
        }

        const RobustPhaseSettings& getRobustPhaseSettings() const { return robustPhaseSettings_; }

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
        // Capture this touchdown's window and queue a stance override confined
        // to [event_time, configured robust end). The next modifyReferences
        // applies it after touchdown delays and before reference computations.
        // GaitSchedule itself and other legs' event times remain unchanged.
        void requestRobustContactSplice(size_t leg, scalar_t event_time, const RobustWindowData& contactWindow);

        bool getLatestReferencePaths(
            std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& rawBasePath,
            std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& terrainAwareBasePath) const;

        bool getLatestFootPlacementDebugInfo(FootPlacementDebugInfo& debugInfo) const;

        bool getLatestFootholdPlanSnapshot(FootholdPlanSnapshot& snapshot) const;

        // Retrieve the reference snapshot belonging to a completed MPC policy
        // by the policy's initialization time. A bounded history prevents the
        // next solve's preSolverRun from racing ahead and replacing metadata.
        bool getFootholdPlanSnapshot(scalar_t solveTime, FootholdPlanSnapshot& snapshot) const;

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
                                  const std::vector<RobustTouchdownTiming>& timings);

        // Saves the first upcoming FL swing reference after the swing planner
        // and robust window have both been updated for the current MPC solve.
        void updateLatestFootholdPlanSnapshot(scalar_t initTime, scalar_t finalTime,
                                              const ModeSchedule& modeSchedule);

        // Drain requests and reapply bounded contact overrides to the delayed
        // schedule copy on every MPC solve. initState is only used for logging
        // guard displacement at apply time. Each leg retains its own event time.
        void applyPendingSplices(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                 ModeSchedule& modeSchedule);

        const CentroidalModelInfo info_;
        feet_array_t<vector3_t> lastLiftoffPos_;
        contact_flag_t previousContactFlags_{};
        feet_array_t<bool> hasLatchedContactPosition_{};
        feet_array_t<bool> activeSwingHeightLatched_{};
        feet_array_t<scalar_t> latchedSwingLiftOffHeights_{};
        feet_array_t<scalar_t> latchedSwingTouchDownHeights_{};
        feet_array_t<scalar_array_t> latestTouchDownHeightSequence_{};

        std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        scalar_t comHeight_;
        bool enableReferenceModification_ = true;
        std::function<void(scalar_t, scalar_t, const ModeSchedule&)> visualizationCallback_;

        // Robust phase state (one window per leg, recomputed each MPC cycle).
        RobustPhaseSettings robustPhaseSettings_{};
        mutable std::mutex robustWindowsMutex_;
        feet_array_t<RobustWindowData> robustWindows_{};

        // Requests capture original window bounds; only the queue is shared
        // with the controller thread. Persistent overlays belong to the MPC thread.
        std::mutex splice_mutex_;
        std::vector<RobustContactOverride> robustContactSplicePending_;
        std::vector<RobustContactOverride> robustContactOverrides_;

        mutable std::mutex latestReferenceTrajectoriesMutex_;
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> latestRawBasePath_;
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> latestTerrainAwareBasePath_;
        FootPlacementDebugInfo latestFootPlacementDebugInfo_;
        bool hasLatestReferenceTrajectories_ = false;

        mutable std::mutex footholdPlanSnapshotMutex_;
        FootholdPlanSnapshot latestFootholdPlanSnapshot_;
        std::deque<FootholdPlanSnapshot> footholdPlanSnapshotHistory_;
        uint64_t footholdPlanSnapshotSequence_ = 0;
        bool hasLatestFootholdPlanSnapshot_ = false;
    };

    // Free function: parse `robustPhase` block from task.info.
    // Defaults: enabled=false, t_a=t_b=0.05, d=0.05, terrain_z_M1=0.0, dt_mpc=0.015.
    // The caller is expected to override dt_mpc with the actual `sqp.dt` from task.info.
    PerceptiveLeggedReferenceManager::RobustPhaseSettings loadRobustPhaseSettings(
        const std::string& taskFile, bool verbose);
} // namespace legged

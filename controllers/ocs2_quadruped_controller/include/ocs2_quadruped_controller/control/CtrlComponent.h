//
// Created by biao on 3/15/25.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <atomic>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_quadruped_controller/estimator/StateEstimateBase.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h>
#include <ocs2_quadruped_controller/perceptive/visualize/SphereVisualization.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>

#include "TargetManager.h"


namespace ocs2
{
    class MPC_BASE;
    class MPC_MRT_Interface;
    class CentroidalModelRbdConversions;
}

namespace ocs2::legged_robot
{

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces);

        void setupStateEstimate(const std::string& estimator_type);
        void updateState(const rclcpp::Time& time, const rclcpp::Duration& period);
        void init();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::unique_ptr<LeggedInterface> legged_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        std::unique_ptr<LeggedRobotVisualizer> visualizer_;
        std::shared_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        std::shared_ptr<GaitManager> gait_manager_ptr_;

        SystemObservation observation_;
        vector_t measured_rbd_state_;
        std::atomic_bool mpc_running_{};

        bool verbose_ = false;

        std::string task_file_;
        std::string urdf_file_;
        std::string reference_file_;
        std::string gait_file_;

    private:
        void setupLeggedInterface();
        void setupMpc();
        void setupMrt();
        void alignPerceptiveBaseHeightToTerrain();
        void publishPerceptiveReferencePaths();
        void logPerceptiveFootPlacementDebug();
        std::optional<scalar_t> samplePerceptiveTerrainHeight(scalar_t x, scalar_t y) const;

        // Track ② step (b/c) — early & late contact detection + queued schedule
        // splice (no WBC override).
        //
        // Detection runs on the controller thread. Splice requests are routed
        // through the PerceptiveLeggedReferenceManager (NOT GaitManager, which
        // is a SolverSynchronizedModule that runs AFTER referenceManager in
        // OCS2 SolverBase::preRun — splicing there delays the schedule by one
        // MPC cycle). The reference manager drains the queue at the START of
        // its modifyReferences call, so the same MPC solve sees the spliced
        // schedule.
        //
        //   early: measured stance during scheduled swing inside robust window
        //          → after kEventSpliceSustainedTicks ticks, requestStanceSplice
        //          → schedule rewritten so leg is stance from event_time onward.
        //   late : scheduled stance with no measured contact (post-touchdown)
        //          → after kEventSpliceSustainedTicks ticks, requestSwingSplice
        //          → schedule rewritten so leg is swing from event_time onward
        //            (touchdown delay; gait template's natural cycle restores
        //            stance at the next nominal touchdown). If physical contact
        //            then arrives during the swing, the early-splice path picks
        //            it up and re-flips to stance — natural chaining.
        void detectAndLogContactEvents();

        // Sustained-tick thresholds for splice triggers (1 kHz controller).
        // Early: 5 ticks = 5 ms. Early contact during scheduled swing is an
        //   abnormal event (foot wasn't supposed to touch yet) — short
        //   debounce vs sensor noise is enough.
        // Late: 30 ticks = 30 ms. Late contact (scheduled stance, no measured
        //   contact) at the moment of touchdown is NORMAL physics — the foot
        //   may still be in transit from swing peak for several tens of ms
        //   even on flat terrain. Only AFTER ~30 ms past the rising edge does
        //   "no contact" become diagnostically meaningful as a real touchdown
        //   delay (e.g., perception-induced premature stance scheduling).
        static constexpr int kEventSpliceSustainedTicks     = 5;
        // ABLATION: late splice temporarily disabled (large threshold) to
        // isolate the effect of Change 1 (splice ordering) + Change 2 (near-
        // touchdown guard) before re-enabling Change 3 with a properly-tuned
        // late threshold.
        static constexpr int kLateSpliceSustainedTicks      = 100000;
        // prev_scheduled_contact_         — previous-tick scheduled flag per leg
        //                                   for rising/falling edge detection.
        // early_event_logged_in_swing_    — once-per-swing latch for the [robust_event]
        //                                   early log; resets on liftoff.
        // sustained_early_ticks_          — consecutive-tick counter for the
        //                                   stance splice trigger.
        // splice_requested_in_swing_      — once-per-swing latch for the stance
        //                                   splice request (resets on liftoff).
        // sustained_late_ticks_           — consecutive-tick counter for the
        //                                   swing splice trigger.
        // late_splice_requested_in_stance_ — once-per-stance latch for the
        //                                    swing splice request (resets on
        //                                    liftoff so the next stance is
        //                                    eligible again).
        feet_array_t<bool> prev_scheduled_contact_{};
        feet_array_t<bool> early_event_logged_in_swing_{};
        feet_array_t<int>  sustained_early_ticks_{};
        feet_array_t<bool> splice_requested_in_swing_{};
        feet_array_t<int>  sustained_late_ticks_{};
        feet_array_t<bool> late_splice_requested_in_stance_{};

        bool enable_perceptive_ = false;
        bool enable_perceptive_reference_modification_ = true;
        bool enable_perceptive_foot_placement_constraint_ = true;
        bool enable_perceptive_foot_collision_constraint_ = true;
        bool enable_perceptive_body_collision_constraint_ = false;
        scalar_t perceptive_foot_placement_boundary_margin_ = 0.05;
        std::string estimator_type_;
        scalar_t perceptive_com_height_ = 0.0;
        CtrlInterfaces& ctrl_interfaces_;
        std::unique_ptr<StateEstimateBase> estimator_;
        std::unique_ptr<CentroidalModelRbdConversions> rbd_conversions_;
        std::unique_ptr<TargetManager> target_manager_;
        nav_msgs::msg::Path pathFromBasePositions(
            const std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& basePositions,
            const rclcpp::Time& stamp) const;

        std::unique_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
        std::unique_ptr<SphereVisualization> sphereVisualizationPtr_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rawReferencePathPublisherPtr_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr terrainAwareReferencePathPublisherPtr_;
        scalar_t lastReferencePathPublishTime_ = std::numeric_limits<scalar_t>::lowest();
        scalar_t minReferencePathPublishTimeDifference_ = 0.1;
        scalar_t lastFootPlacementDebugLogTime_ = std::numeric_limits<scalar_t>::lowest();
        scalar_t minFootPlacementDebugLogTimeDifference_ = 0.5;

        std::vector<std::string> joint_names_;
        std::vector<std::string> feet_names_;
        std::string robot_pkg_;

        // Nonlinear MPC
        benchmark::RepeatedTimer mpc_timer_;
        std::thread mpc_thread_;
        std::atomic_bool controller_running_{};
    };
}


#endif //CTRLCOMPONENT_H

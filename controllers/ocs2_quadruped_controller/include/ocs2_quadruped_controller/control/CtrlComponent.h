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

        // Track ② step (a) — detection + log + schedule splice (no WBC override).
        // Compares per-leg measured contact (estimator → observation_.mode) against
        // scheduled contact (gait schedule). Logs at most one [robust_event] line per
        // (leg, type) per robust window. When sustained early contact is detected
        // (≥ kEventSpliceSustainedTicks consecutive control ticks while inside the
        // robust window), splices the gait schedule so that leg becomes stance from
        // observation_.time onwards (so the NEXT MPC solve plans the leg as stance).
        // The current MPC policy and the WBC are NOT modified — we deliberately do
        // not add a WBC contact-flag override (per chat4.md / chat6.md) so that the
        // splice's effect is attributable to the OCP-level event handling alone.
        void detectAndLogContactEvents();
        void spliceStanceForLeg(size_t leg);

        static constexpr int kEventSpliceSustainedTicks = 5;
        // prev_scheduled_contact_ is the previous-tick scheduled flag per leg, used
        // to detect rising / falling edges (touchdown / liftoff in the schedule).
        // early_event_logged_in_swing_ latches once per swing cycle and resets on
        // the leg's stance→swing transition (liftoff), so a single drawn-out
        // early-contact condition is logged once. Late events are inherently
        // rising-edge so they don't need a latch.
        // sustained_early_ticks_ counts consecutive ticks where the early-contact
        // condition holds; reset on any tick where the condition is false.
        // splice_applied_in_swing_ latches once per swing so we splice at most once
        // per swing cycle; reset on liftoff.
        feet_array_t<bool> prev_scheduled_contact_{};
        feet_array_t<bool> early_event_logged_in_swing_{};
        feet_array_t<int>  sustained_early_ticks_{};
        feet_array_t<bool> splice_applied_in_swing_{};

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

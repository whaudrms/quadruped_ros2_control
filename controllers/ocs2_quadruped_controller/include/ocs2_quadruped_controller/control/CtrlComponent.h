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

        // Track ② — paper-faithful "robust-window contact event" detection +
        // queued schedule splice (no WBC override).
        //
        // Concept (per Trajectory Optimization under Contact Timing
        // Uncertainties, §IV-V): inside the robust window t ∈ [t_a, t_b], a
        // measured contact — regardless of whether g_event is high-side
        // (g > 0, traditionally "early") or low-side (g < 0, traditionally
        // "late") within [-d, +d] — is the event that triggers a stance
        // splice and an MPC replan. Both cases are handled by the SAME
        // path here; they're just different points on the same band. The
        // [robust_event] log includes g_event so post-hoc you can see where
        // in the band the contact landed.
        //
        // NOT implemented here: missed-touchdown fallback (out-of-band,
        // t > t_b with no measured contact). That's a different concern
        // outside the paper's robust phase — it would be a separate design
        // (touchdown delay or similar). Earlier scaffolding for that path
        // was removed in this iteration to keep the core robust phase clean.
        //
        // Splice requests route through PerceptiveLeggedReferenceManager
        // (NOT GaitManager, which is a SolverSynchronizedModule running
        // AFTER the reference manager in OCS2 SolverBase::preRun — splicing
        // there delays the schedule by one MPC cycle). The reference
        // manager drains the queue at the START of its modifyReferences
        // call, so the same MPC solve sees the spliced schedule.
        void detectAndLogContactEvents();

        // 5 ticks = 5 ms at 1 kHz controller rate. Contact inside the
        // robust window while the schedule still says swing is by definition
        // abnormal (foot wasn't supposed to touch yet) — a short debounce
        // against single-tick sensor spikes is enough.
        static constexpr int kRobustContactSpliceSustainedTicks = 5;

        // prev_scheduled_contact_              — previous-tick scheduled flag per
        //                                        leg for rising/falling edge detection.
        // robust_contact_logged_in_window_     — once-per-window latch for the
        //                                        [robust_event] log; resets on liftoff.
        // sustained_robust_contact_ticks_      — consecutive-tick counter for the
        //                                        robust-contact splice trigger.
        // splice_requested_in_window_          — once-per-window latch so we send at
        //                                        most one splice request per swing
        //                                        cycle; resets on liftoff.
        feet_array_t<bool> prev_scheduled_contact_{};
        feet_array_t<bool> robust_contact_logged_in_window_{};
        feet_array_t<int>  sustained_robust_contact_ticks_{};
        feet_array_t<bool> splice_requested_in_window_{};

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

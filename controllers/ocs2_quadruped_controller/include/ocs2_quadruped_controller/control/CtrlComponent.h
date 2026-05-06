//
// Created by biao on 3/15/25.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <atomic>
#include <memory>
#include <limits>
#include <mutex>
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
#include <ocs2_quadruped_controller/control/MpcDumpRecorder.h>
#include <ocs2_quadruped_controller/control/RefinedPolicyReader.h>
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
        std::unique_ptr<MpcDumpRecorder> mpc_dump_recorder_;
        std::shared_ptr<GaitManager> gait_manager_ptr_;

        // Stage 8 — synchronous robust_refine IPC.
        // The MPC thread, after each advanceMpc(), dumps the just-solved
        // PrimalSolution to /dev/shm/robust_refine/in/cycle_<seq>.csv via
        // mpc_dump_recorder_, then blocks for the matching output file via
        // refined_policy_reader_. On success it stores the refined plan in
        // the members below so that StateOCS2::run() can override the WBC
        // tracking targets.
        std::unique_ptr<RefinedPolicyReader> refined_policy_reader_;
        std::atomic<size_t> refined_seq_{std::numeric_limits<size_t>::max()};
        std::mutex refined_mtx_;
        scalar_t refined_init_time_{};
        scalar_array_t refined_time_traj_;
        vector_array_t refined_state_traj_;
        vector_array_t refined_input_traj_;

        SystemObservation observation_;
        vector_t measured_rbd_state_;
        std::atomic_bool mpc_running_{};

        // Stage 9 — full-horizon 1-shot OCP mode.
        // When mpc_one_shot_ is true, init() runs N SQP iterations of advanceMpc()
        // (warm-starting from same observation/reference) to converge a single OCP,
        // dumps the policy once, then sets mpc_one_shot_done_. The MPC thread loop
        // then skips all subsequent advanceMpc()/dump calls but keeps polling
        // refined_policy_reader_ so the offline-refined plan can replace tracking.
        bool mpc_one_shot_ = false;
        int mpc_one_shot_solves_ = 10;
        std::atomic_bool mpc_one_shot_done_{false};

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

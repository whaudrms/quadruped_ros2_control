//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <chrono>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    namespace
    {
        size_t gObservationDebugCounter = 0;
        size_t gPolicyDebugCounter = 0;
        size_t gPreTargetManagerDebugCounter = 0;
        size_t gAdvanceMpcDebugCounter = 0;
        size_t gAdvanceMpcPolicyDebugCounter = 0;
    }

    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        node_->declare_parameter("robot_pkg", robot_pkg_);
        node_->declare_parameter("feet", feet_names_);
        node_->declare_parameter("enable_perceptive", enable_perceptive_);
        node_->declare_parameter("mpc_topic_prefix", mpc_topic_prefix_);
        node_->declare_parameter("task_file_override", std::string(""));
        node_->declare_parameter("reference_file_override", std::string(""));
        node_->declare_parameter("gait_file_override", std::string(""));

        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();
        mpc_topic_prefix_ = node_->get_parameter("mpc_topic_prefix").as_string();
        const auto task_file_override = node_->get_parameter("task_file_override").as_string();
        const auto reference_file_override = node_->get_parameter("reference_file_override").as_string();
        const auto gait_file_override = node_->get_parameter("gait_file_override").as_string();


        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";
        if (!task_file_override.empty()) {
            task_file_ = task_file_override;
        }
        if (!reference_file_override.empty()) {
            reference_file_ = reference_file_override;
        }
        if (!gait_file_override.empty()) {
            gait_file_ = gait_file_override;
        }

        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Init visualizer
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            mpc_topic_prefix_ + "_mpc_observation", 1);

        // Init observation
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;
    }

    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        if (estimator_type == "ground_truth")
        {
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else
        {
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Update State Estimation
        measured_rbd_state_ = estimator_->update(time, period);
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        observation_.mode = estimator_->getMode();

        if ((gObservationDebugCounter++ % 25) == 0)
        {
            const auto basePose = centroidal_model::getBasePose(observation_.state, legged_interface_->getCentroidalModelInfo());
            RCLCPP_INFO(
                node_->get_logger(),
                "[ObservationDebug] t=%.3f mode=%zu basePose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f) baseTwist=(%.3f, %.3f, %.3f)",
                observation_.time,
                observation_.mode,
                basePose[0], basePose[1], basePose[2], basePose[3], basePose[4], basePose[5],
                observation_.state[0], observation_.state[1], observation_.state[2]);
        }

        visualizer_->update(observation_);
        if (observation_publisher_ != nullptr)
        {
            observation_publisher_->publish(ocs2::ros_msg_conversions::createObservationMsg(observation_));
        }
        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_->update(observation_);
            sphereVisualizationPtr_->update(observation_);
        }

        // Compute target trajectory
        if ((gPreTargetManagerDebugCounter++ % 25) == 0)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "[PreTargetManagerDebug] t=%.3f command=%d lx=%.3f ly=%.3f rx=%.3f ry=%.3f",
                observation_.time,
                ctrl_interfaces_.control_inputs_.command,
                ctrl_interfaces_.control_inputs_.lx,
                ctrl_interfaces_.control_inputs_.ly,
                ctrl_interfaces_.control_inputs_.rx,
                ctrl_interfaces_.control_inputs_.ry);
        }
        target_manager_->update(observation_);
        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);

        if (mpc_mrt_interface_->initialPolicyReceived() && (gPolicyDebugCounter++ % 40) == 0)
        {
            try
            {
                const auto& policy = mpc_mrt_interface_->getPolicy();
                if (!policy.timeTrajectory_.empty() && !policy.stateTrajectory_.empty() && !policy.inputTrajectory_.empty())
                {
                    const auto& x0 = policy.stateTrajectory_.front();
                    const auto& u0 = policy.inputTrajectory_.front();
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[PolicyDebug] N=%zu t0=%.3f x0_xyz=(%.3f, %.3f, %.3f) x0_rpy=(%.3f, %.3f, %.3f) u0_xyz=(%.3f, %.3f, %.3f)",
                        policy.timeTrajectory_.size(),
                        policy.timeTrajectory_.front(),
                        x0[6], x0[7], x0[8],
                        x0[9], x0[10], x0[11],
                        u0[0], u0[1], u0[2]);
                }
            }
            catch (const std::runtime_error& e)
            {
                RCLCPP_WARN(node_->get_logger(), "[PolicyDebug] skipped: %s", e.what());
            }
        }
    }

    void CtrlComponent::init()
    {
        if (mpc_running_ == false)
        {
            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                const auto tic = std::chrono::steady_clock::now();
                mpc_mrt_interface_->advanceMpc();
                const auto toc = std::chrono::steady_clock::now();
                const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
                if (dt_ms > 20)
                {
                    RCLCPP_INFO(node_->get_logger(), "[AdvanceMpcDebug] init advanceMpc dt_ms=%ld", dt_ms);
                }
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
        }
        else
        {
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        }

        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

            sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                *dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr(), node_);
        }
    }

    /**
     * Set up the SQP MPC, Gait Manager and Reference Manager
     */
    void CtrlComponent::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule());
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        ros_reference_manager_ = std::make_shared<ocs2::RosReferenceManager>(
            mpc_topic_prefix_, legged_interface_->getReferenceManagerPtr());
        ros_reference_manager_->subscribe(node_);
        mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_);

        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          ros_reference_manager_,
                                                          task_file_,
                                                          reference_file_);

        if (enable_perceptive_)
        {
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    void CtrlComponent::setupMrt()
    {
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                double policy_t0_before = std::numeric_limits<double>::quiet_NaN();
                                if (mpc_mrt_interface_->initialPolicyReceived())
                                {
                                    try
                                    {
                                        const auto& policy = mpc_mrt_interface_->getPolicy();
                                        if (!policy.timeTrajectory_.empty())
                                        {
                                            policy_t0_before = policy.timeTrajectory_.front();
                                        }
                                    }
                                    catch (const std::runtime_error&)
                                    {
                                    }
                                }

                                mpc_timer_.startTimer();
                                const auto tic = std::chrono::steady_clock::now();
                                mpc_mrt_interface_->advanceMpc();
                                const auto toc = std::chrono::steady_clock::now();
                                mpc_timer_.endTimer();
                                const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

                                double policy_t0_after = std::numeric_limits<double>::quiet_NaN();
                                if (mpc_mrt_interface_->initialPolicyReceived())
                                {
                                    try
                                    {
                                        const auto& policy = mpc_mrt_interface_->getPolicy();
                                        if (!policy.timeTrajectory_.empty())
                                        {
                                            policy_t0_after = policy.timeTrajectory_.front();
                                        }
                                    }
                                    catch (const std::runtime_error&)
                                    {
                                    }
                                }

                                if (dt_ms > 20 || (gAdvanceMpcDebugCounter++ % 100) == 0)
                                {
                                    RCLCPP_INFO(
                                        node_->get_logger(),
                                        "[AdvanceMpcDebug] obs_t=%.3f dt_ms=%ld mpc_running=%d",
                                        observation_.time,
                                        dt_ms,
                                        static_cast<int>(mpc_running_));
                                }

                                const bool stalePolicy = std::isfinite(policy_t0_before) && std::isfinite(policy_t0_after) &&
                                                         std::abs(policy_t0_after - policy_t0_before) < 1e-9;
                                if ((dt_ms > 20 && stalePolicy) || (gAdvanceMpcPolicyDebugCounter++ % 100) == 0)
                                {
                                    RCLCPP_INFO(
                                        node_->get_logger(),
                                        "[AdvanceMpcPolicyDebug] obs_t=%.3f dt_ms=%ld policy_t0_before=%.3f policy_t0_after=%.3f stale=%d",
                                        observation_.time,
                                        dt_ms,
                                        policy_t0_before,
                                        policy_t0_after,
                                        static_cast<int>(stalePolicy));
                                }
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                }
                catch (const std::exception& e)
                {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }
}

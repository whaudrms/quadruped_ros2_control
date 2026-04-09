//
// Created by tlab-uav on 24-9-30.
//

#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        scalar_t applyDeadband(const scalar_t value, const scalar_t threshold)
        {
            return std::abs(value) < threshold ? 0.0 : value;
        }
    } // namespace

    TargetManager::TargetManager(CtrlInterfaces& ctrl_component,
                                 rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                 const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                                 const std::string& task_file,
                                 const std::string& reference_file)
        : ctrl_component_(ctrl_component),
          referenceManagerPtr_(referenceManagerPtr),
          node_(std::move(node))
    {
        default_joint_state_ = vector_t::Zero(12);
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_);
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);
        try
        {
            loadData::loadCppDataType(task_file, "model_settings.startupVelocityRampDuration", startup_velocity_ramp_duration_);
        }
        catch (...)
        {
        }
        try
        {
            loadData::loadCppDataType(task_file, "model_settings.startupForwardBiasDuration", startup_forward_bias_duration_);
        }
        catch (...)
        {
        }
        try
        {
            loadData::loadCppDataType(task_file, "model_settings.startupForwardBiasVelocity", startup_forward_bias_velocity_);
        }
        catch (...)
        {
        }

        twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                buffer_.writeFromNonRT(*msg);
                twist_count = ctrl_component_.frequency_ / 5;
                RCLCPP_INFO(node_->get_logger(), "Twist count: %i", twist_count);
            });
    }

    void TargetManager::update(SystemObservation& observation)
    {
        const int currentCommand = ctrl_component_.control_inputs_.command;
        if (currentCommand == 3 && last_command_ != 3)
        {
            gait_activation_time_ = observation.time;
        }
        last_command_ = currentCommand;

        scalar_t startupRampScale = 1.0;
        if (gait_activation_time_ >= 0.0)
        {
            const scalar_t elapsedSinceGaitActivation = observation.time - gait_activation_time_;
            if (elapsedSinceGaitActivation < startup_velocity_ramp_duration_)
            {
                startupRampScale = std::clamp(
                    elapsedSinceGaitActivation / std::max<scalar_t>(startup_velocity_ramp_duration_, 1e-3),
                    scalar_t(0.0), scalar_t(1.0));
            }
        }
        const bool useStartupForwardBias =
            gait_activation_time_ >= 0.0 &&
            (observation.time - gait_activation_time_) >= 0.0 &&
            (observation.time - gait_activation_time_) < startup_forward_bias_duration_;

        vector_t cmdGoal = vector_t::Zero(6);
        constexpr scalar_t kInputDeadband = 0.05;
        if (buffer_.readFromRT() == nullptr || twist_count <= 0)
        {
            const scalar_t ly = applyDeadband(ctrl_component_.control_inputs_.ly, kInputDeadband);
            const scalar_t lx = applyDeadband(ctrl_component_.control_inputs_.lx, kInputDeadband);
            const scalar_t ry = applyDeadband(ctrl_component_.control_inputs_.ry, kInputDeadband);
            const scalar_t rx = applyDeadband(ctrl_component_.control_inputs_.rx, kInputDeadband);

            cmdGoal[0] = startupRampScale * ly * target_displacement_velocity_;
            cmdGoal[1] = startupRampScale * -lx * target_displacement_velocity_;
            cmdGoal[2] = ry;
            cmdGoal[3] = -rx * target_rotation_velocity_;
            if (useStartupForwardBias)
            {
                cmdGoal[0] = std::max(cmdGoal[0], startup_forward_bias_velocity_);
            }
        }
        else
        {
            const geometry_msgs::msg::Twist twist = *buffer_.readFromRT();
            cmdGoal[0] = applyDeadband(twist.linear.x, kInputDeadband);
            cmdGoal[1] = applyDeadband(twist.linear.y, kInputDeadband);
            cmdGoal[2] = 0;
            cmdGoal[3] = applyDeadband(twist.angular.z, kInputDeadband);
            twist_count--;
            if (twist_count <= 0)
            {
                buffer_.reset();
            }
        }

        const vector_t currentPose = observation.state.segment<6>(6);
        const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
        vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdGoal.head(3);

        const vector_t targetPose = [&]
        {
            vector_t target(6);
            target(0) = currentPose(0) + cmd_vel_rot(0) * time_to_target_;
            target(1) = currentPose(1) + cmd_vel_rot(1) * time_to_target_;
            target(2) = command_height_;
            target(3) = currentPose(3) + cmdGoal(3) * time_to_target_;
            target(4) = 0;
            target(5) = 0;
            return target;
        }();

        const scalar_t targetReachingTime = observation.time + time_to_target_;
        auto trajectories =
            targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
        trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;

        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }
}

//
// Created by tlab-uav on 24-9-30.
//

#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        size_t gTargetManagerDebugCounter = 0;

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

        node_->declare_parameter("use_external_target_topic", use_external_target_topic_);
        node_->declare_parameter("external_target_takeover_time", external_target_takeover_time_);
        node_->declare_parameter("use_map_trajectory", use_map_trajectory_);
        node_->declare_parameter("map_trajectory_file", map_trajectory_file_);
        use_external_target_topic_ = node_->get_parameter("use_external_target_topic").as_bool();
        external_target_takeover_time_ = node_->get_parameter("external_target_takeover_time").as_double();
        use_map_trajectory_ = node_->get_parameter("use_map_trajectory").as_bool();
        map_trajectory_file_ = node_->get_parameter("map_trajectory_file").as_string();

        if (use_map_trajectory_)
        {
            std::filesystem::path trajectoryPath(map_trajectory_file_);
            if (trajectoryPath.is_relative())
            {
                trajectoryPath = std::filesystem::path(reference_file).parent_path() / trajectoryPath;
            }

            if (!loadTrajectoryFile(trajectoryPath.string()))
            {
                throw std::runtime_error("Failed to load map trajectory file: " + trajectoryPath.string());
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "[TargetManager] Map trajectory tracking enabled. file=%s num_waypoints=%zu",
                trajectoryPath.c_str(),
                map_trajectory_.size());
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
        const bool externalTargetActive =
            use_external_target_topic_ && observation.time >= external_target_takeover_time_;

        if (externalTargetActive)
        {
            if ((gTargetManagerDebugCounter++ % 25) == 0)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "[TargetManagerDebug] t=%.3f external target topic active after takeover_time=%.3f, internal target update skipped",
                    observation.time,
                    external_target_takeover_time_);
            }
            return;
        }

        if (use_map_trajectory_)
        {
            auto trajectories = trajectoryFileToTargetTrajectories(observation);
            if ((gTargetManagerDebugCounter++ % 25) == 0 && !trajectories.timeTrajectory.empty())
            {
                const auto& first = trajectories.stateTrajectory.front();
                const auto& last = trajectories.stateTrajectory.back();
                RCLCPP_INFO(
                    node_->get_logger(),
                    "[TargetManagerDebug] t=%.3f mapTrajectory waypoints=%zu firstPose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f) lastPose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                    observation.time,
                    trajectories.timeTrajectory.size(),
                    first[6], first[7], first[8], first[9], first[10], first[11],
                    last[6], last[7], last[8], last[9], last[10], last[11]);
            }
            referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
            return;
        }

        vector_t cmdGoal = vector_t::Zero(6);
        constexpr scalar_t kInputDeadband = 0.05;
        if (buffer_.readFromRT() == nullptr || twist_count <= 0)
        {
            const scalar_t ly = applyDeadband(ctrl_component_.control_inputs_.ly, kInputDeadband);
            const scalar_t lx = applyDeadband(ctrl_component_.control_inputs_.lx, kInputDeadband);
            const scalar_t ry = applyDeadband(ctrl_component_.control_inputs_.ry, kInputDeadband);
            const scalar_t rx = applyDeadband(ctrl_component_.control_inputs_.rx, kInputDeadband);

            cmdGoal[0] = ly * target_displacement_velocity_;
            cmdGoal[1] = -lx * target_displacement_velocity_;
            cmdGoal[2] = ry;
            cmdGoal[3] = -rx * target_rotation_velocity_;
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

        if ((gTargetManagerDebugCounter++ % 25) == 0)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "[TargetManagerDebug] t=%.3f rawInput=(cmd=%d lx=%.3f ly=%.3f rx=%.3f ry=%.3f) targetScale=(disp=%.3f rot=%.3f) cmdGoal=(%.3f, %.3f, %.3f, %.3f) cmdVelRot=(%.3f, %.3f, %.3f) currentPose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f) targetPose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                observation.time,
                ctrl_component_.control_inputs_.command,
                ctrl_component_.control_inputs_.lx,
                ctrl_component_.control_inputs_.ly,
                ctrl_component_.control_inputs_.rx,
                ctrl_component_.control_inputs_.ry,
                target_displacement_velocity_,
                target_rotation_velocity_,
                cmdGoal[0], cmdGoal[1], cmdGoal[2], cmdGoal[3],
                cmd_vel_rot[0], cmd_vel_rot[1], cmd_vel_rot[2],
                currentPose[0], currentPose[1], currentPose[2], currentPose[3], currentPose[4], currentPose[5],
                targetPose[0], targetPose[1], targetPose[2], targetPose[3], targetPose[4], targetPose[5]);
        }

        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }

    void TargetManager::resetMapTrajectoryTracking()
    {
        map_trajectory_initialized_ = false;
        cached_map_target_trajectories_ = TargetTrajectories();
    }

    bool TargetManager::loadTrajectoryFile(const std::string& trajectoryFilePath)
    {
        std::ifstream file(trajectoryFilePath);
        if (!file.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), "[TargetManager] Failed to open trajectory file: %s", trajectoryFilePath.c_str());
            return false;
        }

        map_trajectory_.clear();
        std::string line;
        size_t lineNumber = 0;
        while (std::getline(file, line))
        {
            ++lineNumber;
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            std::istringstream iss(line);
            TrajectoryWaypoint waypoint;
            if (!(iss >> waypoint.time >> waypoint.x >> waypoint.y >> waypoint.z >> waypoint.yaw))
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "[TargetManager] Invalid trajectory row at line %zu in %s",
                    lineNumber,
                    trajectoryFilePath.c_str());
                map_trajectory_.clear();
                return false;
            }

            if (!(iss >> waypoint.pitch))
            {
                waypoint.pitch = 0.0;
            }
            if (!(iss >> waypoint.roll))
            {
                waypoint.roll = 0.0;
            }

            map_trajectory_.push_back(waypoint);
        }

        std::sort(map_trajectory_.begin(), map_trajectory_.end(),
                  [](const TrajectoryWaypoint& a, const TrajectoryWaypoint& b) { return a.time < b.time; });

        return map_trajectory_.size() >= 2;
    }

    TargetTrajectories TargetManager::trajectoryFileToTargetTrajectories(const SystemObservation& observation) const
    {
        if (!map_trajectory_initialized_)
        {
            scalar_array_t timeTrajectory;
            vector_array_t stateTrajectory;
            vector_array_t inputTrajectory;

            map_trajectory_start_time_ = observation.time;
            map_trajectory_start_pose_ = observation.state.segment<6>(6);

            const scalar_t xOffset = map_trajectory_start_pose_[0] - map_trajectory_.front().x;
            const scalar_t yOffset = map_trajectory_start_pose_[1] - map_trajectory_.front().y;
            const scalar_t zOffset = command_height_ - map_trajectory_.front().z;
            const scalar_t yawOffset = map_trajectory_start_pose_[3] - map_trajectory_.front().yaw;

            for (size_t i = 0; i < map_trajectory_.size(); ++i)
            {
                const auto& wp = map_trajectory_[i];
                const auto& next = map_trajectory_[std::min(i + 1, map_trajectory_.size() - 1)];
                const scalar_t dt = std::max(next.time - wp.time, scalar_t(1e-3));

                vector_t state = vector_t::Zero(observation.state.size());
                state[0] = (next.x - wp.x) / dt;
                state[1] = (next.y - wp.y) / dt;
                state[2] = (next.z - wp.z) / dt;
                state[6] = wp.x + xOffset;
                state[7] = wp.y + yOffset;
                state[8] = wp.z + zOffset;
                state[9] = wp.yaw + yawOffset;
                state[10] = wp.pitch;
                state[11] = wp.roll;
                state.tail(default_joint_state_.size()) = default_joint_state_;

                timeTrajectory.push_back(map_trajectory_start_time_ + wp.time);
                stateTrajectory.push_back(std::move(state));
                inputTrajectory.push_back(vector_t::Zero(observation.input.size()));
            }

            cached_map_target_trajectories_ = {timeTrajectory, stateTrajectory, inputTrajectory};
            map_trajectory_initialized_ = true;
        }

        return cached_map_target_trajectories_;
    }
}

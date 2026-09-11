//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <mutex>

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        if (!node_->has_parameter("default_kp"))
            node_->declare_parameter("default_kp", default_kp_);
        if (!node_->has_parameter("default_kd"))
            node_->declare_parameter("default_kd", default_kd_);
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();

        // Stage 8 — per-tick analysis log
        if (!node_->has_parameter("tick_log_path"))
            node_->declare_parameter<std::string>("tick_log_path", "");
        tick_log_path_ = node_->get_parameter("tick_log_path").as_string();
        if (!tick_log_path_.empty())
        {
            tick_log_.open(tick_log_path_);
            if (tick_log_.is_open())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[StateOCS2] tick CSV log → '%s'", tick_log_path_.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[StateOCS2] failed to open tick_log_path '%s' — disabled",
                            tick_log_path_.c_str());
            }
        }

        if (!node_->has_parameter("foothold_plan_log_path"))
            node_->declare_parameter<std::string>("foothold_plan_log_path", "");
        foothold_plan_log_path_ = node_->get_parameter("foothold_plan_log_path").as_string();
        if (!foothold_plan_log_path_.empty())
        {
            foothold_plan_log_.open(foothold_plan_log_path_);
            if (foothold_plan_log_.is_open())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[StateOCS2] foothold-plan CSV log → '%s'",
                            foothold_plan_log_path_.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[StateOCS2] failed to open foothold_plan_log_path '%s' — disabled",
                            foothold_plan_log_path_.c_str());
            }
        }

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::enter()
    {
        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& /**time**/,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        const bool policyUpdated = ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Append any new MPC-side FL swing plan before evaluating the policy
        // for WBC. The snapshot sequence changes once per reference update,
        // not once per high-rate controller tick.
        if (policyUpdated)
        {
            // Optional diagnostics must never stop the control path.
            try
            {
                logLatestFootholdPlanSnapshot();
            }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[StateOCS2] foothold-plan logger disabled: %s",
                             error.what());
                foothold_plan_log_.close();
            }
            catch (...)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[StateOCS2] foothold-plan logger disabled by an unknown error");
                foothold_plan_log_.close();
            }
        }

        if (ctrl_component_->robustParameterCount_ > 0) {
            const auto& policy = ctrl_component_->mpc_mrt_interface_->getPolicy();
            const vector_t nominal = LinearInterpolation::interpolate(
                ctrl_component_->observation_.time, policy.timeTrajectory_, policy.stateTrajectory_);
            ctrl_component_->robustWidthSeed_ = nominal.tail(ctrl_component_->robustParameterCount_);
        }
        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->getMpcObservation().state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);

        const vector_t optimizedWidths = optimized_state_.tail(ctrl_component_->robustParameterCount_);
        optimized_state_.conservativeResize(ctrl_component_->observation_.state.size());

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();
        ctrl_component_->logRobustStanceEntry(
            planned_mode, ctrl_component_->mpc_mrt_interface_->getPolicy().modeSchedule_);

        // Per-tick CSV log (only when tick_log_ is open).
        // Layout: t,opt_state[0..23],opt_input[0..23],meas_rbd[0..35],planned_mode,
        //         measured_mode,wbc_solve_ms,control_period_s
        // measured_rbd_state has shape: [theta_zyx(3), r_b(3), q_j(12), w_b(3), r_b_dot(3), q_j_dot(12)] = 36
        if (tick_log_.is_open())
        {
            if (!tick_log_header_written_)
            {
                tick_log_ << "t";
                for (int i = 0; i < 24; ++i) tick_log_ << ",opt_x" << i;
                for (int i = 0; i < 24; ++i) tick_log_ << ",opt_u" << i;
                for (int i = 0;
                     i < static_cast<int>(ctrl_component_->measured_rbd_state_.size()); ++i)
                    tick_log_ << ",meas_rbd" << i;
                tick_log_ << ",planned_mode,measured_mode,wbc_solve_ms,control_period_s";
                for (size_t leg = 0; leg < ctrl_component_->robustParameterCount_; ++leg)
                    tick_log_ << ",opt_d" << leg;
                tick_log_ << "\n";
                tick_log_header_written_ = true;
            }
            tick_log_ << ctrl_component_->observation_.time;
            for (int i = 0; i < optimized_state_.size(); ++i)
                tick_log_ << "," << optimized_state_(i);
            for (int i = 0; i < optimized_input_.size(); ++i)
                tick_log_ << "," << optimized_input_(i);
            for (int i = 0; i < ctrl_component_->measured_rbd_state_.size(); ++i)
                tick_log_ << "," << ctrl_component_->measured_rbd_state_(i);
            tick_log_ << "," << planned_mode
                      << "," << ctrl_component_->observation_.mode
                      << "," << wbc_timer_.getLastIntervalInMilliseconds()
                      << "," << period.seconds();
            for (Eigen::Index leg = 0; leg < optimizedWidths.size(); ++leg) tick_log_ << "," << optimizedWidths(leg);
            tick_log_ << "\n";
        }

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        for (int i = 0; i < 12; i++)
        {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        const auto& policy = ctrl_component_->mpc_mrt_interface_->getPolicy();
        if (ctrl_component_->robustParameterCount_ > 0) {
            // Pinocchio visualization also expects the physical state. Cache
            // the stripped trajectory once per policy, without cloning feedback.
            if (policyUpdated || physical_visualization_policy_.timeTrajectory_.empty()) {
                physical_visualization_policy_.timeTrajectory_ = policy.timeTrajectory_;
                physical_visualization_policy_.stateTrajectory_ = policy.stateTrajectory_;
                physical_visualization_policy_.modeSchedule_ = policy.modeSchedule_;
                for (auto& state : physical_visualization_policy_.stateTrajectory_)
                    state.conservativeResize(ctrl_component_->observation_.state.size());
            }
            ctrl_component_->visualizer_->update(physical_visualization_policy_,
                                                 ctrl_component_->mpc_mrt_interface_->getCommand());
        } else {
            ctrl_component_->visualizer_->update(policy, ctrl_component_->mpc_mrt_interface_->getCommand());
        }
    }

    void StateOCS2::exit()
    {
    }

    void StateOCS2::logLatestFootholdPlanSnapshot()
    {
        if (!foothold_plan_log_.is_open())
        {
            return;
        }

        auto* referenceManager = dynamic_cast<PerceptiveLeggedReferenceManager*>(
            ctrl_component_->legged_interface_->getReferenceManagerPtr().get());
        if (referenceManager == nullptr)
        {
            return;
        }

        const auto& policy = ctrl_component_->mpc_mrt_interface_->getPolicy();
        const scalar_t policyStartTime =
            ctrl_component_->mpc_mrt_interface_->getCommand().mpcInitObservation_.time;
        if (policy.timeTrajectory_.empty() || policy.stateTrajectory_.empty())
        {
            return;
        }

        PerceptiveLeggedReferenceManager::FootholdPlanSnapshot snapshot;
        if (!referenceManager->getFootholdPlanSnapshot(policyStartTime, snapshot) ||
            snapshot.sequence == last_foothold_plan_sequence_)
        {
            return;
        }
        last_foothold_plan_sequence_ = snapshot.sequence;

        if (!foothold_plan_log_header_written_)
        {
            foothold_plan_log_
                << "snapshot_id,solve_time,horizon_end_time,liftoff_time,touchdown_time,"
                   "touchdown_height,robust_enabled,window_active,window_ta,window_tb,"
                   "plane_z,normal_x,normal_y,normal_z,d,foot_frame_offset,"
                   "policy_start_time,sample_index,sample_time,z_ref,z_dot_ref";
            for (size_t stateIndex = 0; stateIndex < 24; ++stateIndex)
            {
                foothold_plan_log_ << ",opt_x" << stateIndex;
            }
            foothold_plan_log_ << ",optimized_mode,opt_d0,opt_d1,opt_d2,opt_d3,v_max,d_init\n";
            foothold_plan_log_header_written_ = true;
        }

        foothold_plan_log_ << std::setprecision(17);
        const size_t sampleCount = std::min(
            snapshot.sampleTimes.size(),
            std::min(snapshot.zReferences.size(), snapshot.zVelocityReferences.size()));
        for (size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
        {
            const auto& window = snapshot.robustWindow;
            const scalar_t sampleTime = snapshot.sampleTimes[sampleIndex];
            const vector_t optimizedState = LinearInterpolation::interpolate(
                sampleTime, policy.timeTrajectory_, policy.stateTrajectory_);
            const size_t optimizedMode = policy.modeSchedule_.modeAtTime(sampleTime);
            foothold_plan_log_
                << snapshot.sequence
                << "," << snapshot.solveTime
                << "," << snapshot.horizonEndTime
                << "," << snapshot.liftOffTime
                << "," << snapshot.touchDownTime
                << "," << snapshot.touchDownHeight
                << "," << (snapshot.robustEnabled ? 1 : 0)
                << "," << (window.active ? 1 : 0)
                << "," << window.t_a
                << "," << window.t_b
                << "," << window.p_plane.z()
                << "," << window.n.x()
                << "," << window.n.y()
                << "," << window.n.z()
                << "," << (window.d_state_index >= 0 ? optimizedState(window.d_state_index) : window.d)
                << "," << window.foot_frame_offset
                << "," << policyStartTime
                << "," << sampleIndex
                << "," << sampleTime
                << "," << snapshot.zReferences[sampleIndex]
                << "," << snapshot.zVelocityReferences[sampleIndex];
            for (size_t stateIndex = 0; stateIndex < 24; ++stateIndex)
            {
                const scalar_t value = stateIndex < static_cast<size_t>(optimizedState.size())
                                           ? optimizedState(static_cast<Eigen::Index>(stateIndex))
                                           : std::numeric_limits<scalar_t>::quiet_NaN();
                foothold_plan_log_ << "," << value;
            }
            foothold_plan_log_ << "," << optimizedMode;
            for (size_t leg = 0; leg < 4; ++leg)
                foothold_plan_log_ << "," << (optimizedState.size() == 28 ? optimizedState(24 + leg) : window.d);
            foothold_plan_log_ << "," << window.v_max << "," << window.d << "\n";
        }
        // This diagnostic is only enabled for focused foothold experiments.
        // Flush once per MPC update so a forced controller shutdown does not
        // discard the pre-contact snapshot still buffered in userspace.
        foothold_plan_log_.flush();
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }
}

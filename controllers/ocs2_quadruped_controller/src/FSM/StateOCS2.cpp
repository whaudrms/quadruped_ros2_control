//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <algorithm>
#include <limits>
#include <mutex>

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
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
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Stage 7/8: the MPC PrimalSolution dump used to live here, but it
        // has moved into the MPC thread itself (CtrlComponent::setupMrt) so
        // that the dumped seq number is aligned 1:1 with each advanceMpc()
        // call and the synchronous robust_refine reader can match files.

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);

        // ---------------------------------------------------------------
        // Stage 8 — REFINED POLICY OVERRIDE (load-bearing).
        // After raw MPC evaluation, if the synchronous robust_refine pipeline
        // has produced a refined plan for the latest MPC cycle, interpolate
        // the refined trajectories at t_now and OVERRIDE optimized_state_ /
        // optimized_input_. WBC then tracks the refined values instead of
        // raw MPC. If no refined plan is available (timeout, disabled, or
        // out-of-horizon), the raw MPC outputs flow through unchanged.
        // ---------------------------------------------------------------
        bool refined_active = false;
        if (ctrl_component_->refined_seq_.load(std::memory_order_acquire) !=
            std::numeric_limits<size_t>::max())
        {
            std::lock_guard<std::mutex> lk(ctrl_component_->refined_mtx_);
            const auto& tTraj = ctrl_component_->refined_time_traj_;
            const auto& xTraj = ctrl_component_->refined_state_traj_;
            const auto& uTraj = ctrl_component_->refined_input_traj_;

            if (tTraj.size() >= 2 && !xTraj.empty())
            {
                const scalar_t t_now = ctrl_component_->observation_.time;
                const scalar_t t0 = tTraj.front();
                const scalar_t tEnd = tTraj.back();

                if (t_now >= t0 && t_now <= tEnd)
                {
                    // Linear interpolation on the refined time grid.
                    auto upper = std::upper_bound(tTraj.begin(), tTraj.end(), t_now);
                    size_t idx_hi = static_cast<size_t>(upper - tTraj.begin());
                    if (idx_hi == 0) idx_hi = 1;
                    if (idx_hi >= tTraj.size()) idx_hi = tTraj.size() - 1;
                    const size_t idx_lo = idx_hi - 1;

                    const scalar_t denom = tTraj[idx_hi] - tTraj[idx_lo];
                    const scalar_t alpha = (denom > 0.0)
                                               ? std::clamp((t_now - tTraj[idx_lo]) / denom, 0.0, 1.0)
                                               : 0.0;

                    if (idx_lo < xTraj.size() && idx_hi < xTraj.size())
                    {
                        const vector_t& x_lo = xTraj[idx_lo];
                        const vector_t& x_hi = xTraj[idx_hi];
                        if (x_lo.size() == optimized_state_.size() &&
                            x_hi.size() == optimized_state_.size())
                        {
                            optimized_state_ = (1.0 - alpha) * x_lo + alpha * x_hi;
                            refined_active = true;
                        }
                    }

                    // Inputs may be one shorter than states (collocation last
                    // node has no input). Clamp idx_hi to input array.
                    if (!uTraj.empty())
                    {
                        size_t u_lo = idx_lo;
                        size_t u_hi = idx_hi;
                        if (u_hi >= uTraj.size()) u_hi = uTraj.size() - 1;
                        if (u_lo > u_hi) u_lo = u_hi;
                        const vector_t& u_lo_v = uTraj[u_lo];
                        const vector_t& u_hi_v = uTraj[u_hi];
                        if (u_lo_v.size() == optimized_input_.size() &&
                            u_hi_v.size() == optimized_input_.size())
                        {
                            optimized_input_ = (1.0 - alpha) * u_lo_v + alpha * u_hi_v;
                            refined_active = true;
                        }
                    }
                }
            }
        }

        // Low-frequency status log (~once per 0.5 s) — confirms whether the
        // controller tick is currently tracking the refined plan or raw MPC.
        {
            const double t_now = ctrl_component_->observation_.time;
            if (t_now - last_refined_log_time_ > 0.5)
            {
                last_refined_log_time_ = t_now;
                RCLCPP_INFO(node_->get_logger(),
                            "[robust_refine] override=%s (t=%.3f)",
                            refined_active ? "active" : "inactive", t_now);
            }
        }

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        // Stage 8 — per-tick CSV log (only when tick_log_ is open).
        // Layout: t,refined_active,opt_state[0..23],opt_input[0..23],meas_rbd[0..23]
        // measured_rbd_state has shape: [theta_zyx(3), r_b(3), q_j(12), w_b(3), r_b_dot(3), q_j_dot(12)] = 36
        if (tick_log_.is_open())
        {
            if (!tick_log_header_written_)
            {
                tick_log_ << "t,refined_active";
                for (int i = 0; i < 24; ++i) tick_log_ << ",opt_x" << i;
                for (int i = 0; i < 24; ++i) tick_log_ << ",opt_u" << i;
                for (int i = 0;
                     i < static_cast<int>(ctrl_component_->measured_rbd_state_.size()); ++i)
                    tick_log_ << ",meas_rbd" << i;
                tick_log_ << ",planned_mode\n";
                tick_log_header_written_ = true;
            }
            tick_log_ << ctrl_component_->observation_.time
                      << "," << (refined_active ? 1 : 0);
            for (int i = 0; i < optimized_state_.size(); ++i)
                tick_log_ << "," << optimized_state_(i);
            for (int i = 0; i < optimized_input_.size(); ++i)
                tick_log_ << "," << optimized_input_(i);
            for (int i = 0; i < ctrl_component_->measured_rbd_state_.size(); ++i)
                tick_log_ << "," << ctrl_component_->measured_rbd_state_(i);
            tick_log_ << "," << planned_mode << "\n";
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
        ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                             ctrl_component_->mpc_mrt_interface_->getCommand());
    }

    void StateOCS2::exit()
    {
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

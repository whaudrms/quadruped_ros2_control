//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>
#include <filesystem>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        node_->declare_parameter("default_kp", default_kp_);
        node_->declare_parameter("default_kd", default_kd_);
        node_->declare_parameter("dataset_log_csv_path", std::string(""));
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();
        dataset_log_csv_path_ = node_->get_parameter("dataset_log_csv_path").as_string();

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
        openDatasetLogIfNeeded();
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

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);
        appendDatasetRow(planned_mode);

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        for (int i = 0; i < 12; i++)
        {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                             ctrl_component_->mpc_mrt_interface_->getCommand());
    }

    void StateOCS2::exit()
    {
        if (dataset_log_stream_.is_open())
        {
            dataset_log_stream_.flush();
            dataset_log_stream_.close();
        }
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

    void StateOCS2::openDatasetLogIfNeeded()
    {
        if (dataset_log_csv_path_.empty() || dataset_log_stream_.is_open())
        {
            return;
        }
        std::filesystem::path csvPath(dataset_log_csv_path_);
        if (csvPath.has_parent_path())
        {
            std::filesystem::create_directories(csvPath.parent_path());
        }
        dataset_log_stream_.open(dataset_log_csv_path_, std::ios::out | std::ios::trunc);
        dataset_log_header_written_ = false;
    }

    void StateOCS2::writeDatasetHeaderIfNeeded(size_t stateDim, size_t inputDim)
    {
        if (!dataset_log_stream_.is_open() || dataset_log_header_written_)
        {
            return;
        }

        dataset_log_stream_ << "time,obs_mode,planned_mode"
                            << ",obs_contact_fl,obs_contact_fr,obs_contact_rl,obs_contact_rr"
                            << ",plan_contact_fl,plan_contact_fr,plan_contact_rl,plan_contact_rr";

        for (size_t i = 0; i < stateDim; ++i)
        {
            dataset_log_stream_ << ",obs_state_" << i;
        }
        for (size_t i = 0; i < inputDim; ++i)
        {
            dataset_log_stream_ << ",obs_input_" << i;
        }
        for (size_t i = 0; i < stateDim; ++i)
        {
            dataset_log_stream_ << ",opt_state_" << i;
        }
        for (size_t i = 0; i < inputDim; ++i)
        {
            dataset_log_stream_ << ",opt_input_" << i;
        }
        dataset_log_stream_ << '\n';
        dataset_log_header_written_ = true;
    }

    void StateOCS2::appendDatasetRow(size_t plannedMode)
    {
        if (!dataset_log_stream_.is_open())
        {
            return;
        }

        const auto obsContacts = modeNumber2StanceLeg(ctrl_component_->observation_.mode);
        const auto plannedContacts = modeNumber2StanceLeg(plannedMode);
        const auto stateDim = static_cast<size_t>(ctrl_component_->observation_.state.size());
        const auto inputDim = static_cast<size_t>(ctrl_component_->observation_.input.size());
        writeDatasetHeaderIfNeeded(stateDim, inputDim);

        dataset_log_stream_ << ctrl_component_->observation_.time
                            << ',' << ctrl_component_->observation_.mode
                            << ',' << plannedMode
                            << ',' << static_cast<int>(obsContacts[0])
                            << ',' << static_cast<int>(obsContacts[1])
                            << ',' << static_cast<int>(obsContacts[2])
                            << ',' << static_cast<int>(obsContacts[3])
                            << ',' << static_cast<int>(plannedContacts[0])
                            << ',' << static_cast<int>(plannedContacts[1])
                            << ',' << static_cast<int>(plannedContacts[2])
                            << ',' << static_cast<int>(plannedContacts[3]);

        for (size_t i = 0; i < stateDim; ++i)
        {
            dataset_log_stream_ << ',' << ctrl_component_->observation_.state(static_cast<Eigen::Index>(i));
        }
        for (size_t i = 0; i < inputDim; ++i)
        {
            dataset_log_stream_ << ',' << ctrl_component_->observation_.input(static_cast<Eigen::Index>(i));
        }
        for (size_t i = 0; i < stateDim; ++i)
        {
            dataset_log_stream_ << ',' << optimized_state_(static_cast<Eigen::Index>(i));
        }
        for (size_t i = 0; i < inputDim; ++i)
        {
            dataset_log_stream_ << ',' << optimized_input_(static_cast<Eigen::Index>(i));
        }
        dataset_log_stream_ << '\n';
    }
}

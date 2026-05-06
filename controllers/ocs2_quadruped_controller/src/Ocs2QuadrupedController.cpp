//
// Created by tlab-uav on 24-9-24.
//

#include "Ocs2QuadrupedController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <angles/angles.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <controller_common/FSM/BaseFixedStand.h>

namespace ocs2::legged_robot
{
    namespace {
        // Fixed-pose stand intermediate between PASSIVE and OCS2.
        // Enter via command=2 from PASSIVE, then command=4 transitions to OCS2.
        // Ported from colleague's quadruped_temp/run_v1.
        class StateStandToOcs2 final : public BaseFixedStand {
        public:
            StateStandToOcs2(CtrlInterfaces& ctrl_interfaces,
                             const std::vector<double>& target_pos,
                             double kp,
                             double kd)
                : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd) {}

            FSMStateName checkChange() override {
                if (percent_ < 1.5) {
                    return FSMStateName::FIXEDSTAND;
                }
                switch (ctrl_interfaces_.control_inputs_.command) {
                    case 1:
                        return FSMStateName::PASSIVE;
                    case 4:
                        return FSMStateName::OCS2;
                    default:
                        return FSMStateName::FIXEDSTAND;
                }
            }
        };
    }  // namespace

    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        if (estimator_type_ == "ground_truth")
        {
            for (const auto& interface_type : odom_interface_types_)
            {
                conf.names.push_back(odom_name_ + "/" += interface_type);
            }
        }
        for (const auto& interface_type : foot_force_interface_types_)
        {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type Ocs2QuadrupedController::update(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period)
    {
        ctrl_comp_->updateState(time, period);

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init()
    {
        get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

        // Hardware Parameters
        command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        // IMU Sensor
        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
        imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);

        // Odometer Sensor (Ground Truth)
        estimator_type_ = auto_declare<std::string>("estimator_type", estimator_type_);
        if (estimator_type_ == "ground_truth")
        {
            odom_name_ = auto_declare<std::string>("odom_name", odom_name_);
            odom_interface_types_ = auto_declare<std::vector<std::string>>("odom_interfaces", state_interface_types_);
        }
        // Foot Force Sensor
        foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
        foot_force_interface_types_ =
            auto_declare<std::vector<std::string>>("foot_force_interfaces", state_interface_types_);

        // Stand pose & gains for StateStandToOcs2
        stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
        stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
        stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

        ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);
        ctrl_comp_->setupStateEstimate(estimator_type_);

        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateStandToOcs2>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.ocs2 = std::make_shared<StateOCS2>(ctrl_interfaces_, ctrl_comp_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == foot_force_name_)
            {
                ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == odom_name_)
            {
                ctrl_interfaces_.odom_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> Ocs2QuadrupedController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::OCS2:
            return state_list_.ocs2;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::legged_robot::Ocs2QuadrupedController, controller_interface::ControllerInterface);

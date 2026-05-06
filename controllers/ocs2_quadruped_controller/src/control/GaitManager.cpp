//
// Created by tlab-uav on 24-9-26.
//

#include <utility>
#include <rclcpp/rclcpp.hpp>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    GaitManager::GaitManager(CtrlInterfaces& ctrl_interfaces,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr)
        : ctrl_interfaces_(ctrl_interfaces),
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),
          target_gait_({0.0, 1.0}, {STANCE})
    {
    }

    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t& /**currentState**/,
                                   const ReferenceManagerInterface& /**referenceManager**/)
    {
        getTargetGait();
        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime,
                                                           timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::init(const std::string& gait_file)
    {
        gait_name_list_.clear();
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        gait_list_.clear();
        for (const auto& name : gait_name_list_)
        {
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    void GaitManager::getTargetGait()
    {
        if (ctrl_interfaces_.control_inputs_.command == 0) return;
        if (ctrl_interfaces_.control_inputs_.command == last_command_) return;
        last_command_ = ctrl_interfaces_.control_inputs_.command;
        const int command = std::max(0, ctrl_interfaces_.control_inputs_.command - 2);
        target_gait_ = gait_list_[command];
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s",
                    gait_name_list_[command].c_str());
        gait_updated_ = true;
    }

    void GaitManager::primeForOneShot(const scalar_t initTime, const scalar_t finalTime)
    {
        // Force the latest commanded gait template into target_gait_, even if
        // the command hasn't changed since last_command_. This bypasses the
        // early-return guards in getTargetGait() that exist to avoid
        // re-inserting on every preSolverRun.
        if (ctrl_interfaces_.control_inputs_.command > 0)
        {
            const int command = std::max(0, ctrl_interfaces_.control_inputs_.command - 2);
            if (command < static_cast<int>(gait_list_.size()))
            {
                target_gait_ = gait_list_[command];
                last_command_ = ctrl_interfaces_.control_inputs_.command;
                RCLCPP_INFO(rclcpp::get_logger("GaitManager"),
                            "[primeForOneShot] tiling '%s' over [%.3f, %.3f]",
                            gait_name_list_[command].c_str(), initTime, finalTime);
            }
        }

        // Tile the template directly across [initTime, finalTime] so the
        // upcoming solve sees a real walking gait inside its horizon.
        // (Normal preSolverRun would tile starting at finalTime instead.)
        gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, initTime, finalTime);
        // Mark as not-needing-update so preSolverRun on the first advanceMpc
        // does not re-tile starting at finalTime and clobber what we just set.
        gait_updated_ = false;
    }
}

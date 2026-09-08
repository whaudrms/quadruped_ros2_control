//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <fstream>
#include <cstdint>
#include <string>

#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>

#include "controller_common/FSM/FSMState.h"

namespace ocs2
{
    class MPC_MRT_Interface;
    class MPC_BASE;
}

namespace ocs2::legged_robot
{
    class StateOCS2 final : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<CtrlComponent>& ctrl_component
        );

        void enter() override;

        void run(const rclcpp::Time& time,
                 const rclcpp::Duration& period) override;

        void exit() override;

        FSMStateName checkChange() override;

    private:

        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        benchmark::RepeatedTimer wbc_timer_;

        double default_kp_ = 0;
        double default_kd_ = 6;

        vector_t optimized_state_, optimized_input_;

        // Per-tick CSV log of (t, optimized_state[24], optimized_input[24],
        // measured_rbd_state[36], planned_mode). Enabled by launch parameter
        // `tick_log_path`; if empty, no log is written. Used by post-trial
        // analysis to compute tracking error and overlay robust_phase markers.
        std::ofstream tick_log_;
        std::string tick_log_path_;
        bool tick_log_header_written_ = false;

        // Long-form MPC foothold-plan snapshots. Each reference-manager
        // sequence is appended once, so later contact-aware replans cannot
        // overwrite the plan that existed immediately before contact.
        std::ofstream foothold_plan_log_;
        std::string foothold_plan_log_path_;
        uint64_t last_foothold_plan_sequence_ = 0;
        bool foothold_plan_log_header_written_ = false;

        void logLatestFootholdPlanSnapshot();
    };
}


#endif //STATEOCS2_H

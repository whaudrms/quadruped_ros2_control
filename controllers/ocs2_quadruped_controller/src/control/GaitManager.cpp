//
// Created by tlab-uav on 24-9-26.
//

#include <algorithm>
#include <utility>
#include <rclcpp/rclcpp.hpp>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

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
        // Splice runs FIRST so the new template insertion below sees the
        // already-spliced schedule (avoids one cycle of stale schedule that
        // would happen if we spliced after gait insertion).
        applyPendingSplices(initTime, finalTime);

        getTargetGait();
        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime,
                                                           timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::requestStanceSplice(const size_t leg, const scalar_t event_time)
    {
        if (leg >= splice_pending_.size()) return;
        std::lock_guard<std::mutex> lk(splice_mutex_);
        // If a request for this leg is already pending and not yet drained,
        // keep the EARLIER event_time — that's the moment the leg actually
        // first touched, which is what the splice should anchor on.
        if (splice_pending_[leg] && event_time >= splice_time_[leg]) return;
        splice_pending_[leg] = true;
        splice_time_[leg] = event_time;
    }

    void GaitManager::applyPendingSplices(const scalar_t initTime, const scalar_t finalTime)
    {
        // Drain pending requests under lock, then operate without lock.
        feet_array_t<bool> pending{};
        feet_array_t<scalar_t> time{};
        {
            std::lock_guard<std::mutex> lk(splice_mutex_);
            for (size_t i = 0; i < splice_pending_.size(); ++i)
            {
                pending[i] = splice_pending_[i];
                time[i]    = splice_time_[i];
                splice_pending_[i] = false;
            }
        }

        // Collect (leg, t) pairs in chronological order so subsequent inserts
        // see the prior inserts. (Ordering matters because findIndexInTimeArray
        // depends on the current contents of eventTimes.)
        std::vector<std::pair<scalar_t, size_t>> requests;
        for (size_t leg = 0; leg < pending.size(); ++leg)
        {
            if (pending[leg]) requests.emplace_back(time[leg], leg);
        }
        if (requests.empty()) return;
        std::sort(requests.begin(), requests.end());

        // Pull a wide enough slice to cover any splice request; getModeSchedule
        // is allowed to mutate internal state because we are on the MPC thread,
        // serialized with all other GaitSchedule reads/writes.
        const scalar_t loBound = std::min(initTime, requests.front().first) - 0.5;
        const scalar_t hiBound = finalTime + 0.5;
        ModeSchedule sched = gait_schedule_ptr_->getModeSchedule(loBound, hiBound);

        for (const auto& [t, leg] : requests)
        {
            const int curPhaseInt = lookup::findIndexInTimeArray(sched.eventTimes, t);
            const size_t curPhase = static_cast<size_t>(
                std::clamp<int>(curPhaseInt, 0,
                                static_cast<int>(sched.modeSequence.size()) - 1));

            contact_flag_t curFlags = modeNumber2StanceLeg(sched.modeSequence[curPhase]);
            if (curFlags[leg]) continue; // already stance — splice is a no-op

            curFlags[leg] = true;
            const size_t newMode = stanceLeg2ModeNumber(curFlags);

            // Insert event at t; the new mode covers [t, original next event).
            // ModeSchedule invariant: |eventTimes| = |modeSequence| - 1, with
            // modeSequence[i] applying to [eventTimes[i-1], eventTimes[i]).
            sched.eventTimes.insert(sched.eventTimes.begin() + curPhase, t);
            sched.modeSequence.insert(sched.modeSequence.begin() + curPhase + 1, newMode);

            // Forward propagation: the original sequence after the inserted
            // phase may still have this leg in swing for one or more phases
            // before the gait template's natural touchdown. Walk forward and
            // force stance until the original schedule itself takes the leg
            // back to stance (which is the natural next touchdown). This
            // makes the splice "early stance transition until natural
            // touchdown" instead of "single-phase patch" — the latter is only
            // correct for gaits where the very next phase already has the leg
            // in stance (e.g. standing trot's all-stance inter-step).
            size_t propagatedTo = curPhase + 1;
            for (size_t i = curPhase + 2; i < sched.modeSequence.size(); ++i)
            {
                contact_flag_t f = modeNumber2StanceLeg(sched.modeSequence[i]);
                if (f[leg]) break; // original schedule already stance — natural touchdown reached
                f[leg] = true;
                sched.modeSequence[i] = stanceLeg2ModeNumber(f);
                propagatedTo = i;
            }

            // sched.modeSequence[curPhase] is unchanged by the insert — it
            // still covers [..., t) — so it's the "old mode" the leg was in
            // when the splice fired.
            // propagated_phases = (propagatedTo - curPhase): 1 means only the
            // newly inserted phase needed flipping (gait's natural next phase
            // already had this leg in stance, e.g. standing trot's all-stance
            // inter-step); >1 means the propagation walk also flipped one or
            // more subsequent swing phases — useful diagnostic for non-trot
            // gaits or perceptive schedules where the natural touchdown is
            // several phases away.
            RCLCPP_INFO(rclcpp::get_logger("gait_manager"),
                        "[robust_splice] leg=%zu t=%.3f mode_old=%zu mode_new=%zu "
                        "propagated_phases=%zu (early stance until natural touchdown)",
                        leg, t, sched.modeSequence[curPhase], newMode,
                        propagatedTo - curPhase);
        }

        gait_schedule_ptr_->setModeSchedule(sched);
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
}

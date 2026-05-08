//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H
#include <mutex>

#include <controller_common/CtrlInterfaces.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::legged_robot
{
    class GaitManager final : public SolverSynchronizedModule
    {
    public:
        GaitManager(CtrlInterfaces& ctrl_interfaces,
                    std::shared_ptr<GaitSchedule> gait_schedule_ptr);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution&/**primalSolution**/) override
        {
        }

        void init(const std::string& gait_file);

        // Track ② step (b): callable from any thread (controller-thread event
        // detector in CtrlComponent::detectAndLogContactEvents). Queues a
        // request to splice `leg` to stance starting at `event_time`. The
        // actual ModeSchedule mutation is deferred to the next preSolverRun
        // (MPC thread) so it is properly serialized with the other
        // GaitSchedule reads/writes that already happen there
        // (insertModeSequenceTemplate, getModeSchedule via the reference
        // manager). This avoids the data race that direct mutation from the
        // controller thread would have introduced.
        void requestStanceSplice(size_t leg, scalar_t event_time);

    private:
        void getTargetGait();

        // Drains pending splice requests and applies them to gait_schedule_ptr_.
        // Called at the start of preSolverRun (MPC thread, synchronized).
        // For each pending leg, inserts a stance-flipped mode at event_time
        // AND propagates the stance bit forward through subsequent phases
        // until the original schedule's first stance phase for that leg
        // (gait-agnostic "early stance transition until natural touchdown").
        void applyPendingSplices(scalar_t initTime, scalar_t finalTime);

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;

        ModeSequenceTemplate target_gait_;
        int last_command_ = 0;
        bool gait_updated_{false};
        bool verbose_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;

        std::mutex splice_mutex_;
        feet_array_t<bool> splice_pending_{};
        feet_array_t<scalar_t> splice_time_{};
    };
}


#endif //GAITMANAGER_H

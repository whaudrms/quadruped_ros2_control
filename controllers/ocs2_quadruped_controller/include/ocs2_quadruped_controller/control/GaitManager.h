//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
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

        // Stage 9 — for one-shot OCP mode. The normal preSolverRun() inserts a
        // newly-selected gait template starting AT finalTime (i.e., AFTER the
        // current solve's horizon). MPC mode tolerates this because the next
        // tick's solve picks up the new gait at its initTime; one-shot mode
        // never re-solves, so the horizon stays stuck on the OLD gait
        // (typically all-STANCE).
        // This helper forces the latest commanded gait to tile across
        // [initTime, finalTime] BEFORE the bootstrap solve runs, so the OCP
        // optimizes over a real walking gait. Caller must invoke this just
        // before advanceMpc() in init().
        void primeForOneShot(scalar_t initTime, scalar_t finalTime);

    private:
        void getTargetGait();

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;

        ModeSequenceTemplate target_gait_;
        int last_command_ = 0;
        bool gait_updated_{false};
        bool verbose_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;
    };
}


#endif //GAITMANAGER_H

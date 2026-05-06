//
// MpcDumpRecorder.h — Stage 7 (contact_timing_uncertainty/robust_refine integration)
//
// Dumps OCS2 PrimalSolution + SystemObservation to per-cycle CSV files for
// offline consumption by the Python robust_refine layer.
//
// Stage 8 extension: when constructed with maxCycles == 0, the recorder runs
// in "infinite" mode (no cycle cap). This is used by the synchronous
// MPC + robust_refine IPC path in CtrlComponent — every MPC solve writes
// /dev/shm/robust_refine/in/cycle_<NNNNN>.csv and the Python daemon writes
// the corresponding /dev/shm/robust_refine/out/cycle_<NNNNN>.csv.
//

#ifndef OCS2_QUADRUPED_CONTROLLER_MPC_DUMP_RECORDER_H
#define OCS2_QUADRUPED_CONTROLLER_MPC_DUMP_RECORDER_H

#include <cstddef>
#include <limits>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

namespace ocs2::legged_robot
{
    class MpcDumpRecorder
    {
    public:
        // maxCycles == 0 means "no cap" (infinite mode for sync IPC).
        MpcDumpRecorder(std::string dumpDir, size_t maxCycles, scalar_t minIntervalSec);

        bool isEnabled() const { return enabled_; }

        // True when a finite cap was set and we've reached it.
        bool isFinished() const
        {
            return enabled_ && maxCycles_ != 0 && seq_ >= maxCycles_;
        }

        size_t cyclesWritten() const { return seq_; }

        // Returns the cycle index of the most recently written file, or
        // std::numeric_limits<size_t>::max() if nothing has been written yet.
        // Used by RefinedPolicyReader to compute the matching output path.
        size_t lastWrittenSeq() const
        {
            return (seq_ == 0) ? std::numeric_limits<size_t>::max() : (seq_ - 1);
        }

        const std::string& dumpDir() const { return dumpDir_; }

        // Writes one CSV file per call when (a) at least minIntervalSec has elapsed
        // since the last write (measured by solution.timeTrajectory_.front()) and
        // (b) cycle counter has not exceeded maxCycles (when maxCycles != 0).
        // Returns true when a file was actually written; false when skipped.
        bool record(const SystemObservation& observation, const PrimalSolution& solution);

    private:
        std::string dumpDir_;
        size_t maxCycles_;
        scalar_t minIntervalSec_;
        size_t seq_ = 0;
        scalar_t lastWrittenInitTime_ = -std::numeric_limits<scalar_t>::infinity();
        bool enabled_ = false;
    };
} // namespace ocs2::legged_robot

#endif // OCS2_QUADRUPED_CONTROLLER_MPC_DUMP_RECORDER_H

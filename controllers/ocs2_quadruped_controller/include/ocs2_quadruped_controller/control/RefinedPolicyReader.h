//
// RefinedPolicyReader.h — Stage 8 (synchronous MPC + robust_refine IPC)
//
// Polls /dev/shm/robust_refine/out/cycle_<NNNNN>.csv for output from the
// Python robust_refine daemon. The output file uses the SAME CSV format as
// the input dumped by MpcDumpRecorder: a single-line JSON metadata header
// (`# { ... }`) followed by a column header (`time,x0..xN,u0..uM`) and one
// body row per time-point.
//
// The reader extracts only `time`, `x_traj`, `u_traj` — the Python daemon
// is responsible for echoing back any mode/event metadata if needed; we do
// not parse the JSON contents on the C++ side beyond verifying the leading
// `# {` sentinel.
//

#ifndef OCS2_QUADRUPED_CONTROLLER_REFINED_POLICY_READER_H
#define OCS2_QUADRUPED_CONTROLLER_REFINED_POLICY_READER_H

#include <cstddef>
#include <string>

#include <ocs2_core/Types.h>

namespace ocs2::legged_robot
{
    class RefinedPolicyReader
    {
    public:
        RefinedPolicyReader(std::string outputDir, scalar_t timeoutSec, bool blocking);

        // Wait for /dev/shm/robust_refine/out/cycle_<seq>.csv to appear, then parse.
        // Returns true on success, false on timeout or unrecoverable parse error.
        // On success, timeTraj/stateTraj/inputTraj are filled with the refined plan.
        bool waitAndLoad(size_t seq,
                         scalar_array_t& timeTraj,
                         vector_array_t& stateTraj,
                         vector_array_t& inputTraj);

        // Async-with-latest path (Stage 8 Option B): scan outputDir_ once,
        // find the cycle_<NNNNN>.csv file with the highest NNNNN, parse it,
        // and return its seq number via outSeq. Does NOT block; returns
        // immediately if no parsable file is found. Returns true on success.
        // Caller is responsible for deciding whether the loaded plan is still
        // fresh enough (compare its time trajectory against current obs time).
        bool loadLatest(size_t& outSeq,
                        scalar_array_t& timeTraj,
                        vector_array_t& stateTraj,
                        vector_array_t& inputTraj);

        const std::string& outputDir() const { return outputDir_; }
        scalar_t timeoutSec() const { return timeoutSec_; }
        bool blocking() const { return blocking_; }

    private:
        // Parse one CSV file. Returns false if file is missing, unreadable, or
        // appears to be partially written (caller should keep polling).
        bool tryParse(const std::string& path,
                      scalar_array_t& timeTraj,
                      vector_array_t& stateTraj,
                      vector_array_t& inputTraj) const;

        std::string outputDir_;
        scalar_t timeoutSec_;
        bool blocking_;
    };
} // namespace ocs2::legged_robot

#endif // OCS2_QUADRUPED_CONTROLLER_REFINED_POLICY_READER_H

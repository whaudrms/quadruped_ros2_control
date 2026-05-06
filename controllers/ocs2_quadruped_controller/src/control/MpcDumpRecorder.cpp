#include "ocs2_quadruped_controller/control/MpcDumpRecorder.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        void writeJsonScalarArray(std::ostream& os, const scalar_array_t& arr)
        {
            os << "[";
            for (size_t i = 0; i < arr.size(); ++i)
            {
                if (i > 0) os << ",";
                os << arr[i];
            }
            os << "]";
        }

        void writeJsonSizeArray(std::ostream& os, const size_array_t& arr)
        {
            os << "[";
            for (size_t i = 0; i < arr.size(); ++i)
            {
                if (i > 0) os << ",";
                os << arr[i];
            }
            os << "]";
        }

        void writeJsonEigen(std::ostream& os, const vector_t& v)
        {
            os << "[";
            for (Eigen::Index i = 0; i < v.size(); ++i)
            {
                if (i > 0) os << ",";
                os << v(i);
            }
            os << "]";
        }
    } // namespace

    MpcDumpRecorder::MpcDumpRecorder(std::string dumpDir, size_t maxCycles, scalar_t minIntervalSec)
        : dumpDir_(std::move(dumpDir)),
          maxCycles_(maxCycles),
          minIntervalSec_(minIntervalSec)
    {
        // dumpDir_ must be non-empty. maxCycles_ == 0 is now legal — infinite mode.
        if (dumpDir_.empty()) return;
        std::error_code ec;
        std::filesystem::create_directories(dumpDir_, ec);
        if (ec)
        {
            return;
        }
        enabled_ = true;
    }

    bool MpcDumpRecorder::record(const SystemObservation& observation, const PrimalSolution& solution)
    {
        if (!enabled_) return false;
        if (maxCycles_ != 0 && seq_ >= maxCycles_) return false;
        if (solution.timeTrajectory_.empty() || solution.stateTrajectory_.empty()) return false;

        const scalar_t initTime = solution.timeTrajectory_.front();
        if (initTime - lastWrittenInitTime_ < minIntervalSec_) return false;
        lastWrittenInitTime_ = initTime;

        std::ostringstream namestream;
        namestream << dumpDir_ << "/cycle_"
                   << std::setw(5) << std::setfill('0') << seq_ << ".csv";
        const std::string finalPath = namestream.str();
        const std::string tmpPath = finalPath + ".tmp";

        std::ofstream f(tmpPath);
        if (!f.is_open()) return false;
        f << std::setprecision(17);

        const Eigen::Index stateDim = solution.stateTrajectory_.front().size();
        const Eigen::Index inputDim = solution.inputTrajectory_.empty()
                                          ? 0
                                          : solution.inputTrajectory_.front().size();
        const size_t numStatePts = solution.stateTrajectory_.size();
        const size_t numInputPts = solution.inputTrajectory_.size();
        const size_t numTimePts = solution.timeTrajectory_.size();

        // Single-line JSON metadata header (consumed by Python loader).
        f << "# {";
        f << "\"obs_time\":" << observation.time;
        f << ",\"obs_mode\":" << observation.mode;
        f << ",\"obs_state\":";
        writeJsonEigen(f, observation.state);
        f << ",\"obs_input\":";
        writeJsonEigen(f, observation.input);
        f << ",\"event_times\":";
        writeJsonScalarArray(f, solution.modeSchedule_.eventTimes);
        f << ",\"mode_seq\":";
        writeJsonSizeArray(f, solution.modeSchedule_.modeSequence);
        f << ",\"post_event_indices\":";
        writeJsonSizeArray(f, solution.postEventIndices_);
        f << ",\"state_dim\":" << stateDim;
        f << ",\"input_dim\":" << inputDim;
        f << ",\"num_time_pts\":" << numTimePts;
        f << ",\"num_state_pts\":" << numStatePts;
        f << ",\"num_input_pts\":" << numInputPts;
        f << "}\n";

        // Column header
        f << "time";
        for (Eigen::Index i = 0; i < stateDim; ++i) f << ",x" << i;
        for (Eigen::Index i = 0; i < inputDim; ++i) f << ",u" << i;
        f << "\n";

        // Body — one row per time point.  If a row has no input (last node in
        // SQP convention), pad with zeros and let the Python loader trim.
        for (size_t k = 0; k < numTimePts; ++k)
        {
            f << solution.timeTrajectory_[k];
            if (k < numStatePts)
            {
                const auto& x = solution.stateTrajectory_[k];
                for (Eigen::Index i = 0; i < stateDim; ++i) f << "," << x(i);
            }
            else
            {
                for (Eigen::Index i = 0; i < stateDim; ++i) f << ",0";
            }
            if (k < numInputPts)
            {
                const auto& u = solution.inputTrajectory_[k];
                for (Eigen::Index i = 0; i < inputDim; ++i) f << "," << u(i);
            }
            else
            {
                for (Eigen::Index i = 0; i < inputDim; ++i) f << ",0";
            }
            f << "\n";
        }

        f.close();

        // Atomic rename so a reader never sees a half-written file.
        std::error_code ec;
        std::filesystem::rename(tmpPath, finalPath, ec);
        if (ec)
        {
            // Best-effort cleanup; treat as failed write so seq_ is not advanced.
            std::filesystem::remove(tmpPath, ec);
            return false;
        }

        ++seq_;
        return true;
    }
} // namespace ocs2::legged_robot

#include "ocs2_quadruped_controller/control/RefinedPolicyReader.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace ocs2::legged_robot
{
    namespace
    {
        std::string seqToFilename(size_t seq)
        {
            std::ostringstream oss;
            oss << "cycle_" << std::setw(5) << std::setfill('0') << seq << ".csv";
            return oss.str();
        }

        // Split CSV row by ',' — no quoted-field handling because the producer
        // (MpcDumpRecorder + Python daemon) only emits plain numeric values.
        bool splitCsv(const std::string& line, std::vector<std::string>& fields)
        {
            fields.clear();
            std::string cur;
            for (const char c : line)
            {
                if (c == ',')
                {
                    fields.push_back(std::move(cur));
                    cur.clear();
                }
                else if (c == '\r')
                {
                    // tolerate CRLF
                }
                else
                {
                    cur.push_back(c);
                }
            }
            fields.push_back(std::move(cur));
            return !fields.empty();
        }
    } // namespace

    RefinedPolicyReader::RefinedPolicyReader(std::string outputDir,
                                             scalar_t timeoutSec,
                                             bool blocking)
        : outputDir_(std::move(outputDir)),
          timeoutSec_(timeoutSec),
          blocking_(blocking)
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir_, ec);
        // If the directory cannot be created we still keep the object alive;
        // waitAndLoad() will simply time out for every call.
    }

    bool RefinedPolicyReader::loadLatest(size_t& outSeq,
                                         scalar_array_t& timeTraj,
                                         vector_array_t& stateTraj,
                                         vector_array_t& inputTraj)
    {
        // Single non-blocking scan of outputDir_. Pick highest seq cycle_*.csv.
        std::error_code ec;
        std::filesystem::directory_iterator it(outputDir_, ec);
        if (ec) return false;

        constexpr size_t kBadSeq = std::numeric_limits<size_t>::max();
        size_t bestSeq = kBadSeq;
        std::string bestPath;
        const std::string prefix = "cycle_";
        const std::string suffix = ".csv";

        for (; it != std::filesystem::end(it); it.increment(ec))
        {
            if (ec) break;
            const auto& p = it->path();
            const std::string name = p.filename().string();
            if (name.size() < prefix.size() + 1 + suffix.size()) continue;
            if (name.compare(0, prefix.size(), prefix) != 0) continue;
            if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) continue;
            const std::string digits =
                name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
            size_t s = 0;
            try { s = std::stoull(digits); } catch (...) { continue; }
            if (bestSeq == kBadSeq || s > bestSeq)
            {
                bestSeq = s;
                bestPath = p.string();
            }
        }

        if (bestSeq == kBadSeq) return false;
        if (!tryParse(bestPath, timeTraj, stateTraj, inputTraj)) return false;
        outSeq = bestSeq;
        return true;
    }

    bool RefinedPolicyReader::waitAndLoad(size_t seq,
                                          scalar_array_t& timeTraj,
                                          vector_array_t& stateTraj,
                                          vector_array_t& inputTraj)
    {
        const std::string path = outputDir_ + "/" + seqToFilename(seq);

        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeoutSec_));

        for (;;)
        {
            std::error_code ec;
            if (std::filesystem::exists(path, ec))
            {
                if (tryParse(path, timeTraj, stateTraj, inputTraj))
                {
                    return true;
                }
                // Parse failed — file may still be mid-write. Keep polling.
            }

            if (!blocking_) return false;
            if (std::chrono::steady_clock::now() >= deadline) return false;

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    bool RefinedPolicyReader::tryParse(const std::string& path,
                                       scalar_array_t& timeTraj,
                                       vector_array_t& stateTraj,
                                       vector_array_t& inputTraj) const
    {
        std::ifstream f(path);
        if (!f.is_open()) return false;

        // 1) JSON metadata header — first non-empty line, must start with "# {".
        std::string headerLine;
        if (!std::getline(f, headerLine)) return false;
        if (headerLine.size() < 3 || headerLine[0] != '#' || headerLine.find('{') == std::string::npos)
        {
            return false;
        }

        // 2) Column header — `time,x0,..,xN,u0,..,uM`.
        std::string columnLine;
        if (!std::getline(f, columnLine)) return false;
        std::vector<std::string> columns;
        splitCsv(columnLine, columns);
        if (columns.size() < 2 || columns[0] != "time") return false;

        size_t stateDim = 0;
        size_t inputDim = 0;
        for (size_t i = 1; i < columns.size(); ++i)
        {
            const auto& name = columns[i];
            if (!name.empty() && name[0] == 'x') ++stateDim;
            else if (!name.empty() && name[0] == 'u') ++inputDim;
        }
        if (stateDim == 0) return false;

        // 3) Body — each row: time, x0..xN, u0..uM
        scalar_array_t tTmp;
        vector_array_t xTmp;
        vector_array_t uTmp;
        tTmp.reserve(64);
        xTmp.reserve(64);
        uTmp.reserve(64);

        std::vector<std::string> fields;
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty()) continue;
            splitCsv(line, fields);
            const size_t expected = 1u + stateDim + inputDim;
            if (fields.size() != expected) return false;

            try
            {
                tTmp.push_back(std::stod(fields[0]));

                vector_t xRow(static_cast<Eigen::Index>(stateDim));
                for (size_t i = 0; i < stateDim; ++i)
                {
                    xRow(static_cast<Eigen::Index>(i)) = std::stod(fields[1 + i]);
                }
                xTmp.push_back(std::move(xRow));

                if (inputDim > 0)
                {
                    vector_t uRow(static_cast<Eigen::Index>(inputDim));
                    for (size_t i = 0; i < inputDim; ++i)
                    {
                        uRow(static_cast<Eigen::Index>(i)) = std::stod(fields[1 + stateDim + i]);
                    }
                    uTmp.push_back(std::move(uRow));
                }
            }
            catch (const std::exception&)
            {
                return false;
            }
        }

        if (tTmp.empty()) return false;

        timeTraj = std::move(tTmp);
        stateTraj = std::move(xTmp);
        inputTraj = std::move(uTmp);
        return true;
    }
} // namespace ocs2::legged_robot

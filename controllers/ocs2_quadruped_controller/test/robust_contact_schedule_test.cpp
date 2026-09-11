#include <iostream>
#include <limits>
#include <stdexcept>

#include "ocs2_quadruped_controller/perceptive/interface/RobustContactSchedule.h"

using namespace ocs2;
using namespace ocs2::legged_robot;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void verify(const ModeSchedule& nominal, const std::vector<RobustContactOverride>& contacts) {
    const auto result = applyRobustContactOverrides(nominal, contacts);
    require(result.modeSequence.size() == result.eventTimes.size() + 1, "invalid schedule size");
    require(std::adjacent_find(result.eventTimes.begin(), result.eventTimes.end(),
                              [](scalar_t a, scalar_t b) { return a >= b; }) == result.eventTimes.end(),
            "duplicate/non-increasing events");
    for (const auto t : nominal.eventTimes) {
        require(std::binary_search(result.eventTimes.begin(), result.eventTimes.end(), t), "nominal event moved");
    }
    std::vector<scalar_t> probes{-0.1, 1.0};
    for (int k = 0; k <= 900; ++k) probes.push_back(k * 0.001);
    for (const auto t : result.eventTimes) {
        probes.push_back(t - 1e-9);
        probes.push_back(t);
        probes.push_back(t + 1e-9);
    }
    for (const auto t : probes) {
        auto expected = modeNumber2StanceLeg(nominal.modeAtTime(t));
        for (const auto& contact : contacts) {
            // Exact events use OCS2's pre-event mode convention.
            if (t > contact.eventTime && t <= contact.touchdownTime) expected[contact.leg] = true;
        }
        require(modeNumber2StanceLeg(result.modeAtTime(t)) == expected, "contact changed outside requested interval");
    }
    // Each replan starts from nominal; results must not drift or acquire events.
    const auto replanned = applyRobustContactOverrides(nominal, contacts);
    require(result.eventTimes == replanned.eventTimes && result.modeSequence == replanned.modeSequence,
            "replanning drifted");
}

int main() {
    // RF/LH swing [0,.25), stance [.25,.6), next swing [.6,.85).
    const ModeSchedule nominal({0.0, 0.25, 0.30, 0.55, 0.60, 0.85, 0.90},
                               {15, 9, 15, 6, 15, 9, 15, 6});
    verify(nominal, {});
    verify(nominal, {{1, 0.15, 0.05, 0.25}});
    verify(nominal, {{1, 0.249, 0.05, 0.25}});  // old near-touchdown merge backdated stance
    verify(nominal, {{1, 0.05, 0.05, 0.25}});   // robust start
    verify(nominal, {{1, 0.15, 0.05, 0.25}, {2, 0.152, 0.05, 0.25}});
    verify(nominal, {{2, 0.152, 0.05, 0.25}, {1, 0.15, 0.05, 0.25}});
    verify(nominal, {{1, 0.15, 0.05, 0.25}, {2, 0.15, 0.05, 0.25}});
    verify(nominal, {{1, 0.15, 0.05, 0.25}, {1, 0.16, 0.05, 0.25}});
    verify(nominal, {{1, 0.15, 0.05, 0.25}, {1, 0.72, 0.65, 0.85}});

    // Preserve unrelated transitions within the overridden swing, including
    // exact coincidences and a contact just after another leg's event.
    const ModeSchedule intermediate({0.0, 0.10, 0.25, 0.30}, {15, 9, 8, 15, 6});
    verify(intermediate, {{1, 0.08, 0.05, 0.25}});
    verify(intermediate, {{1, 0.10, 0.05, 0.25}});
    verify(intermediate, {{1, 0.101, 0.05, 0.25}});

    for (const auto& invalid : std::vector<RobustContactOverride>{
             {1, 0.04, 0.05, 0.25}, {1, 0.25, 0.05, 0.25}, {1, 0.26, 0.05, 0.25},
             {0, 0.15, 0.05, 0.25}, {4, 0.15, 0.05, 0.25},
             {1, 0.15, 0.05, 0.85},  // cannot propagate through intervening stance
             {1, std::numeric_limits<scalar_t>::quiet_NaN(), 0.05, 0.25}}) {
        require(!isValidRobustContactOverride(nominal, invalid), "invalid request accepted");
        const auto result = applyRobustContactOverrides(nominal, {invalid});
        require(result.eventTimes == nominal.eventTimes && result.modeSequence == nominal.modeSequence,
                "invalid request changed schedule");
    }
    const ModeSchedule changedGait({0.0, 0.20, 0.30}, {15, 9, 15, 6});
    require(!isValidRobustContactOverride(changedGait, {1, 0.15, 0.05, 0.25}), "stale touchdown accepted");

    // A second queue drain must preserve the first leg's precise event time.
    const auto first = applyRobustContactOverrides(nominal, {{1, 0.15, 0.05, 0.25}});
    const auto second = applyRobustContactOverrides(nominal, {{1, 0.15, 0.05, 0.25}, {2, 0.152, 0.05, 0.25}});
    require(first.modeAtTime(0.151) == second.modeAtTime(0.151), "later contact backdated another leg");
    std::cout << "Robust contact schedule regression checks passed\n";
}

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "ocs2_quadruped_controller/perceptive/interface/RobustPhaseTiming.h"

using namespace ocs2;
using namespace ocs2::legged_robot;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void near(scalar_t actual, scalar_t expected) {
    require(std::abs(actual - expected) < 1e-12, "unexpected timing or velocity");
}

template <typename Function>
void rejects(Function function) {
    bool rejected = false;
    try { function(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid setting was accepted");
}

int main() {
    const ModeSchedule nominal({0.0, 0.25, 0.30, 0.55, 0.60, 0.85, 0.90},
                               {15, 9, 15, 6, 15, 9, 15, 6});
    const auto timings = makeRobustTouchdownTimings(nominal, 0.05, 0.05);
    const auto delayed = delayRobustTouchdowns(nominal, timings);
    const auto rf = std::find_if(timings.begin(), timings.end(), [](const auto& t) { return t.leg == 1; });
    require(rf != timings.end(), "RF window missing");
    near(rf->nominalTouchdown, 0.25);
    near(rf->start, 0.20);
    near(rf->end, 0.30);
    near(robustPhaseVelocityLimit(0.05, rf->start, rf->end), 1.0);
    near(robustPhaseVelocityLimit(0.02, rf->start, rf->end), 0.4);

    require(!modeNumber2StanceLeg(delayed.modeAtTime(0.27))[1], "stance started before robust end");
    require(modeNumber2StanceLeg(delayed.modeAtTime(0.31))[1], "stance did not start after robust end");
    require(!modeNumber2StanceLeg(delayed.modeAtTime(0.61))[1], "next liftoff moved");
    // 0.55+0.05 must not add an artificial 1-ULP phase beside nominal 0.60.
    require(delayed.eventTimes == nominal.eventTimes, "arithmetic roundoff duplicated an existing event");

    // Every leg changes only between its nominal touchdown and robust end.
    for (int k = 0; k < 10000; ++k) {
        const scalar_t time = k * 0.0001 + 0.000037;
        auto expected = modeNumber2StanceLeg(nominal.modeAtTime(time));
        for (const auto& timing : timings) {
            if (time > timing.nominalTouchdown && time <= timing.end) expected[timing.leg] = false;
        }
        require(modeNumber2StanceLeg(delayed.modeAtTime(time)) == expected, "delay changed unrelated interval");
    }

    // Early and late contact both restore stance only through configured end.
    for (const scalar_t contact : {0.22, 0.27, 0.299}) {
        const RobustContactOverride request{1, contact, rf->start, rf->end};
        require(isValidRobustContactOverride(delayed, request), "contact within robust window rejected");
        const auto final = applyRobustContactOverrides(delayed, {request});
        for (int k = 0; k < 10000; ++k) {
            const scalar_t time = k * 0.0001 + 0.000037;
            auto expected = modeNumber2StanceLeg(delayed.modeAtTime(time));
            if (time > contact && time <= rf->end) expected[1] = true;
            require(modeNumber2StanceLeg(final.modeAtTime(time)) == expected, "splice escaped its robust interval");
        }
    }

    // Replan from original gait, including while past nominal touchdown.
    for (const scalar_t now : {0.0, 0.21, 0.26, 0.29}) {
        const auto next = makeRobustTouchdownTimings(nominal, 0.05, 0.05);
        const auto window = std::find_if(next.begin(), next.end(), [&](const auto& t) {
            return t.leg == 1 && t.end > now;
        });
        near(window->start, 0.20);
        near(window->end, 0.30);
        near(robustPhaseVelocityLimit(0.05, window->start, window->end), 1.0);
        require(delayRobustTouchdowns(nominal, next).eventTimes == delayed.eventTimes, "replan accumulated delay");
    }

    const auto preOnly = delayRobustTouchdowns(nominal, makeRobustTouchdownTimings(nominal, 0.10, 0.0));
    require(preOnly.eventTimes == nominal.eventTimes && preOnly.modeSequence == nominal.modeSequence,
            "zero delay changed the gait");
    const auto postOnly = makeRobustTouchdownTimings(nominal, 0.0, 0.05);
    near(postOnly.front().start, postOnly.front().nominalTouchdown);
    const auto capped = makeRobustTouchdownTimings(nominal, 0.50, 0.05);
    const auto cappedRf = std::find_if(capped.begin(), capped.end(), [](const auto& t) { return t.leg == 1; });
    near(cappedRf->start, 0.0);
    near(robustPhaseVelocityLimit(0.05, cappedRf->start, cappedRf->end), 1.0 / 3.0);

    rejects([&] { makeRobustTouchdownTimings(nominal, 0.05, 0.35); });
    rejects([] { validateRobustPhaseTiming(-0.01, 0.05, 0.05); });
    rejects([] { validateRobustPhaseTiming(0.05, -0.01, 0.05); });
    rejects([] { validateRobustPhaseTiming(0.0, 0.0, 0.05); });
    rejects([] { validateRobustPhaseTiming(0.05, 0.05, 0.0); });
    rejects([] { validateRobustPhaseTiming(std::numeric_limits<double>::infinity(), 0.05, 0.05); });
    rejects([] { robustPhaseVelocityLimit(0.05, 0.3, 0.3); });
    std::cout << "Robust timing, derived velocity and splice composition checks passed\n";
}

#pragma once

#include <limits>
#include <stdexcept>
#include "ocs2_quadruped_controller/perceptive/interface/RobustContactSchedule.h"

namespace ocs2::legged_robot {

struct RobustTouchdownTiming {
    size_t leg;
    scalar_t nominalTouchdown;
    scalar_t start;
    scalar_t end;
};

// Offset arithmetic such as 0.55+0.05 can differ from an existing 0.60
// event by one ULP. Canonicalize only arithmetic roundoff, never measured
// contact timestamps or finite sub-shooting-interval phases.
inline scalar_t canonicalRobustOffsetTime(const ModeSchedule& nominal, scalar_t time) {
    const scalar_t tolerance = 8.0 * std::numeric_limits<scalar_t>::epsilon() * std::max(1.0, std::abs(time));
    const auto next = std::lower_bound(nominal.eventTimes.begin(), nominal.eventTimes.end(), time);
    if (next != nominal.eventTimes.end() && std::abs(*next - time) <= tolerance) return *next;
    if (next != nominal.eventTimes.begin() && std::abs(*(next - 1) - time) <= tolerance) return *(next - 1);
    return time;
}

inline void validateRobustPhaseTiming(scalar_t advance, scalar_t delay, scalar_t d) {
    if (!std::isfinite(advance) || !std::isfinite(delay) || advance < 0.0 || delay < 0.0 ||
        !std::isfinite(advance + delay) || advance + delay <= 0.0) {
        throw std::invalid_argument("robustPhase.t_a/t_b must be finite, nonnegative seconds with a positive sum");
    }
    if (!std::isfinite(d) || d <= 0.0) {
        throw std::invalid_argument("robustPhase.d must be finite and positive");
    }
}

inline scalar_t robustPhaseVelocityLimit(scalar_t dMax, scalar_t start, scalar_t end) {
    if (!std::isfinite(dMax) || dMax <= 0.0 || !std::isfinite(start) || !std::isfinite(end) || end <= start) {
        throw std::invalid_argument("robust phase velocity requires positive d_max and a nonempty finite window");
    }
    const scalar_t speed = 2.0 * dMax / (end - start);
    if (!std::isfinite(speed)) throw std::invalid_argument("robust phase velocity is not finite");
    return speed;
}

// Derive absolute windows from the original gait on every solve, never from
// an already delayed or contact-spliced schedule (which would accumulate drift).
inline std::vector<RobustTouchdownTiming> makeRobustTouchdownTimings(
    const ModeSchedule& nominal, scalar_t advance, scalar_t delay) {
    validateRobustPhaseTiming(advance, delay, 1.0);
    std::vector<RobustTouchdownTiming> timings;
    for (size_t leg = 0; leg < contact_flag_t{}.size(); ++leg) {
        for (size_t phase = 1; phase < nominal.modeSequence.size(); ++phase) {
            const auto inContact = [&](size_t p) { return modeNumber2StanceLeg(nominal.modeSequence[p])[leg]; };
            if (!inContact(phase) || inContact(phase - 1)) continue;
            const scalar_t touchdown = nominal.eventTimes[phase - 1];
            size_t swingStart = phase - 1;
            while (swingStart > 0 && !inContact(swingStart - 1)) --swingStart;
            scalar_t start = canonicalRobustOffsetTime(nominal, touchdown - advance);
            if (swingStart > 0) {
                // An excessive advance must not reach the preceding stance.
                start = std::max(start, nominal.eventTimes[swingStart - 1]);
            }
            const scalar_t end = canonicalRobustOffsetTime(nominal, touchdown + delay);
            size_t nextSwing = phase + 1;
            while (nextSwing < nominal.modeSequence.size() && inContact(nextSwing)) ++nextSwing;
            if (nextSwing < nominal.modeSequence.size() && end >= nominal.eventTimes[nextSwing - 1]) {
                throw std::invalid_argument("robustPhase.t_b must be shorter than the selected gait's stance duration");
            }
            if (!std::isfinite(start) || !std::isfinite(end) || start >= end) {
                throw std::invalid_argument("robust phase offsets produced an empty or nonfinite window");
            }
            timings.push_back({leg, touchdown, start, end});
        }
    }
    return timings;
}

inline ModeSchedule delayRobustTouchdowns(const ModeSchedule& nominal,
                                         const std::vector<RobustTouchdownTiming>& timings) {
    ModeSchedule result = nominal;
    for (const auto& timing : timings) {
        // Keep the leg force-free until robust end unless contact splice
        // subsequently restores stance. The next liftoff is untouched.
        setLegContactInterval(result, timing.leg, timing.nominalTouchdown, timing.end, false);
    }
    return result;
}

}  // namespace ocs2::legged_robot

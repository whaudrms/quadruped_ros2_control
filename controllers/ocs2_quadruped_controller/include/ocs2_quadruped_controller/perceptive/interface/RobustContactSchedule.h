#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace ocs2::legged_robot {

// One confirmed contact belongs to one nominal touchdown. Never extend this
// interval when a later MPC solve recomputes the robust window.
struct RobustContactOverride {
    size_t leg;
    scalar_t eventTime;
    scalar_t windowStart;
    scalar_t touchdownTime;
};

// Split only at the requested endpoints and preserve all existing events.
// Exact event queries retain OCS2's pre-event convention.
inline void setLegContactInterval(ModeSchedule& schedule, size_t leg, scalar_t start, scalar_t end, bool contact) {
    if (start >= end) return;
    for (const scalar_t time : {start, end}) {
        const auto event = std::lower_bound(schedule.eventTimes.begin(), schedule.eventTimes.end(), time);
        const size_t index = static_cast<size_t>(event - schedule.eventTimes.begin());
        if (event == schedule.eventTimes.end() || *event != time) {
            const size_t mode = schedule.modeSequence[index];
            schedule.eventTimes.insert(event, time);
            schedule.modeSequence.insert(schedule.modeSequence.begin() + index + 1, mode);
        }
    }
    const size_t firstPhase = static_cast<size_t>(
        std::upper_bound(schedule.eventTimes.begin(), schedule.eventTimes.end(), start) - schedule.eventTimes.begin());
    for (size_t phase = firstPhase; phase < schedule.modeSequence.size(); ++phase) {
        if (schedule.eventTimes[phase - 1] >= end) break;
        auto flags = modeNumber2StanceLeg(schedule.modeSequence[phase]);
        flags[leg] = contact;
        schedule.modeSequence[phase] = stanceLeg2ModeNumber(flags);
    }
}

inline bool isValidRobustContactOverride(const ModeSchedule& nominal, const RobustContactOverride& contact) {
    if (contact.leg >= contact_flag_t{}.size() || !std::isfinite(contact.eventTime) ||
        !std::isfinite(contact.windowStart) || !std::isfinite(contact.touchdownTime) ||
        contact.eventTime < contact.windowStart || contact.eventTime >= contact.touchdownTime ||
        nominal.modeSequence.size() != nominal.eventTimes.size() + 1 ||
        std::any_of(nominal.eventTimes.begin(), nominal.eventTimes.end(),
                    [](scalar_t t) { return !std::isfinite(t); }) ||
        std::adjacent_find(nominal.eventTimes.begin(), nominal.eventTimes.end(),
                           [](scalar_t a, scalar_t b) { return a >= b; }) != nominal.eventTimes.end()) {
        return false;
    }
    const auto end = std::lower_bound(nominal.eventTimes.begin(), nominal.eventTimes.end(), contact.touchdownTime);
    if (end == nominal.eventTimes.end() || *end != contact.touchdownTime) return false;
    const size_t endPhase = static_cast<size_t>(end - nominal.eventTimes.begin()) + 1;
    if (!modeNumber2StanceLeg(nominal.modeSequence[endPhase])[contact.leg]) return false;

    // Inspect the post-event intervals. OCS2's modeAtTime() itself returns the
    // pre-event mode at an exact event; this patch preserves that convention.
    const size_t startPhase = static_cast<size_t>(
        std::upper_bound(nominal.eventTimes.begin(), nominal.eventTimes.end(), contact.eventTime) -
        nominal.eventTimes.begin());
    for (size_t phase = startPhase; phase < endPhase; ++phase) {
        if (modeNumber2StanceLeg(nominal.modeSequence[phase])[contact.leg]) return false;
    }
    return true;
}

// Preserve every nominal event and every other leg. Only the requested leg's
// intervals from its own contact time to its original touchdown become stance.
inline ModeSchedule applyRobustContactOverrides(const ModeSchedule& nominal,
                                                const std::vector<RobustContactOverride>& contacts) {
    ModeSchedule result = nominal;
    for (const auto& contact : contacts) {
        if (!isValidRobustContactOverride(nominal, contact)) continue;
        setLegContactInterval(result, contact.leg, contact.eventTime, contact.touchdownTime, true);
    }
    return result;
}

}  // namespace ocs2::legged_robot

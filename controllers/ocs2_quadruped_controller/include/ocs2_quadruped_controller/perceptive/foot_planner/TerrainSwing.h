#pragma once
#include <ocs2_quadruped_controller/perceptive/foot_planner/SwingSpline3d.h>
#include <functional>
#include <memory>
#include <utility>

namespace ocs2::legged_robot {
// Foot-frame coordinates throughout (terrain projections are lifted by the
// physical foot radius before constructing this trajectory).
struct TerrainSwing {
    scalar_t startTime, endTime;
    vector3_t startPosition, endPosition, startNormal, endNormal;
    perceptive_swing::SwingSpline3d spline;
    TerrainSwing(scalar_t start, scalar_t end, const vector3_t& from, const vector3_t& to,
                 const vector3_t& fromNormal, const vector3_t& toNormal,
                 scalar_t liftVelocity, scalar_t touchVelocity, scalar_t height, scalar_t timeScale,
                 const std::vector<std::pair<scalar_t, scalar_t>>& heightProfile);
    vector3_t normal(scalar_t time) const;
};
} // namespace ocs2::legged_robot

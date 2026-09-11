// Geometry/profile follows switched_model::SwingPhase::setFullSwing and
// SwingTrajectoryPlanner::applySwingMotionScaling in the local OCS2 ANYmal source.
#include <ocs2_quadruped_controller/perceptive/foot_planner/TerrainSwing.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ocs2::legged_robot {
namespace {
perceptive_swing::SwingSpline3d makeSpline(scalar_t start, scalar_t end,
    const vector3_t& from, const vector3_t& to, const vector3_t& toNormal,
    scalar_t liftVelocity, scalar_t touchVelocity, scalar_t height, scalar_t timeScale,
    const std::vector<std::pair<scalar_t, scalar_t>>& profile) {
    const scalar_t duration = end - start;
    if (!(duration > 0.0) || !(timeScale > 0.0)) throw std::invalid_argument("Invalid terrain swing duration/scale");
    const scalar_t scale = std::min(1.0, duration / timeScale);
    const vector3_t delta = to - from;
    vector3_t midPosition = to;
    vector3_t midVelocity = vector3_t::Zero();
    if (delta.head<2>().norm() > 0.01) {
        const vector3_t normal = delta.cross(vector3_t::UnitZ().cross(delta)).normalized();
        scalar_t obstacle = 0.0;
        for (const auto& point : profile) {
            if (std::isfinite(point.second))
                obstacle = std::max(obstacle, (point.second - (from.z() + point.first * delta.z())) * normal.z());
        }
        obstacle = std::min(obstacle, 2.0 * height);
        midPosition = from + 0.6 * delta + (obstacle + scale * height) * normal;
        midVelocity = 2.0 / duration * delta;
    } else {
        midPosition.z() = std::max(from.z(), to.z()) + scale * height;
    }
    using perceptive_swing::SwingNode3d;
    return perceptive_swing::SwingSpline3d(
        SwingNode3d{start, from, scale * liftVelocity * vector3_t::UnitZ()},
        SwingNode3d{0.5 * (start + end), midPosition, midVelocity},
        SwingNode3d{end, to, scale * touchVelocity * toNormal.normalized()});
}
}
TerrainSwing::TerrainSwing(scalar_t start, scalar_t end, const vector3_t& from, const vector3_t& to,
    const vector3_t& fromNormal, const vector3_t& toNormal, scalar_t liftVelocity, scalar_t touchVelocity,
    scalar_t height, scalar_t timeScale, const std::vector<std::pair<scalar_t, scalar_t>>& profile)
    : startTime(start), endTime(end), startPosition(from), endPosition(to),
      startNormal(fromNormal.normalized()), endNormal(toNormal.normalized()),
      spline(makeSpline(start, end, from, to, toNormal, liftVelocity, touchVelocity, height, timeScale, profile)) {}
vector3_t TerrainSwing::normal(scalar_t time) const {
    // Same smooth transition from lift-off normal to touchdown normal over 25%-75%.
    const scalar_t s = std::clamp(((time - startTime) / (endTime - startTime) - 0.25) / 0.5, 0.0, 1.0);
    const scalar_t a = s * s * (3.0 - 2.0 * s);
    return ((1.0 - a) * startNormal + a * endNormal).normalized();
}
} // namespace ocs2::legged_robot

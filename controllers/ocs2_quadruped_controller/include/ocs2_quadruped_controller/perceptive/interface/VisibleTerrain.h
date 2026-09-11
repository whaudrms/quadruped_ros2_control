#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

namespace ocs2::legged_robot {
// Subtract vertically occluded portions from boundaries AND projection insets.
// Preserve plane transforms; boolean subtraction can split a region or add holes.
std::vector<convex_plane_decomposition::PlanarRegion> visibleTerrainRegions(
    const std::vector<convex_plane_decomposition::PlanarRegion>& regions, double edgeMargin = 0.02);

// Compare against the unsmoothed map at its cell center, compensating for the
// candidate plane's slope. Invalid/out-of-map samples are not valid footholds.
bool projectionMatchesElevation(const convex_plane_decomposition::PlanarTerrain& terrain,
                                const convex_plane_decomposition::PlanarTerrainProjection& projection,
                                double heightTolerance = 0.01);

// Reject inconsistent candidates rather than silently returning a buried plane.
convex_plane_decomposition::PlanarTerrainProjection selectVisibleTerrainProjection(
    const convex_plane_decomposition::PlanarTerrain& terrain, const Eigen::Vector3d& query);
}  // namespace ocs2::legged_robot

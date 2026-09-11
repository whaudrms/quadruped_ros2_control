#include "ocs2_quadruped_controller/perceptive/interface/StaticPlanarTerrain.h"
#include "ocs2_quadruped_controller/perceptive/interface/VisibleTerrain.h"
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <iostream>
#include <stdexcept>

using namespace ocs2::legged_robot;
using namespace convex_plane_decomposition;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

BoxSurface box(double x, double z, double hx = 1.0, double hy = 1.0) {
  BoxSurface b;
  b.topCenterInWorld = Eigen::Vector3d(x, 0, z);
  b.halfX = hx;
  b.halfY = hy;
  return b;
}

void checkQueries(const PlanarTerrain& terrain) {
  for (double x = -0.2; x < 2.8; x += 0.07) {
    for (double y : {-0.8, -0.1, 0.4, 1.1}) {
      for (double z : {0.0, 0.05, 0.2}) {
        const auto p = selectVisibleTerrainProjection(terrain, Eigen::Vector3d(x, y, z));
        require(projectionMatchesElevation(terrain, p), "selected buried or inconsistent surface");
      }
    }
  }
}

int main(int argc, char** argv) {
  SceneDescription flat{true, {}};
  const auto flatTerrain = buildPlanarTerrain(flat, 0.03, "odom", 0.06);
  const auto flatProjection = selectVisibleTerrainProjection(flatTerrain, Eigen::Vector3d(0, 0, 0.03));
  require(flatProjection.positionInWorld.norm() < 1e-9, "flat-ground projection changed");

  // Actual step-up dimensions: leading edge .6, top .15. Query deliberately
  // closer to the hidden floor than to the top, reproducing the old failure.
  SceneDescription up{true, {box(1.6, 0.15)}};
  const auto terrain = buildPlanarTerrain(up, 0.03, "odom", 0.06);
  require(terrain.planarRegions.size() == 2, "unexpected step-up region count");
  const auto p = selectVisibleTerrainProjection(terrain, Eigen::Vector3d(1.2, 0, 0.02));
  require(std::abs(p.positionInWorld.z() - 0.15) < 1e-8, "hidden floor selected inside step");

  bool checkedFloor = false;
  for (const auto& region : terrain.planarRegions) {
    if (std::abs(region.transformPlaneToWorld.translation().z()) > 1e-9) continue;
    checkedFloor = true;
    require(region.boundaryWithInset.boundary.number_of_holes() == 1, "floor boundary lacks obstacle hole");
    require(region.boundaryWithInset.insets.front().number_of_holes() == 1, "projection inset lacks obstacle hole");
    const auto local = region.transformPlaneToWorld.inverse() * Eigen::Vector3d(1.2, 0, 0);
    const auto projected = projectToPlanarRegion(CgalPoint2d(local.x(), local.y()), region);
    const Eigen::Vector3d world = region.transformPlaneToWorld * Eigen::Vector3d(projected.x(), projected.y(), 0);
    require(world.x() <= 0.580001, "floor projection can reach covered region or riser");
    const auto polygon = growConvexPolygonInsideShape(region.boundaryWithInset.boundary, projected, 6, 1.05);
    for (const auto& a : polygon) for (const auto& b : polygon) {
      for (double alpha : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        const Eigen::Vector3d point = region.transformPlaneToWorld *
            Eigen::Vector3d(alpha*a.x()+(1-alpha)*b.x(), alpha*a.y()+(1-alpha)*b.y(), 0);
        require(!(point.x() > .600001 && point.x() < 2.599999 && std::abs(point.y()) < .999999),
                "foot-placement polygon crosses covered floor");
      }
    }
  }
  require(checkedFloor, "visible floor outside obstacle disappeared");
  checkQueries(terrain);
  const auto roundtrip = fromMessage(toMessage(terrain));
  checkQueries(roundtrip);
  require(roundtrip.planarRegions.front().boundaryWithInset.boundary.number_of_holes() == 1,
          "ROS message conversion lost floor hole");

  // Height-map rejection is independent of publisher clipping: a stale planar
  // map containing only floor must fail instead of supplying an unsafe fallback.
  auto stale = flatTerrain;
  stale.gridMap["elevation"].setConstant(0.15);
  bool rejected = false;
  try { selectVisibleTerrainProjection(stale, Eigen::Vector3d(0, 0, 0)); }
  catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "map inconsistency silently accepted");

  // Step down and overlapping/rotated platforms, including a lower surface
  // covered by a higher one. The visible floor must remain outside both.
  SceneDescription down{true, {box(-0.4, 0.2), box(1.6, 0.1)}};
  checkQueries(buildPlanarTerrain(down, .03, "odom", .06));
  SceneDescription overlapping{true, {box(1.2, .1), box(1.4, .2, .5, .6)}};
  overlapping.surfaces[1].rotationPlaneToWorld =
      Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto overlapTerrain = buildPlanarTerrain(overlapping, .03, "odom", .06);
  require(std::abs(selectVisibleTerrainProjection(overlapTerrain, Eigen::Vector3d(1.4, 0, 0)).positionInWorld.z()-.2)<1e-8,
          "lower box selected under higher box");
  checkQueries(overlapTerrain);

  SceneDescription ramp{true, {box(1.2, .05)}};
  ramp.surfaces[0].rotationPlaneToWorld = Eigen::AngleAxisd(.15, Eigen::Vector3d::UnitY()).toRotationMatrix();
  checkQueries(buildPlanarTerrain(ramp, .03, "odom", .06));

  // Perception offsets must affect both representations equally.
  up.surfaces[0].topCenterInWorld.z() -= .048;
  const auto shifted = buildPlanarTerrain(up, .03, "odom", .06);
  require(std::abs(selectVisibleTerrainProjection(shifted, Eigen::Vector3d(1.2, 0, 0)).positionInWorld.z()-.102)<1e-8,
          "perception offset was not retained");
  checkQueries(shifted);
  for (int i = 1; i < argc; ++i) {
    checkQueries(buildPlanarTerrain(loadSceneDescription(argv[i]), .03, "odom", .06));
    std::cout << "scene verified: " << argv[i] << '\n';
  }
  std::cout << "Visible terrain regression checks passed\n";
}

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace ocs2::legged_robot {
struct BoxSurface {
  Eigen::Vector3d topCenterInWorld = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotationPlaneToWorld = Eigen::Matrix3d::Identity();
  double halfX = 0.0;
  double halfY = 0.0;
};

struct SceneDescription {
  bool hasFloorPlane = false;
  std::vector<BoxSurface> surfaces;
};

SceneDescription loadSceneDescription(const std::string& sceneFile);
convex_plane_decomposition::PlanarTerrain buildPlanarTerrain(
    const SceneDescription& scene, double resolution, const std::string& frameId, double smoothingRadius);
}  // namespace ocs2::legged_robot

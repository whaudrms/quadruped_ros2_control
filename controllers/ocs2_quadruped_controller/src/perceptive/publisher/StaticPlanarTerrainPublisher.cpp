#include <rclcpp/rclcpp.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using convex_plane_decomposition::CgalBbox2d;
using convex_plane_decomposition::CgalPoint2d;
using convex_plane_decomposition::CgalPolygonWithHoles2d;
using convex_plane_decomposition::PlanarRegion;
using convex_plane_decomposition::PlanarTerrain;

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

std::string trim(const std::string& text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::vector<double> parseDoubles(const std::string& text) {
  std::istringstream stream(text);
  std::vector<double> values;
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

std::unordered_map<std::string, std::string> parseAttributes(const std::string& tag) {
  static const std::regex attributeRegex(R"ATTR(([A-Za-z_][A-Za-z0-9_:-]*)\s*=\s*"([^"]*)")ATTR");
  std::unordered_map<std::string, std::string> attributes;
  for (std::sregex_iterator it(tag.begin(), tag.end(), attributeRegex), end; it != end; ++it) {
    attributes[(*it)[1].str()] = (*it)[2].str();
  }
  return attributes;
}

SceneDescription loadSceneDescription(const std::string& sceneFile) {
  std::ifstream stream(sceneFile);
  if (!stream.good()) {
    throw std::runtime_error("Failed to open scene file: " + sceneFile);
  }

  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string xml = buffer.str();

  static const std::regex geomRegex(R"(<geom\b[^>]*/>)");
  SceneDescription scene;

  for (std::sregex_iterator it(xml.begin(), xml.end(), geomRegex), end; it != end; ++it) {
    const std::string tag = it->str();
    const auto attributes = parseAttributes(tag);
    const auto typeIt = attributes.find("type");
    if (typeIt == attributes.end()) {
      continue;
    }

    const std::string type = trim(typeIt->second);
    if (type == "plane") {
      scene.hasFloorPlane = true;
      continue;
    }

    if (type != "box") {
      continue;
    }

    const auto posIt = attributes.find("pos");
    const auto sizeIt = attributes.find("size");
    if (posIt == attributes.end() || sizeIt == attributes.end()) {
      continue;
    }

    const auto pos = parseDoubles(posIt->second);
    const auto size = parseDoubles(sizeIt->second);
    if (pos.size() != 3 || size.size() != 3) {
      continue;
    }

    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    const auto quatIt = attributes.find("quat");
    if (quatIt != attributes.end()) {
      const auto quat = parseDoubles(quatIt->second);
      if (quat.size() == 4) {
        quaternion = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        quaternion.normalize();
      }
    }

    BoxSurface surface;
    surface.rotationPlaneToWorld = quaternion.toRotationMatrix();
    surface.halfX = size[0];
    surface.halfY = size[1];
    const Eigen::Vector3d centerInWorld(pos[0], pos[1], pos[2]);
    const Eigen::Vector3d topOffset = surface.rotationPlaneToWorld * Eigen::Vector3d(0.0, 0.0, size[2]);
    surface.topCenterInWorld = centerInWorld + topOffset;
    scene.surfaces.push_back(surface);
  }

  return scene;
}

PlanarRegion makeRectangularRegion(const Eigen::Vector3d& centerInWorld,
                                   const Eigen::Matrix3d& rotationPlaneToWorld,
                                   double halfX,
                                   double halfY,
                                   double insetMargin) {
  PlanarRegion region;
  region.transformPlaneToWorld.setIdentity();
  region.transformPlaneToWorld.linear() = rotationPlaneToWorld;
  region.transformPlaneToWorld.translation() = centerInWorld;
  region.bbox2d = CgalBbox2d(-halfX, -halfY, halfX, halfY);

  CgalPolygonWithHoles2d boundary;
  boundary.outer_boundary().push_back(CgalPoint2d(halfX, halfY));
  boundary.outer_boundary().push_back(CgalPoint2d(-halfX, halfY));
  boundary.outer_boundary().push_back(CgalPoint2d(-halfX, -halfY));
  boundary.outer_boundary().push_back(CgalPoint2d(halfX, -halfY));
  region.boundaryWithInset.boundary = boundary;

  const double insetX = std::max(halfX - insetMargin, halfX * 0.5);
  const double insetY = std::max(halfY - insetMargin, halfY * 0.5);
  CgalPolygonWithHoles2d inset;
  inset.outer_boundary().push_back(CgalPoint2d(insetX, insetY));
  inset.outer_boundary().push_back(CgalPoint2d(-insetX, insetY));
  inset.outer_boundary().push_back(CgalPoint2d(-insetX, -insetY));
  inset.outer_boundary().push_back(CgalPoint2d(insetX, -insetY));
  region.boundaryWithInset.insets.push_back(inset);

  return region;
}

std::array<Eigen::Vector2d, 4> topCornersXY(const BoxSurface& surface) {
  std::array<Eigen::Vector2d, 4> corners{};
  const std::array<Eigen::Vector2d, 4> localCorners = {
      Eigen::Vector2d(surface.halfX, surface.halfY),
      Eigen::Vector2d(-surface.halfX, surface.halfY),
      Eigen::Vector2d(-surface.halfX, -surface.halfY),
      Eigen::Vector2d(surface.halfX, -surface.halfY),
  };

  for (size_t i = 0; i < localCorners.size(); ++i) {
    const Eigen::Vector3d world =
        surface.topCenterInWorld +
        surface.rotationPlaneToWorld.col(0) * localCorners[i].x() +
        surface.rotationPlaneToWorld.col(1) * localCorners[i].y();
    corners[i] = world.head<2>();
  }
  return corners;
}

bool surfaceHeightAt(const BoxSurface& surface, double x, double y, double& z) {
  Eigen::Matrix2d projection;
  projection << surface.rotationPlaneToWorld(0, 0), surface.rotationPlaneToWorld(0, 1),
      surface.rotationPlaneToWorld(1, 0), surface.rotationPlaneToWorld(1, 1);

  if (std::abs(projection.determinant()) < 1e-9) {
    return false;
  }

  const Eigen::Vector2d rhs(x - surface.topCenterInWorld.x(), y - surface.topCenterInWorld.y());
  const Eigen::Vector2d local = projection.inverse() * rhs;
  constexpr double tolerance = 1e-6;
  if (std::abs(local.x()) > surface.halfX + tolerance || std::abs(local.y()) > surface.halfY + tolerance) {
    return false;
  }

  z = surface.topCenterInWorld.z() +
      surface.rotationPlaneToWorld(2, 0) * local.x() +
      surface.rotationPlaneToWorld(2, 1) * local.y();
  return true;
}

PlanarTerrain buildPlanarTerrain(const SceneDescription& scene, double resolution, const std::string& frameId) {
  double minX = std::numeric_limits<double>::infinity();
  double maxX = -std::numeric_limits<double>::infinity();
  double minY = std::numeric_limits<double>::infinity();
  double maxY = -std::numeric_limits<double>::infinity();

  for (const auto& surface : scene.surfaces) {
    for (const auto& corner : topCornersXY(surface)) {
      minX = std::min(minX, corner.x());
      maxX = std::max(maxX, corner.x());
      minY = std::min(minY, corner.y());
      maxY = std::max(maxY, corner.y());
    }
  }

  if (!std::isfinite(minX)) {
    minX = -3.0;
    maxX = 3.0;
    minY = -2.0;
    maxY = 2.0;
  }

  constexpr double floorMargin = 1.0;
  minX -= floorMargin;
  maxX += floorMargin;
  minY -= floorMargin;
  maxY += floorMargin;

  const double centerX = 0.5 * (minX + maxX);
  const double centerY = 0.5 * (minY + maxY);
  const double halfX = 0.5 * (maxX - minX);
  const double halfY = 0.5 * (maxY - minY);

  PlanarTerrain terrain;
  terrain.planarRegions.reserve(scene.surfaces.size() + (scene.hasFloorPlane ? 1 : 0));

  if (scene.hasFloorPlane || scene.surfaces.empty()) {
    terrain.planarRegions.push_back(
        makeRectangularRegion(Eigen::Vector3d(centerX, centerY, 0.0), Eigen::Matrix3d::Identity(), halfX, halfY, 0.02));
  }

  for (const auto& surface : scene.surfaces) {
    const double insetMargin = std::min({0.02, surface.halfX * 0.2, surface.halfY * 0.2});
    terrain.planarRegions.push_back(
        makeRectangularRegion(surface.topCenterInWorld, surface.rotationPlaneToWorld, surface.halfX, surface.halfY,
                              std::max(0.005, insetMargin)));
  }

  terrain.gridMap.setFrameId(frameId);
  terrain.gridMap.setGeometry(grid_map::Length(2.0 * halfX, 2.0 * halfY), resolution,
                              grid_map::Position(centerX, centerY));
  terrain.gridMap.add("elevation", 0.0);
  terrain.gridMap.add("smooth_planar", 0.0);

  for (grid_map::GridMapIterator it(terrain.gridMap); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    terrain.gridMap.getPosition(*it, position);

    double height = 0.0;
    bool found = scene.hasFloorPlane || scene.surfaces.empty();
    for (const auto& surface : scene.surfaces) {
      double candidateHeight = 0.0;
      if (surfaceHeightAt(surface, position.x(), position.y(), candidateHeight)) {
        if (!found || candidateHeight > height) {
          height = candidateHeight;
          found = true;
        }
      }
    }

    terrain.gridMap.at("elevation", *it) = static_cast<float>(height);
    terrain.gridMap.at("smooth_planar", *it) = static_cast<float>(height);
  }

  return terrain;
}

class StaticPlanarTerrainPublisher final : public rclcpp::Node {
 public:
  StaticPlanarTerrainPublisher()
      : Node("planar_terrain_publisher") {
    const auto sceneFile = this->declare_parameter<std::string>(
        "scene_xml", "/home/tony/unitree_mujoco/unitree_robots/go2/basic_step.xml");
    const auto topic = this->declare_parameter<std::string>(
        "terrain_topic", "/convex_plane_decomposition_ros/planar_terrain");
    const auto frameId = this->declare_parameter<std::string>("frame_id", "map");
    const auto resolution = this->declare_parameter<double>("resolution", 0.03);
    const auto publishRate = this->declare_parameter<double>("publish_rate", 2.0);

    const SceneDescription scene = loadSceneDescription(sceneFile);
    terrain_ = buildPlanarTerrain(scene, resolution, frameId);
    terrainMsg_ = convex_plane_decomposition::toMessage(terrain_);

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    publisher_ =
        this->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>(topic, qos);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publishRate));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { publisher_->publish(terrainMsg_); });

    publisher_->publish(terrainMsg_);
    RCLCPP_INFO(this->get_logger(),
                "Publishing static planar terrain from '%s' to '%s' with %zu planar regions.",
                sceneFile.c_str(), topic.c_str(), terrain_.planarRegions.size());
  }

 private:
  PlanarTerrain terrain_;
  convex_plane_decomposition_msgs::msg::PlanarTerrain terrainMsg_;
  rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPlanarTerrainPublisher>());
  rclcpp::shutdown();
  return 0;
}

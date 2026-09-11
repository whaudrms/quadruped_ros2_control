#include <rclcpp/rclcpp.hpp>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include "ocs2_quadruped_controller/perceptive/interface/StaticPlanarTerrain.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {
using convex_plane_decomposition::PlanarTerrain;
using ocs2::legged_robot::SceneDescription;
using ocs2::legged_robot::loadSceneDescription;
using ocs2::legged_robot::buildPlanarTerrain;

class StaticPlanarTerrainPublisher final : public rclcpp::Node {
 public:
  StaticPlanarTerrainPublisher()
      : Node("planar_terrain_publisher") {
    const auto sceneFile = this->declare_parameter<std::string>("scene_xml", "");
    if (sceneFile.empty()) {
      RCLCPP_FATAL(this->get_logger(),
                   "scene_xml parameter is required (absolute path to MuJoCo scene XML)");
      throw std::runtime_error("scene_xml parameter is required");
    }
    const auto topic = this->declare_parameter<std::string>(
        "terrain_topic", "/convex_plane_decomposition_ros/planar_terrain");
    const auto frameId = this->declare_parameter<std::string>("frame_id", "odom");
    const auto resolution = this->declare_parameter<double>("resolution", 0.03);
    const auto smoothingRadius = this->declare_parameter<double>("smoothing_radius", 0.12);
    const auto publishRate = this->declare_parameter<double>("publish_rate", 0.0);
    // Perception uncertainty: shift non-floor box top z by this offset before
    // building the planar terrain. MuJoCo physics is unaffected so the actual
    // ground stays at the true XML height; the controller perceives a wrong z.
    //
    // `terrain_z_offset_only_below_z` restricts the offset to surfaces whose
    // true top z is BELOW this threshold (in meters). Default is +infinity, so
    // the original behavior (offset applied to all non-floor surfaces) is
    // preserved. Use case: in basic_step_short (box1 top at z=0.20, box2 top
    // at z=0.10), passing 0.15 applies the offset only to box2 — simulating
    // perception uncertainty on the descent target while leaving the start
    // platform unchanged.
    const auto terrainZOffset = this->declare_parameter<double>("terrain_z_offset", 0.0);
    const auto terrainZOffsetOnlyBelowZ =
        this->declare_parameter<double>("terrain_z_offset_only_below_z",
                                        std::numeric_limits<double>::infinity());

    SceneDescription scene = loadSceneDescription(sceneFile);
    if (std::abs(terrainZOffset) > 0.0) {
      size_t affected = 0;
      for (auto& surface : scene.surfaces) {
        if (surface.topCenterInWorld.z() < terrainZOffsetOnlyBelowZ) {
          surface.topCenterInWorld.z() += terrainZOffset;
          ++affected;
        }
      }
      RCLCPP_WARN(this->get_logger(),
                  "Applied terrain_z_offset=%+.4f m to %zu / %zu non-floor surfaces "
                  "(filter: top z < %.3f m; perception only — MuJoCo physics unchanged).",
                  terrainZOffset, affected, scene.surfaces.size(),
                  terrainZOffsetOnlyBelowZ);
    }
    terrain_ = buildPlanarTerrain(scene, resolution, frameId, smoothingRadius);
    terrainMsg_ = convex_plane_decomposition::toMessage(terrain_);

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    publisher_ =
        this->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>(topic, qos);

    publisher_->publish(terrainMsg_);
    if (publishRate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publishRate));
      timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this]() { publisher_->publish(terrainMsg_); });
    }

    RCLCPP_INFO(this->get_logger(),
                "Publishing static planar terrain from '%s' to '%s' with %zu visible planar regions "
                "(resolution=%.3f m, smoothing_radius=%.3f m, publish_rate=%.2f Hz).",
                sceneFile.c_str(), topic.c_str(), terrain_.planarRegions.size(), resolution,
                smoothingRadius, publishRate);
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

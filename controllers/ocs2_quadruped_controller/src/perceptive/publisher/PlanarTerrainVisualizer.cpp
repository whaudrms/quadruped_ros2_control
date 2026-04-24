#include <optional>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
class PlanarTerrainVisualizer final : public rclcpp::Node
{
public:
  PlanarTerrainVisualizer()
  : Node("planar_terrain_visualizer")
  {
    const auto terrainTopic = declare_parameter<std::string>(
        "terrain_topic", "/convex_plane_decomposition_ros/planar_terrain");
    const auto gridMapTopic = declare_parameter<std::string>(
        "grid_map_topic", "/convex_plane_decomposition_ros/filtered_map");
    const auto boundaryTopic = declare_parameter<std::string>(
        "boundary_topic", "/convex_plane_decomposition_ros/boundaries");
    const auto insetTopic = declare_parameter<std::string>(
        "inset_topic", "/convex_plane_decomposition_ros/insets");
    const auto republishRate = declare_parameter<double>("republish_rate", 1.0);
    lineWidth_ = declare_parameter<double>("line_width", 0.01);

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();

    gridMapPublisher_ = create_publisher<grid_map_msgs::msg::GridMap>(gridMapTopic, qos);
    boundaryPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(boundaryTopic, qos);
    insetPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(insetTopic, qos);

    terrainSubscription_ = create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
        terrainTopic, qos,
        [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg)
        {
          latestTerrain_ = convex_plane_decomposition::fromMessage(*msg);
          publishLatestTerrain();
        });

    if (republishRate > 0.0)
    {
      const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, republishRate));
      timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this]() { publishLatestTerrain(); });
    }
  }

private:
  void publishLatestTerrain()
  {
    if (!latestTerrain_.has_value())
    {
      return;
    }

    const auto& terrain = *latestTerrain_;
    auto gridMapMsg = *grid_map::GridMapRosConverter::toMessage(terrain.gridMap);
    gridMapPublisher_->publish(gridMapMsg);

    const auto frameId = terrain.gridMap.getFrameId();
    const auto time = terrain.gridMap.getTimestamp();
    boundaryPublisher_->publish(convex_plane_decomposition::convertBoundariesToRosMarkers(
        terrain.planarRegions, frameId, time, lineWidth_));
    insetPublisher_->publish(convex_plane_decomposition::convertInsetsToRosMarkers(
        terrain.planarRegions, frameId, time, lineWidth_));
  }

  double lineWidth_{0.01};
  std::optional<convex_plane_decomposition::PlanarTerrain> latestTerrain_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr terrainSubscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr insetPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarTerrainVisualizer>());
  rclcpp::shutdown();
  return 0;
}

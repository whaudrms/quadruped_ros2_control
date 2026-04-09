#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

grid_map::GridMap makeBoxGridMap(const std::string& elevation_layer,
                                 const std::string& frame_id,
                                 double resolution,
                                 double map_length_x,
                                 double map_length_y,
                                 double z_offset,
                                 const std::string& uncertainty_layer,
                                 double uncertainty_base,
                                 double uncertainty_gradient_weight,
                                 double box_center_x,
                                 double box_center_y,
                                 double box_size_x,
                                 double box_size_y,
                                 double box_height) {
  std::vector<std::string> layers{elevation_layer};
  const bool publish_uncertainty =
      !uncertainty_layer.empty() && (uncertainty_base > 0.0 || uncertainty_gradient_weight > 0.0);
  if (publish_uncertainty) {
    layers.push_back(uncertainty_layer);
  }

  grid_map::GridMap grid_map(layers);
  grid_map.setFrameId(frame_id);
  grid_map.setGeometry(grid_map::Length(map_length_x, map_length_y), resolution, grid_map::Position(0.0, 0.0));
  grid_map[elevation_layer].setConstant(static_cast<float>(z_offset));

  const double x_min = box_center_x - 0.5 * box_size_x;
  const double x_max = box_center_x + 0.5 * box_size_x;
  const double y_min = box_center_y - 0.5 * box_size_y;
  const double y_max = box_center_y + 0.5 * box_size_y;

  for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    grid_map::Position position;
    grid_map.getPosition(index, position);

    const bool in_box = position.x() >= x_min && position.x() <= x_max &&
                        position.y() >= y_min && position.y() <= y_max;
    if (in_box) {
      grid_map.at(elevation_layer, index) = static_cast<float>(z_offset + box_height);
    }

    if (publish_uncertainty) {
      // Keep uncertainty simple: zero inside flat regions, slightly raised near box edges.
      double edge_dx = 0.0;
      if (position.x() < x_min) edge_dx = x_min - position.x();
      else if (position.x() > x_max) edge_dx = position.x() - x_max;
      else edge_dx = std::min(position.x() - x_min, x_max - position.x());

      double edge_dy = 0.0;
      if (position.y() < y_min) edge_dy = y_min - position.y();
      else if (position.y() > y_max) edge_dy = position.y() - y_max;
      else edge_dy = std::min(position.y() - y_min, y_max - position.y());

      const double edge_distance = std::min(edge_dx, edge_dy);
      const double edge_band = std::max(resolution * 2.0, 1e-3);
      const double edge_factor = std::clamp(1.0 - edge_distance / edge_band, 0.0, 1.0);
      grid_map.at(uncertainty_layer, index) =
          static_cast<float>(std::clamp(uncertainty_base + uncertainty_gradient_weight * edge_factor, 0.0, 1.0));
    }
  }

  return grid_map;
}

grid_map::GridMap loadGridMapFromImage(const std::string& image_path,
                                       const std::string& elevation_layer,
                                       const std::string& frame_id,
                                       double resolution,
                                       double scale,
                                       double z_offset,
                                       const std::string& uncertainty_layer,
                                       double uncertainty_base,
                                       double uncertainty_gradient_weight,
                                       int uncertainty_blur_kernel) {
  cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    throw std::runtime_error("Could not open terrain image: " + image_path);
  }

  std::vector<std::string> layers{elevation_layer};
  const bool publish_uncertainty =
      !uncertainty_layer.empty() && (uncertainty_base > 0.0 || uncertainty_gradient_weight > 0.0);
  if (publish_uncertainty) {
    layers.push_back(uncertainty_layer);
  }

  grid_map::GridMap grid_map(layers);
  grid_map.setFrameId(frame_id);
  grid_map::GridMapCvConverter::initializeFromImage(
      image, resolution, grid_map, grid_map::Position(0.0, 0.0));
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
      image, elevation_layer, grid_map, 0.0f, static_cast<float>(scale), 0.5);
  grid_map[elevation_layer].array() += static_cast<float>(z_offset);

  if (publish_uncertainty) {
    cv::Mat image_float;
    image.convertTo(image_float, CV_32FC1, 1.0 / 255.0);

    int blur_kernel = std::max(1, uncertainty_blur_kernel);
    if (blur_kernel % 2 == 0) {
      blur_kernel += 1;
    }
    if (blur_kernel > 1) {
      cv::GaussianBlur(image_float, image_float, cv::Size(blur_kernel, blur_kernel), 0.0, 0.0);
    }

    cv::Mat grad_x;
    cv::Mat grad_y;
    cv::Sobel(image_float, grad_x, CV_32FC1, 1, 0, 3);
    cv::Sobel(image_float, grad_y, CV_32FC1, 0, 1, 3);
    cv::Mat grad_mag;
    cv::magnitude(grad_x, grad_y, grad_mag);

    double max_grad = 0.0;
    cv::minMaxLoc(grad_mag, nullptr, &max_grad);
    if (max_grad > 1e-6) {
      grad_mag /= static_cast<float>(max_grad);
    } else {
      grad_mag = cv::Mat::zeros(grad_mag.size(), grad_mag.type());
    }

    grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(
        grad_mag, uncertainty_layer, grid_map, static_cast<float>(uncertainty_base),
        static_cast<float>(uncertainty_base + uncertainty_gradient_weight));

    auto& uncertainty = grid_map[uncertainty_layer];
    uncertainty = uncertainty.array().max(0.0f).min(1.0f);
  }
  return grid_map;
}

}  // namespace

class FakeElevationMapNode final : public rclcpp::Node {
public:
  FakeElevationMapNode() : Node("fake_elevation_map_node") {
    map_mode_ = declare_parameter<std::string>("map_mode", "image");
    image_path_ = declare_parameter<std::string>("image_path", "");
    topic_name_ = declare_parameter<std::string>("topic_name", "/elevation_mapping/elevation_map_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    resolution_ = declare_parameter<double>("resolution", 0.03);
    scale_ = declare_parameter<double>("height_scale", 0.35);
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);
    uncertainty_layer_ = declare_parameter<std::string>("uncertainty_layer", "uncertainty");
    uncertainty_base_ = declare_parameter<double>("uncertainty_base", 0.0);
    uncertainty_gradient_weight_ = declare_parameter<double>("uncertainty_gradient_weight", 0.0);
    uncertainty_blur_kernel_ = declare_parameter<int>("uncertainty_blur_kernel", 5);
    map_length_x_ = declare_parameter<double>("map_length_x", 2.5);
    map_length_y_ = declare_parameter<double>("map_length_y", 1.6);
    box_center_x_ = declare_parameter<double>("box_center_x", 0.55);
    box_center_y_ = declare_parameter<double>("box_center_y", 0.0);
    box_size_x_ = declare_parameter<double>("box_size_x", 0.36);
    box_size_y_ = declare_parameter<double>("box_size_y", 1.10);
    box_height_ = declare_parameter<double>("box_height", 0.08);

    if (map_mode_ == "image") {
      if (image_path_.empty()) {
        throw std::runtime_error("Parameter 'image_path' must not be empty when map_mode=image.");
      }

      grid_map_ = loadGridMapFromImage(image_path_, elevation_layer_, frame_id_,
                                       resolution_, scale_, z_offset_, uncertainty_layer_,
                                       uncertainty_base_, uncertainty_gradient_weight_,
                                       uncertainty_blur_kernel_);
      RCLCPP_INFO(get_logger(), "Loaded terrain image: %s", image_path_.c_str());
    } else if (map_mode_ == "box") {
      grid_map_ = makeBoxGridMap(elevation_layer_, frame_id_, resolution_,
                                 map_length_x_, map_length_y_, z_offset_,
                                 uncertainty_layer_, uncertainty_base_, uncertainty_gradient_weight_,
                                 box_center_x_, box_center_y_, box_size_x_, box_size_y_, box_height_);
      RCLCPP_INFO(get_logger(),
                  "Generated box terrain map: center=(%.3f, %.3f) size=(%.3f, %.3f) height=%.3f map=(%.3f x %.3f)",
                  box_center_x_, box_center_y_, box_size_x_, box_size_y_, box_height_, map_length_x_, map_length_y_);
    } else {
      throw std::runtime_error("Unsupported map_mode: " + map_mode_);
    }
    publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(topic_name_, 1);

    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
        std::bind(&FakeElevationMapNode::publishMap, this));
    RCLCPP_INFO(get_logger(), "Publishing static elevation map on: %s", topic_name_.c_str());
    if (uncertainty_base_ > 0.0 || uncertainty_gradient_weight_ > 0.0) {
      RCLCPP_INFO(
          get_logger(),
          "Publishing uncertainty layer '%s' with base=%.3f gradient_weight=%.3f blur_kernel=%d",
          uncertainty_layer_.c_str(), uncertainty_base_, uncertainty_gradient_weight_, uncertainty_blur_kernel_);
    }
  }

private:
  void publishMap() {
    grid_map_.setTimestamp(get_clock()->now().nanoseconds());
    auto message = grid_map::GridMapRosConverter::toMessage(grid_map_);
    publisher_->publish(*message);
  }

  std::string map_mode_;
  std::string image_path_;
  std::string topic_name_;
  std::string frame_id_;
  std::string elevation_layer_;
  double resolution_{0.03};
  double scale_{0.35};
  double z_offset_{0.0};
  double publish_rate_{2.0};
  std::string uncertainty_layer_;
  double uncertainty_base_{0.0};
  double uncertainty_gradient_weight_{0.0};
  int uncertainty_blur_kernel_{5};
  double map_length_x_{2.5};
  double map_length_y_{1.6};
  double box_center_x_{0.55};
  double box_center_y_{0.0};
  double box_size_x_{0.36};
  double box_size_y_{1.10};
  double box_height_{0.08};

  grid_map::GridMap grid_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeElevationMapNode>());
  rclcpp::shutdown();
  return 0;
}

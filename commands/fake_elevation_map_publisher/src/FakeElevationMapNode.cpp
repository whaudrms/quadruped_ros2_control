#include <memory>
#include <string>
#include <algorithm>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

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

    if (image_path_.empty()) {
      throw std::runtime_error("Parameter 'image_path' must not be empty.");
    }

    grid_map_ = loadGridMapFromImage(image_path_, elevation_layer_, frame_id_,
                                     resolution_, scale_, z_offset_, uncertainty_layer_,
                                     uncertainty_base_, uncertainty_gradient_weight_,
                                     uncertainty_blur_kernel_);
    publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(topic_name_, 1);

    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
        std::bind(&FakeElevationMapNode::publishMap, this));

    RCLCPP_INFO(get_logger(), "Loaded terrain image: %s", image_path_.c_str());
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

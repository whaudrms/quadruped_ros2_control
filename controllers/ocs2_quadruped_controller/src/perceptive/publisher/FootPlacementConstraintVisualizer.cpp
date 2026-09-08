#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using convex_plane_decomposition::CgalPoint2d;
using convex_plane_decomposition::CgalPolygon2d;
using convex_plane_decomposition::PlanarTerrain;

struct HalfSpacePolygon
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA value;
  value.r = red;
  value.g = green;
  value.b = blue;
  value.a = alpha;
  return value;
}

geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

HalfSpacePolygon polygonToHalfSpaces(const CgalPolygon2d& polygon)
{
  const auto numberOfVertices = static_cast<Eigen::Index>(polygon.size());
  HalfSpacePolygon result{
      Eigen::MatrixXd::Zero(numberOfVertices, 2), Eigen::VectorXd::Zero(numberOfVertices)};

  for (Eigen::Index i = 0; i < numberOfVertices; ++i)
  {
    const Eigen::Index j = (i + 1) % numberOfVertices;
    const Eigen::Index k = (j + 1) % numberOfVertices;
    const auto pointA = polygon.vertex(static_cast<std::size_t>(i));
    const auto pointB = polygon.vertex(static_cast<std::size_t>(j));
    const auto pointC = polygon.vertex(static_cast<std::size_t>(k));

    result.A.row(i) << pointB.y() - pointA.y(), pointA.x() - pointB.x();
    result.b(i) = pointA.y() * pointB.x() - pointA.x() * pointB.y();

    const Eigen::Vector2d testPoint(pointC.x(), pointC.y());
    if ((result.A.row(i) * testPoint)(0) + result.b(i) < 0.0)
    {
      result.A.row(i) *= -1.0;
      result.b(i) *= -1.0;
    }
  }
  return result;
}

bool shrinkHalfSpaces(const HalfSpacePolygon& original, const Eigen::Vector2d& interiorPoint,
                      double boundaryMargin, HalfSpacePolygon& shrunk)
{
  shrunk = original;
  if (boundaryMargin <= 0.0)
  {
    return true;
  }

  for (Eigen::Index row = 0; row < original.A.rows(); ++row)
  {
    const double normalNorm = original.A.row(row).norm();
    if (normalNorm <= 1e-9)
    {
      return false;
    }
    shrunk.b(row) -= boundaryMargin * normalNorm;
  }

  const Eigen::VectorXd slack = shrunk.A * interiorPoint + shrunk.b;
  return (slack.array() > 1e-6).all();
}

// Clip the original convex polygon by every active half-space. Intersecting only
// neighbouring lines is incorrect when an inset makes some edges redundant.
std::optional<CgalPolygon2d> halfSpacesToPolygon(const HalfSpacePolygon& polygon,
                                               const CgalPolygon2d& original)
{
  const Eigen::Index numberOfEdges = polygon.A.rows();
  if (numberOfEdges < 3 || polygon.b.size() != numberOfEdges)
  {
    return std::nullopt;
  }

  std::vector<Eigen::Vector2d> vertices;
  for (const auto& point : original)
  {
    vertices.emplace_back(point.x(), point.y());
  }
  for (Eigen::Index i = 0; i < numberOfEdges; ++i)
  {
    if (vertices.empty())
    {
      return std::nullopt;
    }
    std::vector<Eigen::Vector2d> clipped;
    Eigen::Vector2d previous = vertices.back();
    double previousSlack = polygon.A.row(i).dot(previous) + polygon.b(i);
    for (const auto& current : vertices)
    {
      const double currentSlack = polygon.A.row(i).dot(current) + polygon.b(i);
      const bool previousInside = previousSlack >= 0.0;
      const bool currentInside = currentSlack >= 0.0;
      if (previousInside != currentInside)
      {
        const double fraction = previousSlack / (previousSlack - currentSlack);
        clipped.push_back(previous + fraction * (current - previous));
      }
      if (currentInside)
      {
        clipped.push_back(current);
      }
      previous = current;
      previousSlack = currentSlack;
    }
    vertices = std::move(clipped);
  }

  CgalPolygon2d result;
  for (const auto& vertex : vertices)
  {
    if (!vertex.allFinite())
    {
      return std::nullopt;
    }
    if (result.is_empty() ||
        (vertex - Eigen::Vector2d(result.vertices_end()[-1].x(), result.vertices_end()[-1].y())).norm() > 1e-10)
    {
      result.push_back(CgalPoint2d(vertex.x(), vertex.y()));
    }
  }
  if (result.size() < 3 || std::abs(result.area()) < 1e-12)
  {
    return std::nullopt;
  }
  return result;
}

class FootPlacementConstraintVisualizer final : public rclcpp::Node
{
public:
  FootPlacementConstraintVisualizer()
      : Node("foot_placement_constraint_visualizer")
  {
    terrainTopic_ = declare_parameter<std::string>(
        "terrain_topic", "/planar_terrain");
    markerTopic_ = declare_parameter<std::string>(
        "marker_topic", "/foot_placement_constraint_demo");
    numberOfVertices_ = declare_parameter<int>("num_vertices", 16);
    growthFactor_ = declare_parameter<double>("growth_factor", 1.05);
    boundaryMargin_ = declare_parameter<double>("boundary_margin", 0.05);
    lineWidth_ = declare_parameter<double>("line_width", 0.012);
    markerHeightOffset_ = declare_parameter<double>("marker_height_offset", 0.008);
    publishHalfSpaceNormals_ = declare_parameter<bool>("publish_halfspace_normals", true);
    nominalPoint_.x() = declare_parameter<double>("nominal_x", 0.0);
    nominalPoint_.y() = declare_parameter<double>("nominal_y", 0.0);
    nominalPoint_.z() = declare_parameter<double>("nominal_z", 0.4);

    if (numberOfVertices_ < 3)
    {
      throw std::invalid_argument("num_vertices must be at least 3");
    }
    if (!std::isfinite(boundaryMargin_) || boundaryMargin_ < 0.0)
    {
      throw std::invalid_argument("boundary_margin must be non-negative");
    }
    if (!std::isfinite(growthFactor_) || growthFactor_ <= 1.0 ||
        !nominalPoint_.allFinite() || !std::isfinite(lineWidth_) || lineWidth_ <= 0.0 ||
        !std::isfinite(markerHeightOffset_))
    {
      throw std::invalid_argument("Use finite parameters, growth_factor > 1, and line_width > 0");
    }

    rclcpp::QoS terrainQos(1);
    terrainQos.reliable();
    // The terrain demo publishes volatile data; requesting transient-local here
    // would prevent DDS from matching its publisher.
    terrainSubscription_ = create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
        terrainTopic_, terrainQos,
        [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr message)
        {
          terrain_ = convex_plane_decomposition::fromMessage(*message);
          terrainFrame_ = terrain_->gridMap.getFrameId();
          publishConstraint();
        });

    clickedPointSubscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr message)
        {
          if (!terrainFrame_.empty() && !message->header.frame_id.empty() &&
              message->header.frame_id != terrainFrame_)
          {
            RCLCPP_WARN(get_logger(),
                        "Ignoring /clicked_point in frame '%s'; terrain frame is '%s'.",
                        message->header.frame_id.c_str(), terrainFrame_.c_str());
            return;
          }
          const Eigen::Vector3d clicked(message->point.x, message->point.y, message->point.z);
          if (!clicked.allFinite())
          {
            RCLCPP_WARN(get_logger(), "Ignoring non-finite /clicked_point.");
            return;
          }
          nominalPoint_ = clicked;
          hasClickedPoint_ = true;
          publishConstraint();
        });

    markerPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        markerTopic_, rclcpp::QoS(1).reliable().transient_local());

    const double republishRate = declare_parameter<double>("republish_rate", 2.0);
    if (!std::isfinite(republishRate) || republishRate < 0.0 || republishRate > 100.0)
    {
      throw std::invalid_argument("republish_rate must be in [0, 100] Hz");
    }
    if (republishRate > 0.0)
    {
      const auto period = std::chrono::duration<double>(1.0 / republishRate);
      timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this]() { publishConstraint(); });
    }

    RCLCPP_INFO(get_logger(),
                "Waiting for PlanarTerrain on %s. Use RViz Publish Point (/clicked_point) to move the nominal foothold.",
                terrainTopic_.c_str());
  }

private:
  visualization_msgs::msg::Marker baseMarker(const std::string& markerNamespace, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = terrainFrame_.empty() ? "odom" : terrainFrame_;
    marker.header.stamp = now();
    marker.ns = markerNamespace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  Eigen::Vector3d pointInWorld(const CgalPoint2d& point, const Eigen::Isometry3d& planeToWorld,
                               double normalOffset = 0.0) const
  {
    return planeToWorld * Eigen::Vector3d(point.x(), point.y(), normalOffset);
  }

  visualization_msgs::msg::Marker polygonOutline(const CgalPolygon2d& polygon,
                                                  const Eigen::Isometry3d& planeToWorld,
                                                  const std::string& markerNamespace, int id,
                                                  const std_msgs::msg::ColorRGBA& markerColor,
                                                  double width, double heightOffset) const
  {
    auto marker = baseMarker(markerNamespace, id, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = width;
    marker.color = markerColor;
    if (polygon.is_empty())
    {
      return marker;
    }
    marker.points.reserve(polygon.size() + 1);
    for (const auto& point : polygon)
    {
      marker.points.push_back(pointMsg(pointInWorld(point, planeToWorld, heightOffset)));
    }
    marker.points.push_back(pointMsg(pointInWorld(polygon.vertex(0), planeToWorld, heightOffset)));
    return marker;
  }

  visualization_msgs::msg::Marker polygonFill(const CgalPolygon2d& polygon,
                                               const Eigen::Isometry3d& planeToWorld,
                                               const std_msgs::msg::ColorRGBA& markerColor,
                                               double heightOffset) const
  {
    auto marker = baseMarker("Actual constraint feasible set", 0,
                             visualization_msgs::msg::Marker::TRIANGLE_LIST);
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color = markerColor;
    if (polygon.size() < 3)
    {
      return marker;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& point : polygon)
    {
      centroid += Eigen::Vector2d(point.x(), point.y());
    }
    centroid /= static_cast<double>(polygon.size());
    const auto centroidWorld = pointMsg(
        planeToWorld * Eigen::Vector3d(centroid.x(), centroid.y(), heightOffset));

    marker.points.reserve(3 * polygon.size());
    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
      const std::size_t next = (i + 1) % polygon.size();
      marker.points.push_back(centroidWorld);
      marker.points.push_back(pointMsg(pointInWorld(polygon.vertex(i), planeToWorld, heightOffset)));
      marker.points.push_back(pointMsg(pointInWorld(polygon.vertex(next), planeToWorld, heightOffset)));
    }
    return marker;
  }

  void appendPointMarker(visualization_msgs::msg::MarkerArray& markers, const std::string& markerNamespace,
                         int id, const Eigen::Vector3d& point, double diameter,
                         const std_msgs::msg::ColorRGBA& markerColor) const
  {
    auto marker = baseMarker(markerNamespace, id, visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position = pointMsg(point);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color = markerColor;
    markers.markers.push_back(std::move(marker));
  }

  void appendProjectionLine(visualization_msgs::msg::MarkerArray& markers,
                            const Eigen::Vector3d& nominal, const Eigen::Vector3d& projected) const
  {
    auto marker = baseMarker("Projection", 0, visualization_msgs::msg::Marker::LINE_LIST);
    marker.scale.x = 0.006;
    marker.color = color(0.15F, 0.45F, 1.0F, 0.95F);
    marker.points.push_back(pointMsg(nominal));
    marker.points.push_back(pointMsg(projected));
    markers.markers.push_back(std::move(marker));
  }

  void appendHalfSpaceNormals(visualization_msgs::msg::MarkerArray& markers,
                              const CgalPolygon2d& polygon, const Eigen::Isometry3d& planeToWorld,
                              const Eigen::Vector2d& interiorPoint) const
  {
    constexpr double arrowLength = 0.08;
    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
      const std::size_t next = (i + 1) % polygon.size();
      const Eigen::Vector2d a(polygon.vertex(i).x(), polygon.vertex(i).y());
      const Eigen::Vector2d b(polygon.vertex(next).x(), polygon.vertex(next).y());
      const Eigen::Vector2d midpoint = 0.5 * (a + b);
      Eigen::Vector2d inward(-(b - a).y(), (b - a).x());
      if (inward.dot(interiorPoint - midpoint) < 0.0)
      {
        inward = -inward;
      }
      if (inward.norm() <= 1e-9)
      {
        continue;
      }
      inward.normalize();

      auto marker = baseMarker("Half-space inward normals", static_cast<int>(i),
                               visualization_msgs::msg::Marker::ARROW);
      marker.scale.x = 0.008;
      marker.scale.y = 0.016;
      marker.scale.z = 0.025;
      marker.color = color(0.10F, 0.80F, 0.30F, 0.95F);
      marker.points.push_back(pointMsg(
          planeToWorld * Eigen::Vector3d(midpoint.x(), midpoint.y(), 2.0 * markerHeightOffset_)));
      const Eigen::Vector2d arrowEnd = midpoint + arrowLength * inward;
      marker.points.push_back(pointMsg(
          planeToWorld * Eigen::Vector3d(arrowEnd.x(), arrowEnd.y(), 2.0 * markerHeightOffset_)));
      markers.markers.push_back(std::move(marker));
    }
  }

  void publishConstraint()
  {
    visualization_msgs::msg::MarkerArray markers;
    auto clearMarker = baseMarker("clear", 0, visualization_msgs::msg::Marker::CUBE);
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(std::move(clearMarker));
    if (!terrain_.has_value() || terrain_->planarRegions.empty())
    {
      markerPublisher_->publish(markers);
      return;
    }

    try
    {
      const auto zeroPenalty = [](const Eigen::Vector3d&) { return 0.0; };
      const auto projection = convex_plane_decomposition::getBestPlanarRegionAtPositionInWorld(
          nominalPoint_, terrain_->planarRegions, zeroPenalty);
      if (projection.regionPtr == nullptr)
      {
        markerPublisher_->publish(markers);
        return;
      }

      const auto rawPolygon = convex_plane_decomposition::growConvexPolygonInsideShape(
          projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame,
          numberOfVertices_, growthFactor_);

      const auto& planeToWorld = projection.regionPtr->transformPlaneToWorld;
      markers.markers.push_back(polygonOutline(
          rawPolygon, planeToWorld, "Selected convex region (before margin)", 0,
          color(1.0F, 0.72F, 0.10F, 1.0F), lineWidth_, markerHeightOffset_));

      const bool vertexCountMatches =
          rawPolygon.size() == static_cast<std::size_t>(numberOfVertices_);
      bool shrinkApplied = false;
      CgalPolygon2d constraintPolygon = rawPolygon;
      if (rawPolygon.size() >= 3 && vertexCountMatches)
      {
        const HalfSpacePolygon original = polygonToHalfSpaces(rawPolygon);
        HalfSpacePolygon shrunk;
        const Eigen::Vector2d interiorPoint(
            projection.positionInTerrainFrame.x(), projection.positionInTerrainFrame.y());
        shrinkApplied = shrinkHalfSpaces(original, interiorPoint, boundaryMargin_, shrunk);
        const HalfSpacePolygon& activeHalfSpaces = shrinkApplied ? shrunk : original;
        if (const auto reconstructed = halfSpacesToPolygon(activeHalfSpaces, rawPolygon); reconstructed.has_value())
        {
          constraintPolygon = *reconstructed;
        }
        else
        {
          throw std::runtime_error("Half-space intersection is empty or degenerate; not displaying a feasible set");
        }

        markers.markers.push_back(polygonFill(
            constraintPolygon, planeToWorld, color(0.10F, 0.85F, 0.30F, 0.24F),
            1.5 * markerHeightOffset_));
        markers.markers.push_back(polygonOutline(
            constraintPolygon, planeToWorld, "Actual constraint boundary", 0,
            color(0.05F, 1.0F, 0.25F, 1.0F), 1.5 * lineWidth_, 1.7 * markerHeightOffset_));

        if (publishHalfSpaceNormals_)
        {
          appendHalfSpaceNormals(markers, constraintPolygon, planeToWorld, interiorPoint);
        }
      }

      appendPointMarker(markers, "Nominal foothold", 0, nominalPoint_, 0.035,
                        color(0.15F, 0.45F, 1.0F, 1.0F));
      appendPointMarker(markers, "Projected foothold", 0,
                        projection.positionInWorld +
                            markerHeightOffset_ * planeToWorld.linear().col(2),
                        0.028, color(1.0F, 0.15F, 0.75F, 1.0F));
      appendProjectionLine(markers, nominalPoint_, projection.positionInWorld);

      markerPublisher_->publish(markers);

      if (hasClickedPoint_)
      {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "nominal=(%.3f, %.3f, %.3f), projection=(%.3f, %.3f, %.3f), vertices=%zu, shrink=%s",
            nominalPoint_.x(), nominalPoint_.y(), nominalPoint_.z(),
            projection.positionInWorld.x(), projection.positionInWorld.y(),
            projection.positionInWorld.z(), rawPolygon.size(), shrinkApplied ? "applied" : "fallback");
      }
    }
    catch (const std::exception& error)
    {
      markers.markers.resize(1);
      markerPublisher_->publish(markers);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Could not construct foot-placement constraint: %s", error.what());
    }
  }

  std::string terrainTopic_;
  std::string markerTopic_;
  std::string terrainFrame_;
  int numberOfVertices_{16};
  double growthFactor_{1.05};
  double boundaryMargin_{0.05};
  double lineWidth_{0.012};
  double markerHeightOffset_{0.008};
  bool publishHalfSpaceNormals_{true};
  bool hasClickedPoint_{false};
  Eigen::Vector3d nominalPoint_{0.0, 0.0, 0.4};
  std::optional<PlanarTerrain> terrain_;

  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr terrainSubscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSubscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootPlacementConstraintVisualizer>());
  rclcpp::shutdown();
  return 0;
}

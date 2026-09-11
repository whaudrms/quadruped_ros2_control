//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h"
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot
{
    namespace
    {
        // Clip all half-spaces: inset edges may become redundant, so simply
        // intersecting adjacent supporting lines does not give the feasible set.
        convex_plane_decomposition::CgalPolygon2d clipPolygon(
            const convex_plane_decomposition::CgalPolygon2d& original, const matrix_t& a, const vector_t& b)
        {
            std::vector<Eigen::Vector2d> vertices;
            for (const auto& p : original) vertices.emplace_back(p.x(), p.y());
            for (Eigen::Index row = 0; row < a.rows() && !vertices.empty(); ++row)
            {
                std::vector<Eigen::Vector2d> clipped;
                Eigen::Vector2d previous = vertices.back();
                scalar_t previousSlack = a.row(row).dot(previous) + b(row);
                for (const auto& current : vertices)
                {
                    const scalar_t slack = a.row(row).dot(current) + b(row);
                    if ((previousSlack >= 0.0) != (slack >= 0.0))
                        clipped.push_back(previous + previousSlack / (previousSlack - slack) * (current - previous));
                    if (slack >= 0.0) clipped.push_back(current);
                    previous = current;
                    previousSlack = slack;
                }
                vertices = std::move(clipped);
            }
            convex_plane_decomposition::CgalPolygon2d result;
            for (const auto& p : vertices) result.push_back({p.x(), p.y()});
            return result;
        }

        visualization_msgs::msg::Marker clearMarker(const std_msgs::msg::Header& header)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.action = visualization_msgs::msg::Marker::DELETEALL;
            return marker;
        }
    }

    FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector,
                                                           size_t numFoot,
                                                           const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                           scalar_t maxUpdateFrequency,
                                                           scalar_t boundaryMargin, bool placementEnabled)
        : convex_region_selector_(convexRegionSelector),
          num_foot_(numFoot),
          last_time_(std::numeric_limits<scalar_t>::lowest()),
          min_publish_time_difference_(1.0 / maxUpdateFrequency),
          boundary_margin_(boundaryMargin),
          placement_enabled_(placementEnabled)
    {
        marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("foot_placement", 1);
        swing_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/perceptive_reference/swing_trajectories", 1);
    }

    void FootPlacementVisualization::update(scalar_t initTime, scalar_t finalTime, const ModeSchedule& modeSchedule,
                                             const SwingTrajectoryPlanner& swingPlanner)
    {
        if (marker_publisher_->get_subscription_count() == 0 &&
            marker_publisher_->get_intra_process_subscription_count() == 0 &&
            swing_publisher_->get_subscription_count() == 0 &&
            swing_publisher_->get_intra_process_subscription_count() == 0)
        {
            return;
        }

        if (initTime < last_time_ || initTime - last_time_ > min_publish_time_difference_)
        {
            last_time_ = initTime;

            std_msgs::msg::Header header;
            //    header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
            header.frame_id = "odom";

            visualization_msgs::msg::MarkerArray makerArray;
            makerArray.markers.push_back(clearMarker(header));
            visualization_msgs::msg::MarkerArray swingArray;
            swingArray.markers.push_back(clearMarker(header));
            const auto contactFlags = convex_region_selector_.extractContactFlags(modeSchedule.modeSequence);
            const auto initialStanceEnds = convex_region_selector_.getInitStandFinalTimes();

            size_t i = 0;
            for (size_t leg = 0; leg < num_foot_; ++leg)
            {
                auto middleTimes = convex_region_selector_.getMiddleTimes(leg);
                // Keep the current stance visible after its selection time.
                middleTimes.erase(std::remove_if(middleTimes.begin(), middleTimes.end(),
                    [initTime, finalTime](scalar_t t) { return t <= initTime || t > finalTime; }), middleTimes.end());
                if (convex_region_selector_.getProjection(leg, initTime).regionPtr != nullptr)
                    middleTimes.insert(middleTimes.begin(), initTime);

                for (size_t k = 0; k < middleTimes.size(); ++k)
                {
                    const auto projection = convex_region_selector_.getProjection(leg, middleTimes[k]);
                    if (projection.regionPtr == nullptr)
                    {
                        continue;
                    }
                    auto color = feet_color_map_[leg];
                    float alpha = 1 - 0.6f * static_cast<float>(k) / static_cast<float>(middleTimes.size());
                    // Projections
                    auto projectionMaker = getArrowAtPointMsg(
                        projection.regionPtr->transformPlaneToWorld.linear() * vector3_t(0, 0, 0.1),
                        projection.positionInWorld, color);
                    projectionMaker.header = header;
                    projectionMaker.ns = "Projections";
                    projectionMaker.id = i;
                    projectionMaker.color.a = alpha;
                    makerArray.markers.push_back(projectionMaker);

                    // Convex Region
                    const auto convexRegion = convex_region_selector_.getConvexPolygon(leg, middleTimes[k]);
                    auto rawMarker = to3dRosMarker(convexRegion,
                                                               projection.regionPtr->transformPlaneToWorld, header,
                                                               color, 0.35f * alpha, i);
                    rawMarker.ns = "Raw Convex Regions";
                    makerArray.markers.push_back(rawMarker);

                    if (placement_enabled_ && middleTimes[k] >= initialStanceEnds[leg] &&
                        convexRegion.size() >= 3 && convexRegion.size() == convex_region_selector_.getNumVertices())
                    {
                        auto [a, b] = PerceptiveLeggedPrecomputation::getPolygonConstraint(convexRegion);
                        matrix_t activeA;
                        vector_t activeB;
                        const Eigen::Vector2d seed(projection.positionInTerrainFrame.x(), projection.positionInTerrainFrame.y());
                        const bool marginAccepted = PerceptiveLeggedPrecomputation::tryShrinkPolygonConstraint(
                            a, b, seed, activeA, activeB, boundary_margin_);
                        const auto activePolygon = marginAccepted ? clipPolygon(convexRegion, activeA, activeB) : convexRegion;
                        if (activePolygon.size() >= 3)
                        {
                            auto activeMarker = to3dRosMarker(activePolygon,
                                projection.regionPtr->transformPlaneToWorld, header, color, alpha, i);
                            activeMarker.ns = marginAccepted ? "Foot Placement Feasible Regions" : "Foot Placement Margin Fallback";
                            activeMarker.scale.x = 0.012;
                            makerArray.markers.push_back(activeMarker);
                        }
                    }

                    // Nominal Footholds
                    const auto nominal = convex_region_selector_.getNominalFootholds(leg, middleTimes[k]);
                    auto nominalMarker = getFootMarker(nominal, true, color, foot_marker_diameter_, 1.);
                    nominalMarker.header = header;
                    nominalMarker.ns = "Nominal Footholds";
                    nominalMarker.id = i;
                    nominalMarker.color.a = alpha;
                    makerArray.markers.push_back(nominalMarker);

                    i++;
                }

                // The planner constrains z(t), not a full xyz swing curve.
                // xy is interpolated between adjacent stance projections ONLY
                // to place the actual height reference in the RViz scene.
                for (size_t phase = 1; phase + 1 < modeSchedule.modeSequence.size(); ++phase)
                {
                    if (contactFlags[leg][phase] || !contactFlags[leg][phase - 1]) continue;
                    size_t endPhase = phase;
                    while (endPhase + 1 < contactFlags[leg].size() && !contactFlags[leg][endPhase + 1]) ++endPhase;
                    if (endPhase + 1 >= contactFlags[leg].size()) break;
                    const scalar_t liftOff = modeSchedule.eventTimes[phase - 1];
                    const scalar_t touchDown = modeSchedule.eventTimes[endPhase];
                    if (touchDown <= initTime || liftOff >= finalTime || touchDown <= liftOff) continue;
                    const auto from = convex_region_selector_.getProjection(leg, liftOff - 1e-7);
                    const auto to = convex_region_selector_.getProjection(leg, touchDown + 1e-7);
                    if (!from.regionPtr || !to.regionPtr) continue;
                    visualization_msgs::msg::Marker marker;
                    marker.header = header;
                    marker.ns = "Swing Z Reference (XY interpolated)";
                    marker.id = static_cast<int>(leg * modeSchedule.modeSequence.size() + phase);
                    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.009;
                    marker.color = getColor(feet_color_map_[leg], 1.0);
                    const scalar_t start = std::max(initTime, liftOff);
                    const scalar_t end = std::min(finalTime, touchDown);
                    for (size_t sample = 0; sample <= 40; ++sample)
                    {
                        const scalar_t t = start + (end - start) * sample / 40.0;
                        const scalar_t fraction = (t - liftOff) / (touchDown - liftOff);
                        const vector3_t xy = (1.0 - fraction) * from.positionInWorld + fraction * to.positionInWorld;
                        geometry_msgs::msg::Point point;
                        point.x = xy.x();
                        point.y = xy.y();
                        // Query inside the swing at both boundaries; an exact
                        // event timestamp can select the stance-side spline.
                        const scalar_t epsilon = std::min<scalar_t>(1e-7, 0.01 * (touchDown - liftOff));
                        point.z = swingPlanner.getZpositionConstraint(leg, std::clamp(t, liftOff + epsilon, touchDown - epsilon));
                        if (const auto* swing = swingPlanner.getTerrainSwing(leg, t)) {
                            const vector3_t position = swing->spline.position(t);
                            point.x = position.x(); point.y = position.y(); point.z = position.z();
                        }
                        marker.points.push_back(point);
                    }
                    swingArray.markers.push_back(std::move(marker));
                    phase = endPhase;
                }
            }

            marker_publisher_->publish(makerArray);
            swing_publisher_->publish(swingArray);
        }
    }

    visualization_msgs::msg::Marker FootPlacementVisualization::to3dRosMarker(
        const convex_plane_decomposition::CgalPolygon2d& polygon,
        const Eigen::Isometry3d& transformPlaneToWorld,
        const std_msgs::msg::Header& header, Color color, float alpha, size_t i) const
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = "Convex Regions";
        marker.id = i;
        marker.header = header;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = line_width_;
        marker.color = getColor(color, alpha);
        if (!polygon.is_empty())
        {
            marker.points.reserve(polygon.size() + 1);
            for (const auto& point : polygon)
            {
                const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    point, transformPlaneToWorld);
                geometry_msgs::msg::Point point_ros;
                point_ros.x = pointInWorld.x();
                point_ros.y = pointInWorld.y();
                point_ros.z = pointInWorld.z();
                marker.points.push_back(point_ros);
            }
            // repeat the first point to close to polygon
            const auto pointInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    polygon.vertex(0), transformPlaneToWorld);
            geometry_msgs::msg::Point point_ros;
            point_ros.x = pointInWorld.x();
            point_ros.y = pointInWorld.y();
            point_ros.z = pointInWorld.z();
            marker.points.push_back(point_ros);
        }
        marker.pose.orientation.w = 1.0;
        return marker;
    }
} // namespace legged

// Headless test of the actual controller visualizer; no simulator or robot IO.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h"

using namespace ocs2;
using namespace ocs2::legged_robot;
using namespace convex_plane_decomposition;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

class FixedFeet final : public EndEffectorKinematics<scalar_t>
{
public:
    FixedFeet* clone() const override { return new FixedFeet(*this); }
    const std::vector<std::string>& getIds() const override { return ids_; }
    std::vector<vector3_t> getPosition(const vector_t&) const override
    { return {{0.2, 0.2, 0.0}, {0.2, -0.2, 0.0}, {-0.2, 0.2, 0.0}, {-0.2, -0.2, 0.0}}; }
    std::vector<vector3_t> getVelocity(const vector_t&, const vector_t&) const override { return {}; }
    std::vector<vector3_t> getOrientationError(const vector_t&, const std::vector<quaternion_t>&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t&, const vector_t&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
        const vector_t&, const std::vector<quaternion_t>&) const override { return {}; }
private:
    std::vector<std::string> ids_{"FL", "FR", "RL", "RR"};
};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perceptive_visualization_test");
        auto listener = rclcpp::Node::make_shared("perceptive_visualization_listener");
        MarkerArray::SharedPtr feetMessage, swingMessage;
        auto feetSub = listener->create_subscription<MarkerArray>("/foot_placement", 1,
            [&](MarkerArray::SharedPtr msg) { feetMessage = std::move(msg); });
        auto swingSub = listener->create_subscription<MarkerArray>("/perceptive_reference/swing_trajectories", 1,
            [&](MarkerArray::SharedPtr msg) { swingMessage = std::move(msg); });
        CentroidalModelInfo info{};
        info.numThreeDofContacts = 4;
        info.generalizedCoordinatesNum = 18;
        info.actuatedDofNum = 12;
        info.stateDim = 24;
        info.inputDim = 24;
        auto terrain = std::make_shared<PlanarTerrain>();
        PlanarRegion region;
        region.transformPlaneToWorld = Eigen::Isometry3d::Identity();
        region.transformPlaneToWorld.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
        region.transformPlaneToWorld.translation() = vector3_t(0.1, -0.1, 0.2);
        CgalPolygon2d square;
        for (const auto& p : std::vector<CgalPoint2d>{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) square.push_back(p);
        region.boundaryWithInset.boundary = CgalPolygonWithHoles2d(square);
        region.boundaryWithInset.insets.push_back(CgalPolygonWithHoles2d(square));
        region.bbox2d = square.bbox();
        terrain->planarRegions.push_back(region);
        FixedFeet kinematics;
        ConvexRegionSelector selector(info, terrain, std::make_shared<std::mutex>(), kinematics, 16);
        SwingTrajectoryPlanner planner({}, 4);
        ModeSchedule schedule;
        schedule.eventTimes = {-1.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 2.0};
        schedule.modeSequence = {15, 15, 9, 15, 6, 15, 9, 15, 15};
        const vector_t state = vector_t::Zero(24);
        TargetTrajectories targets({-1.0, 3.0}, {state, state}, {state, state});
        selector.update(schedule, 0.05, state, targets);
        planner.update(schedule, 0.2);
        FootPlacementVisualization visualizer(selector, 4, node);
        FootPlacementVisualization disabled(selector, 4, node, 20.0, 0.05, false);
        FootPlacementVisualization fallback(selector, 4, node, 20.0, 10.0, true);
        node->configure();
        node->activate();

        auto receive = [&](FootPlacementVisualization& publisher, scalar_t time)
        {
            feetMessage.reset();
            swingMessage.reset();
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while ((!feetMessage || !swingMessage) && std::chrono::steady_clock::now() < deadline)
            {
                publisher.update(time, 1.1, schedule, planner);
                rclcpp::spin_some(listener);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            require(feetMessage && swingMessage, "No visualization messages");
            require(feetMessage->markers.front().action == Marker::DELETEALL, "Old polygons not cleared");
            require(swingMessage->markers.front().action == Marker::DELETEALL, "Old swings not cleared");
            for (const auto& marker : feetMessage->markers)
                require(marker.type != Marker::TEXT_VIEW_FACING, "Unexpected text marker");
        };
        receive(visualizer, 0.05);
        size_t feasibleCount = 0;
        for (const auto& marker : feetMessage->markers)
        {
            if (marker.ns != "Foot Placement Feasible Regions") continue;
            ++feasibleCount;
            const auto raw = std::find_if(feetMessage->markers.begin(), feetMessage->markers.end(),
                [&](const Marker& m) { return m.ns == "Raw Convex Regions" && m.id == marker.id; });
            require(raw != feetMessage->markers.end(), "Missing raw polygon");
            CgalPolygon2d rawPolygon;
            for (size_t i = 0; i + 1 < raw->points.size(); ++i)
            {
                const auto& p = raw->points[i];
                const vector3_t local = region.transformPlaneToWorld.inverse() * vector3_t(p.x, p.y, p.z);
                rawPolygon.push_back({local.x(), local.y()});
            }
            auto [a, b] = PerceptiveLeggedPrecomputation::getPolygonConstraint(rawPolygon);
            for (Eigen::Index row = 0; row < a.rows(); ++row) b(row) -= 0.05 * a.row(row).norm();
            for (const auto& p : marker.points)
            {
                const vector3_t local = region.transformPlaneToWorld.inverse() * vector3_t(p.x, p.y, p.z);
                require(std::abs(local.z()) < 1e-9, "Polygon is not on terrain plane");
                require((a * local.head<2>() + b).minCoeff() > -1e-8, "Displayed vertex violates MPC margin");
            }
        }
        require(feasibleCount > 0, "No feasible polygons");
        require(swingMessage->markers.size() > 1, "No swing reference curves");
        for (size_t i = 1; i < swingMessage->markers.size(); ++i)
        {
            const auto& marker = swingMessage->markers[i];
            const size_t leg = marker.id / schedule.modeSequence.size();
            const size_t phase = marker.id % schedule.modeSequence.size();
            const scalar_t start = schedule.eventTimes[phase - 1];
            const scalar_t end = schedule.eventTimes[phase];
            require(marker.points.size() == 41, "Wrong swing sampling");
            for (size_t j = 0; j < marker.points.size(); ++j)
            {
                const scalar_t visibleStart = std::max<scalar_t>(0.05, start);
                const scalar_t t = std::clamp(visibleStart + (end - visibleStart) * j / 40.0, start + 1e-7, end - 1e-7);
                require(std::abs(marker.points[j].z - planner.getZpositionConstraint(leg, t)) < 1e-10,
                        "Swing marker differs from planner reference");
            }
        }

        receive(disabled, 0.05);
        for (const auto& marker : feetMessage->markers)
            require(marker.ns.find("Foot Placement") == std::string::npos, "Disabled constraint displayed as active");
        receive(fallback, 0.05);
        require(std::any_of(feetMessage->markers.begin(), feetMessage->markers.end(),
            [](const Marker& m) { return m.ns == "Foot Placement Margin Fallback"; }), "Missing margin fallback");
        schedule.modeSequence.assign(schedule.modeSequence.size(), 15);
        selector.update(schedule, 0.5, state, targets);
        planner.update(schedule, 0.2);
        receive(visualizer, 0.5);
        require(swingMessage->markers.size() == 1, "Stance replan retains old swing curves");
        std::cout << "PASS: sloped-plane margin, swing z reference, disabled constraint, margin fallback, stale-marker cleanup\n";
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}

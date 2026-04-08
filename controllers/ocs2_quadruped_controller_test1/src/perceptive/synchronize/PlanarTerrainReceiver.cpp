//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        size_t gPlanarTerrainDebugCounter = 0;
    }

    PlanarTerrainReceiver::PlanarTerrainReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                 const std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                                 & planarTerrainPtr,
                                                 const std::shared_ptr<grid_map::SignedDistanceField>&
                                                 signedDistanceFieldPtr,
                                                 const std::string& mapTopic, const std::string& sdfElevationLayer)
        : node_(node),
          planarTerrainPtr_(planarTerrainPtr),
          sdfPtr_(signedDistanceFieldPtr),
          sdfElevationLayer_(sdfElevationLayer)
    {
        subscription_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
            mapTopic, 10, [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain msg)
            {
                std::lock_guard lock(mutex_);
                updated_ = true;

                planarTerrain_ = convex_plane_decomposition::PlanarTerrain(
                    convex_plane_decomposition::fromMessage(msg));

                auto& elevationData = planarTerrain_.gridMap.get(sdfElevationLayer_);
                if (elevationData.hasNaN())
                {
                    const float inpaint{elevationData.minCoeffOfFinites()};
                    RCLCPP_WARN(node_->get_logger(),
                                "[PlanarTerrainReceiver] Map contains NaN values. Will apply inpainting with min value.")
                    ;
                    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
                }

                if ((gPlanarTerrainDebugCounter++ % 20) == 0)
                {
                    const auto& gridMap = planarTerrain_.gridMap;
                    const auto position = gridMap.getPosition();
                    const auto length = gridMap.getLength();
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[PlanarTerrainReceiverDebug] frame=%s regions=%zu layers=%zu map_center=(%.3f, %.3f) map_length=(%.3f, %.3f)",
                        gridMap.getFrameId().c_str(),
                        planarTerrain_.planarRegions.size(),
                        gridMap.getLayers().size(),
                        position.x(),
                        position.y(),
                        length.x(),
                        length.y());
                }
            });
    }

    void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/,
                                             const vector_t& /*currentState*/,
                                             const ReferenceManagerInterface& /*referenceManager*/)
    {
        if (updated_)
        {
            std::lock_guard lock(mutex_);
            updated_ = false;
            *planarTerrainPtr_ = planarTerrain_;

            const auto& elevationData = planarTerrainPtr_->gridMap.get(sdfElevationLayer_);
            constexpr float heightMargin{0.1};
            const float maxValue{elevationData.maxCoeffOfFinites() + 3 * heightMargin};
            sdfPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, sdfElevationLayer_, maxValue);
        }
    }
} // namespace legged

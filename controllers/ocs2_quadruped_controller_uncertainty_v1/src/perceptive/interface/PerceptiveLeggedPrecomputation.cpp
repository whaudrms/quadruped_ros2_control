//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

#include <cstdio>

namespace ocs2::legged_robot
{
    namespace
    {
        constexpr scalar_t kContactMarginActivationDelay = 2.0;
        constexpr scalar_t kUncertaintyContactMarginScale = 0.004;
        constexpr scalar_t kHighUncertaintyThreshold = 0.05;
        constexpr scalar_t kMetricsLogPeriod = 0.5;
        constexpr const char* kMetricsFilePath = "/tmp/uncertainty_contact_metrics.log";
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface,
                                                                   const CentroidalModelInfo& info,
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                   ModelSettings settings,
                                                                   const ConvexRegionSelector& convexRegionSelector)
        : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
          convexRegionSelectorPtr_(&convexRegionSelector)
    {
        footPlacementConParameters_.resize(info.numThreeDofContacts);
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
        : LeggedRobotPreComputation(rhs), convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_)
    {
        footPlacementConParameters_ = rhs.footPlacementConParameters_;
    }

    void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
    {
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
        {
            return;
        }
        LeggedRobotPreComputation::request(request, t, x, u);

        if (request.contains(Request::Constraint) || request.contains(Request::SoftConstraint))
        {
            size_t totalContacts = 0;
            size_t activeMarginContacts = 0;
            size_t highUncertaintyContacts = 0;
            size_t totalLegChecks = footPlacementConParameters_.size();
            size_t timingWindowLegs = 0;
            scalar_t uncertaintySum = 0.0;
            scalar_t uncertaintyMax = 0.0;
            for (size_t i = 0; i < footPlacementConParameters_.size(); i++)
            {
                if (getSwingTrajectoryPlanner().isInContactTimingUncertaintyWindow(i, t))
                {
                    timingWindowLegs++;
                }

                FootPlacementConstraint::Parameter params;
                params.a = matrix_t::Zero(0, 3);
                params.b = vector_t::Zero(0);

                auto projection = convexRegionSelectorPtr_->getProjection(i, t);
                if (projection.regionPtr == nullptr)
                {
                    // No valid terrain projection is available for this foot yet.
                    // Keep an empty parameter block and let the constraint fall back to a no-op.
                    footPlacementConParameters_[i] = params;
                    continue;
                }

                matrix_t polytopeA;
                vector_t polytopeB;
                std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegionSelectorPtr_->getConvexPolygon(i, t));

                totalContacts++;
                const scalar_t uncertainty =
                    convexRegionSelectorPtr_->getUncertaintyAtPosition(projection.positionInWorld);
                uncertaintySum += uncertainty;
                uncertaintyMax = std::max(uncertaintyMax, uncertainty);
                if (uncertainty >= kHighUncertaintyThreshold)
                {
                    highUncertaintyContacts++;
                }

                scalar_t margin = 0.0;
                if (t > kContactMarginActivationDelay)
                {
                    margin += kUncertaintyContactMarginScale * std::max<scalar_t>(0.0, uncertainty);

                    if (margin > 0.0)
                    {
                        activeMarginContacts++;
                        for (int row = 0; row < polytopeA.rows(); ++row)
                        {
                            polytopeB(row) -= margin * polytopeA.row(row).norm();
                        }
                    }
                }

                matrix_t p = (matrix_t(2, 3) << // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
                params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
                params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().
                                                              head(2);

                footPlacementConParameters_[i] = params;
            }

            static scalar_t lastMetricsLogTime = -1.0;
            if (t - lastMetricsLogTime >= kMetricsLogPeriod)
            {
                if (FILE* metricsFile = std::fopen(kMetricsFilePath, "a"))
                {
                    std::fprintf(
                        metricsFile,
                        "[UncertaintyContactMetrics] t=%.3f total_contacts=%zu total_leg_checks=%zu active_margin_contacts=%zu high_uncertainty_contacts=%zu timing_window_contacts=%zu uncertainty_mean=%.6f uncertainty_max=%.6f\n",
                        static_cast<double>(t), totalContacts, totalLegChecks, activeMarginContacts, highUncertaintyContacts,
                        timingWindowLegs,
                        totalContacts > 0 ? static_cast<double>(uncertaintySum / static_cast<scalar_t>(totalContacts)) : 0.0,
                        static_cast<double>(uncertaintyMax));
                    std::fclose(metricsFile);
                }
                lastMetricsLogTime = t;
            }
        }
    }

    std::pair<matrix_t, vector_t> PerceptiveLeggedPrecomputation::getPolygonConstraint(
        const convex_plane_decomposition::CgalPolygon2d& polygon) const
    {
        size_t numVertices = polygon.size();
        matrix_t polytopeA = matrix_t::Zero(numVertices, 2);
        vector_t polytopeB = vector_t::Zero(numVertices);

        for (size_t i = 0; i < numVertices; i++)
        {
            size_t j = i + 1;
            if (j == numVertices)
            {
                j = 0;
            }
            size_t k = j + 1;
            if (k == numVertices)
            {
                k = 0;
            }
            const auto point_a = polygon.vertex(i);
            const auto point_b = polygon.vertex(j);
            const auto point_c = polygon.vertex(k);

            polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
            polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
            if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0)
            {
                polytopeA.row(i) *= -1;
                polytopeB(i) *= -1;
            }
        }

        return {polytopeA, polytopeB};
    }
} // namespace legged

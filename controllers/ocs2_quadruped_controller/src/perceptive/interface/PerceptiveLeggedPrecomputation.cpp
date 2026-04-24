//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

#include <cmath>

namespace ocs2::legged_robot
{
    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface,
                                                                   const CentroidalModelInfo& info,
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                   ModelSettings settings,
                                                                   const ConvexRegionSelector& convexRegionSelector,
                                                                   scalar_t footPlacementBoundaryMargin)
        : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
          convexRegionSelectorPtr_(&convexRegionSelector),
          numVertices_(convexRegionSelector.getNumVertices()),
          footPlacementBoundaryMargin_(footPlacementBoundaryMargin)
    {
        footPlacementConParameters_.resize(info.numThreeDofContacts);
        for (auto& parameter : footPlacementConParameters_)
        {
            parameter = makeSafeFootPlacementConstraintParameter();
        }
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
        : LeggedRobotPreComputation(rhs),
          convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_),
          numVertices_(rhs.numVertices_),
          footPlacementBoundaryMargin_(rhs.footPlacementBoundaryMargin_),
          footPlacementConParameters_(rhs.footPlacementConParameters_)
    {
    }

    void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
    {
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
        {
            return;
        }
        LeggedRobotPreComputation::request(request, t, x, u);

        if (request.containsAny(Request::Constraint + Request::SoftConstraint))
        {
            for (size_t i = 0; i < footPlacementConParameters_.size(); i++)
            {
                auto params = makeSafeFootPlacementConstraintParameter();

                auto projection = convexRegionSelectorPtr_->getProjection(i, t);
                if (projection.regionPtr == nullptr)
                {
                    // Swing leg
                    footPlacementConParameters_[i] = params;
                    continue;
                }

                const auto convexPolygon = convexRegionSelectorPtr_->getConvexPolygon(i, t);
                if (convexPolygon.size() < 3)
                {
                    footPlacementConParameters_[i] = params;
                    continue;
                }

                matrix_t polytopeA;
                vector_t polytopeB;
                std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexPolygon);
                if (polytopeA.rows() != static_cast<Eigen::Index>(numVertices_))
                {
                    footPlacementConParameters_[i] = params;
                    continue;
                }

                matrix_t activePolytopeA = polytopeA;
                vector_t activePolytopeB = polytopeB;
                const Eigen::Matrix<scalar_t, 2, 1> interiorPoint(
                    projection.positionInTerrainFrame.x(), projection.positionInTerrainFrame.y());
                if (!tryShrinkPolygonConstraint(polytopeA, polytopeB, interiorPoint, activePolytopeA, activePolytopeB))
                {
                    activePolytopeA = polytopeA;
                    activePolytopeB = polytopeB;
                }

                matrix_t p = (matrix_t(2, 3) << // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
                params.a = activePolytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
                params.b = activePolytopeB + activePolytopeA *
                                              projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);

                footPlacementConParameters_[i] = params;
            }
        }
    }

    FootPlacementConstraint::Parameter PerceptiveLeggedPrecomputation::makeSafeFootPlacementConstraintParameter() const
    {
        FootPlacementConstraint::Parameter parameter;
        parameter.a = matrix_t::Zero(numVertices_, 3);
        parameter.b = vector_t::Ones(numVertices_);
        return parameter;
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

    bool PerceptiveLeggedPrecomputation::tryShrinkPolygonConstraint(const matrix_t& polytopeA, const vector_t& polytopeB,
                                                                    const Eigen::Matrix<scalar_t, 2, 1>& interiorPoint, matrix_t& shrunkA,
                                                                    vector_t& shrunkB) const
    {
        shrunkA = polytopeA;
        shrunkB = polytopeB;

        if (footPlacementBoundaryMargin_ <= 0.0)
        {
            return true;
        }

        for (Eigen::Index row = 0; row < polytopeA.rows(); ++row)
        {
            const scalar_t normalNorm = polytopeA.row(row).norm();
            if (normalNorm <= 1e-9)
            {
                return false;
            }
            shrunkB(row) -= footPlacementBoundaryMargin_ * normalNorm;
        }

        const vector_t slack = shrunkA * interiorPoint + shrunkB;
        return (slack.array() > 1e-6).all();
    }
} // namespace legged

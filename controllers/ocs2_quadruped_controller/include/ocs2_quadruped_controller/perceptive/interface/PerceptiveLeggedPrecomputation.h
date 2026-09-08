//
// Created by biao on 3/21/25.
//

#pragma once

#include <utility>

#include <ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"

namespace ocs2::legged_robot
{
    /** Callback for caching and reference update */
    class PerceptiveLeggedPrecomputation : public LeggedRobotPreComputation
    {
    public:
        PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                       const ConvexRegionSelector& convexRegionSelector,
                                       scalar_t footPlacementBoundaryMargin);
        ~PerceptiveLeggedPrecomputation() override = default;

        PerceptiveLeggedPrecomputation* clone() const override { return new PerceptiveLeggedPrecomputation(*this); }

        void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

        const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const
        {
            return footPlacementConParameters_;
        }

        PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs);

        // Shared with visualization so the displayed admissible set uses the
        // exact MPC half-spaces and the same margin fallback rule.
        static std::pair<matrix_t, vector_t> getPolygonConstraint(
            const convex_plane_decomposition::CgalPolygon2d& polygon);
        static bool tryShrinkPolygonConstraint(const matrix_t& polytopeA, const vector_t& polytopeB,
                                        const Eigen::Matrix<scalar_t, 2, 1>& interiorPoint,
                                        matrix_t& shrunkA, vector_t& shrunkB, scalar_t boundaryMargin);

    private:
        FootPlacementConstraint::Parameter makeSafeFootPlacementConstraintParameter() const;

        const ConvexRegionSelector* convexRegionSelectorPtr_;
        size_t numVertices_;
        scalar_t footPlacementBoundaryMargin_{0.05};

        std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
    };
} // namespace ocs2::legged_robot

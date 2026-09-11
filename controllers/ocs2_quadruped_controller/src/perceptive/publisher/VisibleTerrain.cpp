#include "ocs2_quadruped_controller/perceptive/interface/VisibleTerrain.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/convex_hull_2.h>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace ocs2::legged_robot {
namespace {
using namespace convex_plane_decomposition;
using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point = ExactKernel::Point_2;
using Polygon = CGAL::Polygon_2<ExactKernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<ExactKernel>;
using PolygonSet = CGAL::Polygon_set_2<ExactKernel>;

Polygon exactPolygon(const CgalPolygon2d& input) {
  Polygon result;
  for (const auto& p : input) result.push_back(Point(p.x(), p.y()));
  return result;
}

PolygonWithHoles exactPolygon(const CgalPolygonWithHoles2d& input) {
  std::vector<Polygon> holes;
  for (const auto& h : input.holes()) holes.push_back(exactPolygon(h));
  return PolygonWithHoles(exactPolygon(input.outer_boundary()), holes.begin(), holes.end());
}

CgalPolygon2d inexactPolygon(const Polygon& input) {
  CgalPolygon2d result;
  for (const auto& p : input) result.push_back(CgalPoint2d(CGAL::to_double(p.x()), CGAL::to_double(p.y())));
  return result;
}

CgalPolygonWithHoles2d inexactPolygon(const PolygonWithHoles& input) {
  std::vector<CgalPolygon2d> holes;
  for (const auto& h : input.holes()) holes.push_back(inexactPolygon(h));
  return CgalPolygonWithHoles2d(inexactPolygon(input.outer_boundary()), holes.begin(), holes.end());
}

double planeHeight(const PlanarRegion& region, double x, double y) {
  const auto normal = region.transformPlaneToWorld.linear().col(2).eval();
  const auto origin = region.transformPlaneToWorld.translation().eval();
  if (std::abs(normal.z()) < 1e-6) throw std::runtime_error("Vertical plane cannot be a foothold surface");
  return origin.z() - (normal.x() * (x - origin.x()) + normal.y() * (y - origin.y())) / normal.z();
}

// Original static surfaces are convex rectangles. Clip the occluder's footprint
// by the linear height difference so partially crossing ramps also work.
Polygon occlusion(const PlanarRegion& lower, const PlanarRegion& upper) {
  struct Vertex { Eigen::Vector2d xy; double above; };
  std::vector<Vertex> vertices;
  for (const auto& p : upper.boundaryWithInset.boundary.outer_boundary()) {
    const Eigen::Vector3d world = upper.transformPlaneToWorld * Eigen::Vector3d(p.x(), p.y(), 0.0);
    vertices.push_back({world.head<2>(), world.z() - planeHeight(lower, world.x(), world.y()) - 1e-8});
  }
  std::vector<Eigen::Vector2d> clipped;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& a = vertices[i];
    const auto& b = vertices[(i + 1) % vertices.size()];
    if (a.above > 0.0) clipped.push_back(a.xy);
    if ((a.above > 0.0) != (b.above > 0.0))
      clipped.push_back(a.xy + (a.above / (a.above - b.above)) * (b.xy - a.xy));
  }
  Polygon result;
  for (const auto& xy : clipped) {
    const Eigen::Vector3d local = lower.transformPlaneToWorld.inverse() *
        Eigen::Vector3d(xy.x(), xy.y(), planeHeight(lower, xy.x(), xy.y()));
    result.push_back(Point(local.x(), local.y()));
  }
  if (result.size() >= 3 && result.is_clockwise_oriented()) result.reverse_orientation();
  return result;
}

// Conservative square expansion in the lower plane's coordinates. Keeping the
// inset away from a new hole prevents projections exactly on the riser edge.
Polygon expanded(const Polygon& polygon, double margin) {
  std::vector<Point> points;
  for (const auto& p : polygon)
    for (double dx : {-margin, margin})
      for (double dy : {-margin, margin}) points.emplace_back(p.x() + dx, p.y() + dy);
  Polygon result;
  CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(result));
  return result;
}
}  // namespace

std::vector<PlanarRegion> visibleTerrainRegions(const std::vector<PlanarRegion>& regions, double edgeMargin) {
  if (!std::isfinite(edgeMargin) || edgeMargin < 0.0) throw std::invalid_argument("Invalid terrain edge margin");
  std::vector<PlanarRegion> result;
  for (size_t i = 0; i < regions.size(); ++i) {
    const auto& region = regions[i];
    PolygonSet boundary(exactPolygon(region.boundaryWithInset.boundary));
    PolygonSet inset;
    for (const auto& p : region.boundaryWithInset.insets) inset.join(exactPolygon(p));
    for (size_t j = 0; j < regions.size(); ++j) {
      if (i == j) continue;
      const auto mask = occlusion(region, regions[j]);
      if (mask.size() < 3 || mask.area() == 0) continue;
      boundary.difference(mask);
      inset.difference(expanded(mask, edgeMargin));
    }
    std::vector<PolygonWithHoles> components;
    boundary.polygons_with_holes(std::back_inserter(components));
    for (const auto& component : components) {
      PolygonSet componentInset = inset;
      componentInset.intersection(component);
      std::vector<PolygonWithHoles> insetComponents;
      componentInset.polygons_with_holes(std::back_inserter(insetComponents));
      if (insetComponents.empty()) continue;  // No room for a safe foothold.
      PlanarRegion visible = region;
      visible.boundaryWithInset.boundary = inexactPolygon(component);
      visible.bbox2d = visible.boundaryWithInset.boundary.outer_boundary().bbox();
      visible.boundaryWithInset.insets.clear();
      for (const auto& part : insetComponents) visible.boundaryWithInset.insets.push_back(inexactPolygon(part));
      result.push_back(std::move(visible));
    }
  }
  return result;
}

bool projectionMatchesElevation(const PlanarTerrain& terrain, const PlanarTerrainProjection& projection,
                                double heightTolerance) {
  if (!projection.regionPtr || !projection.positionInWorld.allFinite() || !terrain.gridMap.exists("elevation"))
    return false;
  grid_map::Index index;
  if (!terrain.gridMap.getIndex(projection.positionInWorld.head<2>(), index)) return false;
  const double height = terrain.gridMap.at("elevation", index);
  grid_map::Position cell;
  if (!std::isfinite(height) || !terrain.gridMap.getPosition(index, cell)) return false;
  return std::abs(height - planeHeight(*projection.regionPtr, cell.x(), cell.y())) <= heightTolerance;
}

PlanarTerrainProjection selectVisibleTerrainProjection(const PlanarTerrain& terrain, const Eigen::Vector3d& query) {
  PlanarTerrainProjection best;
  best.cost = std::numeric_limits<double>::infinity();
  const auto sorted = sortWithBoundingBoxes(query, terrain.planarRegions);
  for (const auto& candidate : sorted) {
    if (candidate.boundingBoxSquareDistance > best.cost) continue;
    const auto& region = *candidate.regionPtr;
    if (region.boundaryWithInset.insets.empty()) continue;
    PlanarTerrainProjection p;
    p.regionPtr = &region;
    p.positionInTerrainFrame = projectToPlanarRegion(candidate.positionInTerrainFrame, region);
    p.positionInWorld = region.transformPlaneToWorld *
        Eigen::Vector3d(p.positionInTerrainFrame.x(), p.positionInTerrainFrame.y(), 0.0);
    p.cost = (query - p.positionInWorld).squaredNorm();
    if (p.cost < best.cost && projectionMatchesElevation(terrain, p)) best = p;
  }
  if (!best.regionPtr) throw std::runtime_error("No foothold plane consistent with elevation at query (" +
      std::to_string(query.x()) + ", " + std::to_string(query.y()) + ", " + std::to_string(query.z()) + ")");
  return best;
}
}  // namespace ocs2::legged_robot

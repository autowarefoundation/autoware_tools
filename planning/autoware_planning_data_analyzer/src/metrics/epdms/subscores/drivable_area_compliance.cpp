// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "drivable_area_compliance.hpp"

#include "../../geometry/metric_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

std::vector<LinearRing2d> create_vehicle_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto base_footprint = vehicle_info.createFootprint(0.0);

  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    vehicle_footprints.push_back(
      autoware_utils_geometry::transform_vector(
        base_footprint, autoware_utils_geometry::pose2transform(point.pose)));
  }

  return vehicle_footprints;
}

LinearRing2d create_hull_from_footprints(const std::vector<LinearRing2d> & footprints)
{
  boost::geometry::model::multi_point<Point2d> combined;
  for (const auto & footprint : footprints) {
    for (const auto & point : footprint) {
      combined.push_back(point);
    }
  }

  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);
  boost::geometry::correct(hull);
  return hull;
}

lanelet::ConstLanelets collect_candidate_lanelets(
  const lanelet::ConstLanelets & drivable_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  const auto footprint_hull = create_hull_from_footprints(vehicle_footprints);

  lanelet::ConstLanelets candidate_lanelets;
  for (const auto & lanelet : drivable_lanelets) {
    if (!boost::geometry::disjoint(lanelet.polygon2d().basicPolygon(), footprint_hull)) {
      candidate_lanelets.push_back(lanelet);
    }
  }

  return candidate_lanelets;
}

Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back(Point2d(point.x(), point.y()));
  }
  boost::geometry::correct(converted);
  return converted;
}

std::optional<MultiPolygon2d> fuse_lanelet_polygons(
  const lanelet::ConstLanelets & candidate_lanelets)
{
  if (candidate_lanelets.empty()) {
    return std::nullopt;
  }

  MultiPolygon2d lanelet_unions;
  MultiPolygon2d result;

  for (const auto & lanelet : candidate_lanelets) {
    const auto polygon = to_polygon_2d(lanelet.polygon2d().basicPolygon());
    boost::geometry::union_(lanelet_unions, polygon, result);
    lanelet_unions = result;
    result.clear();
  }

  if (lanelet_unions.empty()) {
    return std::nullopt;
  }

  for (auto & polygon : lanelet_unions) {
    boost::geometry::correct(polygon);
  }

  return lanelet_unions;
}

bool is_footprint_inside_any_polygon(
  const LinearRing2d & vehicle_footprint, const MultiPolygon2d & fused_polygons)
{
  return std::any_of(
    fused_polygons.begin(), fused_polygons.end(), [&vehicle_footprint](const auto & polygon) {
      return boost::geometry::within(vehicle_footprint, polygon);
    });
}

}  // namespace

DrivableAreaComplianceResult calculate_drivable_area_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const lanelet::ConstLanelets & drivable_lanelets,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  DrivableAreaComplianceResult result;
  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (drivable_lanelets.empty()) {
    result.reason = "unavailable_no_drivable_lanelets";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  const auto vehicle_footprints = create_vehicle_footprints(trajectory, vehicle_info);
  if (vehicle_footprints.empty() || vehicle_footprints.front().empty()) {
    result.reason = "unavailable_invalid_footprint";
    return result;
  }

  const auto candidate_lanelets = collect_candidate_lanelets(drivable_lanelets, vehicle_footprints);
  if (candidate_lanelets.empty()) {
    result.reason = "unavailable_no_candidate_lanelets";
    return result;
  }

  const auto fused_lanelet_polygons = fuse_lanelet_polygons(candidate_lanelets);
  if (!fused_lanelet_polygons) {
    result.reason = "unavailable_invalid_drivable_area_polygon";
    return result;
  }

  result.available = true;
  result.reason = "compliant";

  for (const auto & footprint : vehicle_footprints) {
    if (!is_footprint_inside_any_polygon(footprint, *fused_lanelet_polygons)) {
      result.reason = "non_compliant_footprint_outside_drivable_area";
      return result;
    }
  }

  result.score = 1.0;
  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

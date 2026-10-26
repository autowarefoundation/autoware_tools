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

#ifndef METRICS__GEOMETRY__EGO_FOOTPRINT_HPP_
#define METRICS__GEOMETRY__EGO_FOOTPRINT_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct EgoAreaFlags
{
  bool multiple_lanes{false};
  bool non_drivable_area{false};
  bool in_intersection{false};
  bool intersection_context_available{false};
  bool road_border_fallback_used{false};
};

struct RoadBorderFallbackProbe
{
  std::size_t corner_index{0};
  autoware_utils_geometry::Point2d corner;
  autoware_utils_geometry::Point2d semantic_closest_point;
  autoware_utils_geometry::Point2d segment_start;
  autoware_utils_geometry::Point2d segment_end;
  autoware_utils_geometry::Point2d closest_point;
  autoware_utils_geometry::Point2d plus_sample;
  autoware_utils_geometry::Point2d minus_sample;
  bool plus_sample_drivable{false};
  bool minus_sample_drivable{false};
  bool accepted{false};
  double distance_m{std::numeric_limits<double>::infinity()};
  double corner_semantic_distance_m{std::numeric_limits<double>::infinity()};
  double semantic_to_border_distance_m{std::numeric_limits<double>::infinity()};
  double corner_between_ratio{std::numeric_limits<double>::quiet_NaN()};
  double corner_to_semantic_border_line_m{std::numeric_limits<double>::infinity()};
  double border_tangent_alignment{std::numeric_limits<double>::infinity()};
  double plus_sample_semantic_distance_m{std::numeric_limits<double>::infinity()};
  double minus_sample_semantic_distance_m{std::numeric_limits<double>::infinity()};
};

struct EgoAreaEvaluation
{
  EgoAreaFlags flags;
  std::vector<autoware_utils_geometry::Point2d> footprint_points;
  std::vector<bool> corner_drivable;
  std::vector<bool> corner_drivable_by_road_border;
  std::vector<RoadBorderFallbackProbe> road_border_side_tests;
  lanelet::ConstLanelets road_lanelets;
  lanelet::ConstLanelets shoulder_lanelets;
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
  std::vector<lanelet::ConstPolygon3d> hatched_road_markings;
  std::vector<lanelet::ConstPolygon3d> parking_lots;
  std::vector<lanelet::ConstLineString3d> road_border_lines;
  std::size_t designated_lanelet_count{0};
};

struct TrajectoryFootprintEvaluation
{
  autoware_utils_geometry::Polygon2d ego_polygon;
  std::optional<EgoAreaEvaluation> ego_area_evaluation;
};

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint);

// Shared EPDMS geometry helper used by follow-up subscore PRs. It is intentionally introduced
// before its first caller so the larger footprint/map-context implementation can be reviewed
// independently from subscore semantic changes.
std::optional<EgoAreaEvaluation> compute_ego_area_evaluation(
  const geometry_msgs::msg::Pose & pose, const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & designated_lanelets = {},
  bool evaluate_intersection_context = false);

// Shared EPDMS geometry helper used by follow-up subscore PRs. Existing callers are not switched
// to this API in the preparatory helper PR.
std::vector<TrajectoryFootprintEvaluation> evaluate_trajectory_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler = nullptr,
  const lanelet::ConstLanelets * route_relevant_lanelets = nullptr,
  bool evaluate_intersection_context = false);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__EGO_FOOTPRINT_HPP_

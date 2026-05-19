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

#ifndef METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_
#define METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_

#include "metrics/geometry/ego_footprint.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct DrivableAreaComplianceHorizonFootprint
{
  double time_s{0.0};
  bool non_drivable_area{false};
  std::vector<geometry_msgs::msg::Point> footprint;
};

struct DrivableAreaComplianceDebugPolygon
{
  double time_s{0.0};
  std::vector<geometry_msgs::msg::Point> polygon;
};

struct DrivableAreaComplianceDebugCorner
{
  double time_s{0.0};
  std::size_t corner_index{0};
  geometry_msgs::msg::Point point;
};

struct DrivableAreaComplianceDebugInfo
{
  double first_failure_time_s{std::numeric_limits<double>::infinity()};
  std::size_t corner_count_inside{0};
  std::vector<std::size_t> failing_corner_indices;
  std::size_t route_candidate_count{0};
  std::size_t road_candidate_count{0};
  std::size_t shoulder_candidate_count{0};
  std::size_t intersection_candidate_count{0};
  std::size_t hatched_road_marking_candidate_count{0};
  std::size_t parking_candidate_count{0};
  std::size_t road_border_line_count{0};
  bool road_border_fallback_used{false};
  std::size_t road_border_side_test_count{0};
  std::size_t road_border_side_accept_count{0};
  geometry_msgs::msg::Point label_anchor;
  std::vector<DrivableAreaComplianceHorizonFootprint> ego_horizon_footprints;
  std::vector<DrivableAreaComplianceDebugPolygon> admissible_road_areas;
  std::vector<DrivableAreaComplianceDebugPolygon> admissible_shoulder_areas;
  std::vector<DrivableAreaComplianceDebugPolygon> admissible_intersection_areas;
  std::vector<DrivableAreaComplianceDebugPolygon> admissible_hatched_road_markings;
  std::vector<DrivableAreaComplianceDebugPolygon> admissible_parking_areas;
  std::vector<DrivableAreaComplianceDebugPolygon> road_border_lines;
  std::vector<DrivableAreaComplianceDebugPolygon> road_border_side_test_segments;
  std::vector<DrivableAreaComplianceDebugPolygon> road_border_gap_segments;
  std::vector<DrivableAreaComplianceDebugCorner> semantic_boundary_points;
  std::vector<DrivableAreaComplianceDebugCorner> road_border_closest_points;
  std::vector<DrivableAreaComplianceDebugCorner> corner_projection_points;
  std::vector<DrivableAreaComplianceDebugCorner> road_border_plus_samples;
  std::vector<DrivableAreaComplianceDebugCorner> road_border_minus_samples;
  std::vector<DrivableAreaComplianceDebugCorner> road_border_fallback_corners;
  std::vector<DrivableAreaComplianceDebugCorner> failing_corners;
};

struct DrivableAreaComplianceResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  DrivableAreaComplianceDebugInfo debug_info;
};

DrivableAreaComplianceResult calculate_drivable_area_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::vector<TrajectoryFootprintEvaluation> * footprint_evaluations = nullptr);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_

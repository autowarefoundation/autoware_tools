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

#ifndef METRICS__EPDMS__SUBSCORES__DRIVING_DIRECTION_COMPLIANCE_HPP_
#define METRICS__EPDMS__SUBSCORES__DRIVING_DIRECTION_COMPLIANCE_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct DrivingDirectionComplianceParameters
{
  double horizon_s{1.0};
  double compliance_threshold_m{2.0};
  double violation_threshold_m{6.0};
};

struct DrivingDirectionEvaluationPoint
{
  double time_from_start_s{0.0};
  double progress_m{0.0};
  bool in_oncoming_traffic{false};
  bool is_intersection{false};
};

struct DrivingDirectionComplianceResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double max_oncoming_progress_m{0.0};
  double worst_window_start_time_s{0.0};
  double worst_window_end_time_s{0.0};
  std::size_t worst_window_sample_count{0U};
};

struct DrivingDirectionDebugPolygon
{
  double time_s{0.0};
  std::vector<geometry_msgs::msg::Point> polygon;
};

struct DrivingDirectionDebugSample
{
  double time_s{0.0};
  double progress_m{0.0};
  double counted_progress_m{0.0};
  bool in_oncoming_traffic{false};
  bool in_lane_margin_only{false};
  bool is_intersection{false};
  geometry_msgs::msg::Point ego_center;
};

struct DrivingDirectionComplianceDebugInfo
{
  double worst_window_start_time_s{0.0};
  double worst_window_end_time_s{0.0};
  std::size_t worst_window_sample_count{0U};
  double window_progress_m{0.0};
  geometry_msgs::msg::Point label_anchor;
  std::vector<DrivingDirectionDebugSample> samples;
  std::vector<DrivingDirectionDebugPolygon> route_lane_polygons;
  std::vector<DrivingDirectionDebugPolygon> intersection_lane_polygons;
};

DrivingDirectionComplianceResult calculate_driving_direction_compliance(
  const std::vector<DrivingDirectionEvaluationPoint> & evaluation_points,
  const DrivingDirectionComplianceParameters & params = DrivingDirectionComplianceParameters{});

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__DRIVING_DIRECTION_COMPLIANCE_HPP_

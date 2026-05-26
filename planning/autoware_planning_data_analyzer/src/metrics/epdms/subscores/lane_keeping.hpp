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

#ifndef METRICS__EPDMS__SUBSCORES__LANE_KEEPING_HPP_
#define METRICS__EPDMS__SUBSCORES__LANE_KEEPING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct LaneKeepingParameters
{
  double max_lateral_deviation{0.5};
  double max_continuous_violation_time{2.0};
  double lane_change_pre_grace_time{1.0};
  double lane_change_post_grace_time{1.0};
  double queue_speed_threshold{1.0};
  double queue_progress_window_time{1.0};
  double queue_progress_threshold{1.5};
  double queue_release_grace_time{1.5};
};

struct LaneKeepingEvaluationPoint
{
  rclcpp::Duration time_from_start{0, 0};
  double lateral_deviation{0.0};
  bool is_in_intersection{false};
  geometry_msgs::msg::Point ego_center{};
  std::vector<geometry_msgs::msg::Point> reference_centerline;
  std::int64_t reference_lanelet_id{-1};
  double speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double cumulative_progress_m{std::numeric_limits<double>::quiet_NaN()};
};

struct LaneKeepingDebugSample
{
  double time_s{0.0};
  geometry_msgs::msg::Point ego_center{};
  double lateral_deviation{0.0};
  bool is_in_intersection{false};
  bool over_threshold{false};
  bool in_failure_run{false};
  bool lane_change_exempt{false};
  bool queue_exempt{false};
  bool queue_release_exempt{false};
  std::vector<geometry_msgs::msg::Point> reference_centerline;
  std::int64_t reference_lanelet_id{-1};
};

struct LaneKeepingDebugInfo
{
  std::vector<LaneKeepingDebugSample> samples;
  double first_failure_time_s{std::numeric_limits<double>::infinity()};
  double failure_run_start_time_s{std::numeric_limits<double>::infinity()};
  double failure_run_end_time_s{std::numeric_limits<double>::infinity()};
  double max_continuous_violation_time_s{0.0};
  double peak_abs_lateral_deviation_m{0.0};
  geometry_msgs::msg::Point label_anchor{};
};

struct LaneKeepingResult
{
  double score{0.0};
  LaneKeepingDebugInfo debug;
};

/**
 * @brief Evaluate the binary lane-keeping subscore for one trajectory.
 *
 * Returns `0.0` when the continuous over-threshold duration reaches the configured limit,
 * otherwise returns `1.0`.
 */
LaneKeepingResult calculate_lane_keeping_result(
  const std::vector<LaneKeepingEvaluationPoint> & evaluation_points,
  const LaneKeepingParameters & parameters = LaneKeepingParameters{},
  const std::vector<std::pair<double, double>> & lane_change_windows_s = {});

double calculate_lane_keeping_score(
  const std::vector<LaneKeepingEvaluationPoint> & evaluation_points,
  const LaneKeepingParameters & parameters = LaneKeepingParameters{},
  const std::vector<std::pair<double, double>> & lane_change_windows_s = {});

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__LANE_KEEPING_HPP_

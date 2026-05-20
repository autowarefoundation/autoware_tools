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

#include <limits>
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
  double speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double cumulative_progress_m{std::numeric_limits<double>::quiet_NaN()};
};

/**
 * @brief Evaluate the binary lane-keeping subscore for one trajectory.
 *
 * Returns `0.0` when the continuous over-threshold duration reaches the configured limit,
 * otherwise returns `1.0`.
 */
double calculate_lane_keeping_score(
  const std::vector<LaneKeepingEvaluationPoint> & evaluation_points,
  const LaneKeepingParameters & parameters = LaneKeepingParameters{},
  const std::vector<std::pair<double, double>> & lane_change_windows_s = {});

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__LANE_KEEPING_HPP_

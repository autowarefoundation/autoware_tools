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

#include "lane_keeping.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

LaneKeepingResult calculate_lane_keeping_result(
  const std::vector<LaneKeepingEvaluationPoint> & evaluation_points,
  const LaneKeepingParameters & parameters,
  const std::vector<std::pair<double, double>> & lane_change_windows_s)
{
  LaneKeepingResult result;
  if (
    evaluation_points.empty() || parameters.max_lateral_deviation < 0.0 ||
    parameters.max_continuous_violation_time < 0.0) {
    return result;
  }

  std::optional<rclcpp::Duration> violation_start_time;
  double max_violation_duration = 0.0;
  double peak_abs_lateral_deviation = 0.0;
  bool failure_recorded = false;
  std::optional<double> queue_release_until_s;

  const auto reset_violation_run = [&]() { violation_start_time.reset(); };

  for (std::size_t index = 0; index < evaluation_points.size(); ++index) {
    const auto & evaluation_point = evaluation_points.at(index);
    const double time_s = evaluation_point.time_from_start.seconds();
    const bool finite = std::isfinite(evaluation_point.lateral_deviation);
    if (!finite) {
      reset_violation_run();
      continue;
    }

    peak_abs_lateral_deviation =
      std::max(peak_abs_lateral_deviation, std::abs(evaluation_point.lateral_deviation));

    const bool over_threshold =
      std::abs(evaluation_point.lateral_deviation) > parameters.max_lateral_deviation;
    const bool lane_change_exempt = std::any_of(
      lane_change_windows_s.begin(), lane_change_windows_s.end(),
      [&](const auto & window) { return time_s >= window.first && time_s <= window.second; });

    const double progress_window_start =
      std::max(0.0, time_s - parameters.queue_progress_window_time);
    double progress_window_distance = 0.0;
    for (std::size_t lookback = index; lookback > 0; --lookback) {
      const auto & previous = evaluation_points.at(lookback - 1U);
      const auto & current = evaluation_points.at(lookback);
      if (current.time_from_start.seconds() < progress_window_start) {
        break;
      }
      progress_window_distance =
        evaluation_point.cumulative_progress_m - previous.cumulative_progress_m;
    }

    const bool queue_signal_available = std::isfinite(evaluation_point.speed_mps) &&
                                        std::isfinite(evaluation_point.cumulative_progress_m);
    const bool queue_exempt = queue_signal_available &&
                              evaluation_point.speed_mps <= parameters.queue_speed_threshold &&
                              progress_window_distance <= parameters.queue_progress_threshold;
    if (queue_exempt) {
      queue_release_until_s = time_s + parameters.queue_release_grace_time;
    }
    const bool queue_release_exempt =
      !queue_exempt && queue_release_until_s.has_value() && time_s <= *queue_release_until_s;

    if (
      evaluation_point.is_in_intersection || lane_change_exempt || queue_exempt ||
      queue_release_exempt || !over_threshold) {
      reset_violation_run();
      continue;
    }
    if (!violation_start_time.has_value()) {
      violation_start_time = evaluation_point.time_from_start;
    }

    const double violation_duration =
      (evaluation_point.time_from_start - violation_start_time.value()).seconds();
    max_violation_duration = std::max(max_violation_duration, violation_duration);
    if (!failure_recorded && violation_duration >= parameters.max_continuous_violation_time) {
      result.first_failure_time_s = time_s;
      failure_recorded = true;
    }
  }

  result.score = failure_recorded ? 0.0 : 1.0;
  result.max_continuous_violation_time_s = max_violation_duration;
  result.peak_abs_lateral_deviation_m = peak_abs_lateral_deviation;
  return result;
}

double calculate_lane_keeping_score(
  const std::vector<LaneKeepingEvaluationPoint> & evaluation_points,
  const LaneKeepingParameters & parameters,
  const std::vector<std::pair<double, double>> & lane_change_windows_s)
{
  return calculate_lane_keeping_result(evaluation_points, parameters, lane_change_windows_s).score;
}

}  // namespace autoware::planning_data_analyzer::metrics

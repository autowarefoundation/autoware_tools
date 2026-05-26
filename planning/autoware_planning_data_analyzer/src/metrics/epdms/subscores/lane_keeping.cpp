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
  std::optional<std::size_t> violation_start_index;
  double max_violation_duration = 0.0;
  double peak_abs_lateral_deviation = 0.0;
  bool failure_recorded = false;
  std::optional<double> queue_release_until_s;

  result.debug.samples.reserve(evaluation_points.size());

  const auto reset_violation_run = [&]() {
    violation_start_time.reset();
    violation_start_index.reset();
  };

  for (std::size_t index = 0; index < evaluation_points.size(); ++index) {
    const auto & evaluation_point = evaluation_points[index];
    const double time_s = evaluation_point.time_from_start.seconds();
    const bool finite = std::isfinite(evaluation_point.lateral_deviation);

    const bool over_threshold =
      finite && std::abs(evaluation_point.lateral_deviation) > parameters.max_lateral_deviation;
    const bool lane_change_exempt = std::any_of(
      lane_change_windows_s.begin(), lane_change_windows_s.end(),
      [&](const auto & window) { return time_s >= window.first && time_s <= window.second; });

    const double progress_window_start =
      std::max(0.0, time_s - parameters.queue_progress_window_time);
    std::size_t earliest_in_window = index;
    while (earliest_in_window > 0 &&
           evaluation_points.at(earliest_in_window - 1U).time_from_start.seconds() >=
             progress_window_start) {
      --earliest_in_window;
    }
    const double progress_window_distance =
      evaluation_point.cumulative_progress_m -
      evaluation_points.at(earliest_in_window).cumulative_progress_m;

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
    result.debug.samples.push_back(
      LaneKeepingDebugSample{
        time_s, evaluation_point.ego_center, evaluation_point.lateral_deviation,
        evaluation_point.is_in_intersection, over_threshold, false, lane_change_exempt,
        queue_exempt, queue_release_exempt, evaluation_point.reference_centerline,
        evaluation_point.reference_lanelet_id});

    if (!finite) {
      reset_violation_run();
      continue;
    }

    peak_abs_lateral_deviation =
      std::max(peak_abs_lateral_deviation, std::abs(evaluation_point.lateral_deviation));

    // Reset the violation run on any exemption: intersection samples, signalled lane-change
    // windows, queue, queue-release grace, or sub-threshold deviation.
    if (
      evaluation_point.is_in_intersection || lane_change_exempt || queue_exempt ||
      queue_release_exempt || !over_threshold) {
      reset_violation_run();
      continue;
    }
    if (!violation_start_time.has_value()) {
      violation_start_time = evaluation_point.time_from_start;
      violation_start_index = index;
    }

    const double violation_duration =
      (evaluation_point.time_from_start - *violation_start_time).seconds();
    max_violation_duration = std::max(max_violation_duration, violation_duration);
    if (!failure_recorded && violation_duration >= parameters.max_continuous_violation_time) {
      result.score = 0.0;
      result.debug.first_failure_time_s = time_s;
      result.debug.failure_run_start_time_s = violation_start_time->seconds();
      result.debug.failure_run_end_time_s = time_s;
      if (violation_start_index.has_value()) {
        for (std::size_t run_index = *violation_start_index; run_index <= index; ++run_index) {
          result.debug.samples.at(run_index).in_failure_run = true;
        }
        result.debug.label_anchor = result.debug.samples.at(*violation_start_index).ego_center;
      }
      failure_recorded = true;
    }
  }

  result.score = failure_recorded ? 0.0 : 1.0;
  result.debug.max_continuous_violation_time_s = max_violation_duration;
  result.debug.peak_abs_lateral_deviation_m = peak_abs_lateral_deviation;
  if (!failure_recorded && !result.debug.samples.empty()) {
    result.debug.label_anchor = result.debug.samples.front().ego_center;
  }
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

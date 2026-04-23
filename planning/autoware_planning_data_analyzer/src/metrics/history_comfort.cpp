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

#include "history_comfort.hpp"

#include "metric_utils.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

double calculate_time_resolution(
  const autoware_planning_msgs::msg::TrajectoryPoint & point,
  const autoware_planning_msgs::msg::TrajectoryPoint & next_point, const double epsilon)
{
  const double time_diff = rclcpp::Duration(next_point.time_from_start).seconds() -
                           rclcpp::Duration(point.time_from_start).seconds();
  return time_diff > epsilon ? time_diff : epsilon;
}

void backfill_last_value_from_previous(std::vector<double> & values)
{
  if (values.size() > 1U) {
    values.back() = values[values.size() - 2];
  }
}

bool is_within(const std::vector<double> & values, const double min_value, const double max_value)
{
  return std::all_of(values.begin(), values.end(), [min_value, max_value](const double value) {
    return min_value < value && value < max_value;
  });
}

bool is_abs_within(const std::vector<double> & values, const double max_abs_value)
{
  return std::all_of(values.begin(), values.end(), [max_abs_value](const double value) {
    return std::abs(value) < max_abs_value;
  });
}

}  // namespace

void calculate_history_comfort_metrics(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const HistoryComfortParameters & history_comfort_params, TrajectoryPointMetrics & metrics)
{
  const size_t num_points = trajectory.points.size();
  if (num_points == 0U) {
    return;
  }

  metrics.longitudinal_accelerations.resize(num_points, 0.0);
  metrics.lateral_accelerations.resize(num_points, 0.0);
  metrics.lateral_jerks.resize(num_points, 0.0);
  metrics.jerk_magnitudes.resize(num_points, 0.0);
  metrics.longitudinal_jerks.resize(num_points, 0.0);
  metrics.yaw_rates.resize(num_points, 0.0);
  metrics.yaw_accelerations.resize(num_points, 0.0);

  for (size_t i = 0; i < num_points - 1; ++i) {
    const double time_resolution = calculate_time_resolution(
      trajectory.points[i], trajectory.points[i + 1],
      history_comfort_params.finite_difference_epsilon);

    const double yaw_delta = autoware_utils_math::normalize_radian(
      get_yaw(trajectory.points[i + 1].pose.orientation) -
      get_yaw(trajectory.points[i].pose.orientation));
    metrics.yaw_rates[i] = yaw_delta / time_resolution;
    metrics.longitudinal_accelerations[i] = (trajectory.points[i + 1].longitudinal_velocity_mps -
                                             trajectory.points[i].longitudinal_velocity_mps) /
                                            time_resolution;
    metrics.lateral_accelerations[i] =
      trajectory.points[i].longitudinal_velocity_mps * metrics.yaw_rates[i];
  }
  backfill_last_value_from_previous(metrics.longitudinal_accelerations);
  backfill_last_value_from_previous(metrics.lateral_accelerations);
  backfill_last_value_from_previous(metrics.yaw_rates);

  for (size_t i = 0; i < num_points - 1; ++i) {
    const double time_resolution = calculate_time_resolution(
      trajectory.points[i], trajectory.points[i + 1],
      history_comfort_params.finite_difference_epsilon);

    metrics.longitudinal_jerks[i] =
      (metrics.longitudinal_accelerations[i + 1] - metrics.longitudinal_accelerations[i]) /
      time_resolution;
    metrics.lateral_jerks[i] =
      (metrics.lateral_accelerations[i + 1] - metrics.lateral_accelerations[i]) / time_resolution;
    metrics.jerk_magnitudes[i] =
      std::hypot(metrics.longitudinal_jerks[i], metrics.lateral_jerks[i]);
    metrics.yaw_accelerations[i] =
      (metrics.yaw_rates[i + 1] - metrics.yaw_rates[i]) / time_resolution;
  }
  backfill_last_value_from_previous(metrics.longitudinal_jerks);
  backfill_last_value_from_previous(metrics.lateral_jerks);
  backfill_last_value_from_previous(metrics.jerk_magnitudes);
  backfill_last_value_from_previous(metrics.yaw_accelerations);

  const bool longitudinal_acceleration_ok = is_within(
    metrics.longitudinal_accelerations, history_comfort_params.min_longitudinal_acceleration,
    history_comfort_params.max_longitudinal_acceleration);
  const bool lateral_acceleration_ok =
    is_abs_within(metrics.lateral_accelerations, history_comfort_params.max_lateral_acceleration);
  const bool jerk_magnitude_ok =
    is_abs_within(metrics.jerk_magnitudes, history_comfort_params.max_jerk_magnitude);
  const bool longitudinal_jerk_ok =
    is_abs_within(metrics.longitudinal_jerks, history_comfort_params.max_longitudinal_jerk);
  const bool yaw_rate_ok = is_abs_within(metrics.yaw_rates, history_comfort_params.max_yaw_rate);
  const bool yaw_acceleration_ok =
    is_abs_within(metrics.yaw_accelerations, history_comfort_params.max_yaw_acceleration);

  metrics.history_comfort = longitudinal_acceleration_ok && lateral_acceleration_ok &&
                                jerk_magnitude_ok && longitudinal_jerk_ok && yaw_rate_ok &&
                                yaw_acceleration_ok
                              ? 1.0
                              : 0.0;
}

}  // namespace autoware::planning_data_analyzer::metrics

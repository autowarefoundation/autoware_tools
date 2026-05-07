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

#include "extended_comfort.hpp"

#include "metrics/geometry/metric_utils.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

struct DynamicSignals
{
  std::vector<double> accelerations;
  std::vector<double> jerks;
  std::vector<double> yaw_rates;
  std::vector<double> yaw_accelerations;
};

double get_time_seconds(const builtin_interfaces::msg::Duration & time_from_start)
{
  return static_cast<double>(time_from_start.sec) +
         static_cast<double>(time_from_start.nanosec) * 1e-9;
}

double compute_rms_difference(const std::vector<double> & lhs, const std::vector<double> & rhs)
{
  const auto count = std::min(lhs.size(), rhs.size());
  if (count == 0U) {
    return 0.0;
  }

  double sum_squared_error = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double error = lhs.at(i) - rhs.at(i);
    sum_squared_error += error * error;
  }

  return std::sqrt(sum_squared_error / static_cast<double>(count));
}

DynamicSignals calculate_dynamic_signals(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double epsilon)
{
  DynamicSignals signals;
  const auto num_points = trajectory.points.size();
  if (num_points < 2U) {
    return signals;
  }

  signals.accelerations.resize(num_points - 1U, 0.0);
  signals.yaw_rates.resize(num_points - 1U, 0.0);

  for (std::size_t i = 0; i + 1U < num_points; ++i) {
    const auto & point = trajectory.points.at(i);
    const auto & next_point = trajectory.points.at(i + 1U);
    const double dt = std::max(
      get_time_seconds(next_point.time_from_start) - get_time_seconds(point.time_from_start),
      epsilon);

    const double speed = std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
    const double next_speed =
      std::hypot(next_point.longitudinal_velocity_mps, next_point.lateral_velocity_mps);
    signals.accelerations.at(i) = (next_speed - speed) / dt;

    const double yaw = get_yaw(point.pose.orientation);
    const double next_yaw = get_yaw(next_point.pose.orientation);
    const double delta_yaw = autoware_utils_math::normalize_radian(next_yaw - yaw);
    signals.yaw_rates.at(i) = delta_yaw / dt;
  }

  if (signals.accelerations.size() >= 2U) {
    signals.jerks.resize(signals.accelerations.size() - 1U, 0.0);
    signals.yaw_accelerations.resize(signals.yaw_rates.size() - 1U, 0.0);

    for (std::size_t i = 0; i + 1U < signals.accelerations.size(); ++i) {
      const auto & point = trajectory.points.at(i);
      const auto & next_point = trajectory.points.at(i + 1U);
      const double dt = std::max(
        get_time_seconds(next_point.time_from_start) - get_time_seconds(point.time_from_start),
        epsilon);
      signals.jerks.at(i) = (signals.accelerations.at(i + 1U) - signals.accelerations.at(i)) / dt;
      signals.yaw_accelerations.at(i) =
        (signals.yaw_rates.at(i + 1U) - signals.yaw_rates.at(i)) / dt;
    }
  }

  return signals;
}

bool are_parameters_valid(const ExtendedComfortParameters & parameters)
{
  return parameters.max_acceleration_rms >= 0.0 && parameters.max_jerk_rms >= 0.0 &&
         parameters.max_yaw_rate_rms >= 0.0 && parameters.max_yaw_acceleration_rms >= 0.0 &&
         parameters.finite_difference_epsilon > 0.0;
}

}  // namespace

ExtendedComfortResult calculate_extended_comfort(
  const autoware_planning_msgs::msg::Trajectory & previous_trajectory,
  const autoware_planning_msgs::msg::Trajectory & current_trajectory,
  const ExtendedComfortParameters & parameters)
{
  if (!are_parameters_valid(parameters)) {
    return {0.0, false, "unavailable_invalid_parameters"};
  }

  if (previous_trajectory.points.size() < 3U || current_trajectory.points.size() < 3U) {
    return {0.0, false, "unavailable_short_trajectory"};
  }

  auto previous_signals =
    calculate_dynamic_signals(previous_trajectory, parameters.finite_difference_epsilon);
  auto current_signals =
    calculate_dynamic_signals(current_trajectory, parameters.finite_difference_epsilon);

  if (
    previous_signals.accelerations.empty() || current_signals.accelerations.empty() ||
    previous_signals.jerks.empty() || current_signals.jerks.empty() ||
    previous_signals.yaw_rates.empty() || current_signals.yaw_rates.empty() ||
    previous_signals.yaw_accelerations.empty() || current_signals.yaw_accelerations.empty()) {
    return {0.0, false, "unavailable_short_trajectory"};
  }

  const double acceleration_rms =
    compute_rms_difference(previous_signals.accelerations, current_signals.accelerations);
  const double jerk_rms = compute_rms_difference(previous_signals.jerks, current_signals.jerks);
  const double yaw_rate_rms =
    compute_rms_difference(previous_signals.yaw_rates, current_signals.yaw_rates);
  const double yaw_acceleration_rms =
    compute_rms_difference(previous_signals.yaw_accelerations, current_signals.yaw_accelerations);

  if (
    acceleration_rms > parameters.max_acceleration_rms || jerk_rms > parameters.max_jerk_rms ||
    yaw_rate_rms > parameters.max_yaw_rate_rms ||
    yaw_acceleration_rms > parameters.max_yaw_acceleration_rms) {
    return {0.0, true, "available"};
  }

  return {1.0, true, "available"};
}

}  // namespace autoware::planning_data_analyzer::metrics

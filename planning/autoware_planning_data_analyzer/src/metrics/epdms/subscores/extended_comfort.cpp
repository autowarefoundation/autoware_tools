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

#include "metrics/geometry/comfort_signal.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

constexpr double kDefaultTrajectoryDt = 0.1;

double trajectory_dt_s(const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.size() < 2U) {
    return kDefaultTrajectoryDt;
  }
  const double dt = rclcpp::Duration(trajectory.points[1].time_from_start).seconds() -
                    rclcpp::Duration(trajectory.points.front().time_from_start).seconds();
  return dt > 0.0 ? dt : kDefaultTrajectoryDt;
}

/**
 * @brief Prepare ComfortSignalInputs for a trajectory segment.
 * @note Trajectory points do not carry lateral acceleration; EC therefore compares
 * longitudinal-only acceleration magnitudes for planned horizons.
 */
std::vector<ComfortSignalInput> make_overlap_signal_inputs(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const std::size_t start_index,
  const std::size_t count)
{
  std::vector<ComfortSignalInput> inputs;
  inputs.reserve(count);
  const double start_time_s =
    rclcpp::Duration(trajectory.points[start_index].time_from_start).seconds();
  for (std::size_t index = 0; index < count; ++index) {
    const auto & point = trajectory.points[start_index + index];
    const double time_s = rclcpp::Duration(point.time_from_start).seconds() - start_time_s;
    inputs.push_back(ComfortSignalInput{time_s, point.pose, point.acceleration_mps2, 0.0});
  }
  return inputs;
}

std::vector<double> subtract_signals(
  const std::vector<double> & current, const std::vector<double> & previous)
{
  const auto count = std::min(current.size(), previous.size());
  std::vector<double> delta;
  delta.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    delta.push_back(current.at(index) - previous.at(index));
  }
  return delta;
}

double rms(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum_squared = 0.0;
  for (const auto value : values) {
    sum_squared += value * value;
  }
  return std::sqrt(sum_squared / static_cast<double>(values.size()));
}

bool are_parameters_valid(const ExtendedComfortParameters & parameters)
{
  return parameters.max_acceleration_rms >= 0.0 && parameters.max_jerk_rms >= 0.0 &&
         parameters.max_yaw_rate_rms >= 0.0 && parameters.max_yaw_acceleration_rms >= 0.0;
}

}  // namespace

ExtendedComfortResult calculate_extended_comfort(
  const autoware_planning_msgs::msg::Trajectory & previous_trajectory,
  const autoware_planning_msgs::msg::Trajectory & current_trajectory,
  const ExtendedComfortParameters & parameters)
{
  ExtendedComfortResult result;
  if (!are_parameters_valid(parameters)) {
    result.reason = "unavailable_invalid_parameters";
    return result;
  }

  const double dt = trajectory_dt_s(current_trajectory);
  const double raw_observation_interval =
    (rclcpp::Time(current_trajectory.header.stamp) - rclcpp::Time(previous_trajectory.header.stamp))
      .seconds();
  if (raw_observation_interval < 0.0) {
    result.reason = "unavailable_invalid_time_alignment";
    return result;
  }
  const double observation_interval =
    raw_observation_interval > 0.0 ? raw_observation_interval : dt;
  const auto overlap_shift = static_cast<std::size_t>(std::llround(observation_interval / dt));
  if (overlap_shift == 0U) {
    result.reason = "unavailable_invalid_time_alignment";
    return result;
  }

  const auto previous_size = previous_trajectory.points.size();
  const auto current_size = current_trajectory.points.size();
  if (previous_size < 3U || current_size < 3U || overlap_shift >= previous_size) {
    result.reason = "unavailable_short_trajectory";
    return result;
  }

  const auto overlap_count = std::min(current_size, previous_size - overlap_shift);
  if (overlap_count < 3U) {
    result.reason = "unavailable_short_overlap";
    return result;
  }
  result.sample_times.reserve(overlap_count);
  for (std::size_t index = 0; index < overlap_count; ++index) {
    result.sample_times.push_back(static_cast<double>(index) * dt);
  }

  const auto current_inputs = make_overlap_signal_inputs(current_trajectory, 0U, overlap_count);
  const auto previous_inputs =
    make_overlap_signal_inputs(previous_trajectory, overlap_shift, overlap_count);
  const auto current_signals = compute_comfort_signals(current_inputs);
  const auto previous_signals = compute_comfort_signals(previous_inputs);

  result.delta_acceleration = subtract_signals(
    current_signals.acceleration_magnitudes, previous_signals.acceleration_magnitudes);
  result.delta_jerk =
    subtract_signals(current_signals.jerk_magnitudes, previous_signals.jerk_magnitudes);
  result.delta_yaw_rate = subtract_signals(current_signals.yaw_rates, previous_signals.yaw_rates);
  result.delta_yaw_accel =
    subtract_signals(current_signals.yaw_accelerations, previous_signals.yaw_accelerations);

  const double acceleration_rms = rms(result.delta_acceleration);
  const double jerk_rms = rms(result.delta_jerk);
  const double yaw_rate_rms = rms(result.delta_yaw_rate);
  const double yaw_acceleration_rms = rms(result.delta_yaw_accel);

  const bool acceleration_ok = acceleration_rms <= parameters.max_acceleration_rms;
  const bool jerk_ok = jerk_rms <= parameters.max_jerk_rms;
  const bool yaw_rate_ok = yaw_rate_rms <= parameters.max_yaw_rate_rms;
  const bool yaw_acceleration_ok = yaw_acceleration_rms <= parameters.max_yaw_acceleration_rms;

  result.available = true;
  result.reason = "available";
  result.score = acceleration_ok && jerk_ok && yaw_rate_ok && yaw_acceleration_ok ? 1.0 : 0.0;
  std::ostringstream summary;
  summary << "{\"score\":" << result.score << ",\"reason\":\"" << result.reason
          << "\",\"rms_acceleration\":" << acceleration_rms << ",\"rms_jerk\":" << jerk_rms
          << ",\"rms_yaw_rate\":" << yaw_rate_rms << ",\"rms_yaw_accel\":" << yaw_acceleration_rms
          << "}";
  result.debug_summary = summary.str();
  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

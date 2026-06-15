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

#include "metrics/geometry/comfort_signal.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

constexpr double kClosestSampleToleranceFactor = 0.75;

struct ComfortState
{
  double time_s{0.0};
  geometry_msgs::msg::Pose pose;
  double longitudinal_acceleration_mps2{0.0};
  double lateral_acceleration_mps2{0.0};
  double segment_id{2.0};
};

template <typename MessageT>
std::shared_ptr<const MessageT> closest_message(
  const std::vector<std::shared_ptr<const MessageT>> & history, const rclcpp::Time & target,
  const double tolerance_s)
{
  if (history.empty()) {
    return nullptr;
  }

  const auto it = std::lower_bound(
    history.begin(), history.end(), target,
    [](const auto & msg, const rclcpp::Time & t) { return rclcpp::Time(msg->header.stamp) < t; });

  std::shared_ptr<const MessageT> best;
  double best_diff_s = std::numeric_limits<double>::max();

  // Check the element at 'it' and the one before it
  auto check_best = [&](const auto & iter) {
    if (iter != history.end() && *iter) {
      const double diff_s = std::abs((rclcpp::Time((*iter)->header.stamp) - target).seconds());
      if (diff_s < best_diff_s) {
        best_diff_s = diff_s;
        best = *iter;
      }
    }
  };

  check_best(it);
  if (it != history.begin()) {
    check_best(std::prev(it));
  }

  return best_diff_s <= tolerance_s ? best : nullptr;
}

ComfortState make_state_from_odometry(
  const Odometry & odometry, const AccelWithCovarianceStamped * acceleration,
  const double relative_time_s)
{
  ComfortState state;
  state.time_s = relative_time_s;
  state.pose = odometry.pose.pose;
  state.segment_id = 0.0;
  if (acceleration) {
    // Note: This assumes the acceleration is in the vehicle body frame (base_link).
    state.longitudinal_acceleration_mps2 = acceleration->accel.accel.linear.x;
    state.lateral_acceleration_mps2 = acceleration->accel.accel.linear.y;
  }
  return state;
}

ComfortState make_state_from_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & point, const double relative_time_s)
{
  ComfortState state;
  state.time_s = relative_time_s;
  state.pose = point.pose;
  state.longitudinal_acceleration_mps2 = point.acceleration_mps2;
  // Trajectory points don't carry lateral acceleration; NAVSIM HC is history-focused, so future
  // lateral_acceleration_mps2 is left at 0 by design rather than derived from yaw rate.
  return state;
}

std::vector<ComfortState> build_padded_states(
  const SynchronizedData & sync_data, const HistoryComfortParameters & parameters)
{
  std::vector<ComfortState> states;
  if (!sync_data.trajectory || parameters.sample_interval_s <= 0.0) {
    return states;
  }

  const auto trajectory_start = rclcpp::Time(sync_data.trajectory->header.stamp);
  const double dt = parameters.sample_interval_s;
  for (double t = -parameters.past_horizon_s; t < -dt / 2.0; t += dt) {
    const auto target = trajectory_start + rclcpp::Duration::from_seconds(t);
    const auto odom = closest_message(
      sync_data.kinematic_state_history, target, dt * kClosestSampleToleranceFactor);
    if (!odom) {
      continue;
    }
    const auto accel =
      closest_message(sync_data.acceleration_history, target, dt * kClosestSampleToleranceFactor);
    states.push_back(make_state_from_odometry(*odom, accel.get(), t));
  }

  for (const auto & point : sync_data.trajectory->points) {
    const double t = rclcpp::Duration(point.time_from_start).seconds();
    if (t < 0.0) {
      continue;
    }
    if (t > parameters.future_horizon_s + 1.0e-6) {
      break;
    }
    states.push_back(make_state_from_trajectory_point(point, t));
  }
  return states;
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
  const SynchronizedData & sync_data, const HistoryComfortParameters & history_comfort_params,
  TrajectoryPointMetrics & metrics)
{
  const auto states = build_padded_states(sync_data, history_comfort_params);
  if (states.empty()) {
    metrics.history_comfort = 0.0;
    metrics.history_comfort_available = false;
    metrics.history_comfort_reason = "unavailable_no_motion_history";
    return;
  }

  std::vector<ComfortSignalInput> signal_inputs;
  signal_inputs.reserve(states.size());
  metrics.history_comfort_sample_times.reserve(states.size());
  metrics.history_comfort_segment_ids.reserve(states.size());
  metrics.history_comfort_sample_poses.reserve(states.size());
  for (const auto & state : states) {
    metrics.history_comfort_sample_times.push_back(state.time_s);
    metrics.history_comfort_segment_ids.push_back(state.segment_id);
    metrics.history_comfort_sample_poses.push_back(state.pose);
    signal_inputs.push_back(
      ComfortSignalInput{
        state.time_s, state.pose, state.longitudinal_acceleration_mps2,
        state.lateral_acceleration_mps2});
  }

  const auto signals =
    compute_comfort_signals(signal_inputs, history_comfort_params.past_horizon_s);
  metrics.longitudinal_accelerations = signals.longitudinal_accelerations;
  metrics.lateral_accelerations = signals.lateral_accelerations;
  metrics.longitudinal_jerks = signals.longitudinal_jerks;
  metrics.lateral_jerks = signals.lateral_jerks;
  metrics.jerk_magnitudes = signals.jerk_magnitudes;
  metrics.yaw_rates = signals.yaw_rates;
  metrics.yaw_accelerations = signals.yaw_accelerations;

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
  metrics.history_comfort_available = true;
  metrics.history_comfort_reason = "available";

  std::ostringstream summary;
  summary << "{\"score\":" << metrics.history_comfort << ",\"sample_count\":" << states.size()
          << ",\"past_horizon_s\":" << history_comfort_params.past_horizon_s
          << ",\"future_horizon_s\":" << history_comfort_params.future_horizon_s
          << ",\"longitudinal_acceleration_ok\":"
          << (longitudinal_acceleration_ok ? "true" : "false")
          << ",\"lateral_acceleration_ok\":" << (lateral_acceleration_ok ? "true" : "false")
          << ",\"jerk_magnitude_ok\":" << (jerk_magnitude_ok ? "true" : "false")
          << ",\"longitudinal_jerk_ok\":" << (longitudinal_jerk_ok ? "true" : "false")
          << ",\"yaw_rate_ok\":" << (yaw_rate_ok ? "true" : "false")
          << ",\"yaw_acceleration_ok\":" << (yaw_acceleration_ok ? "true" : "false") << "}";
  metrics.history_comfort_debug_summary = summary.str();
}

}  // namespace autoware::planning_data_analyzer::metrics

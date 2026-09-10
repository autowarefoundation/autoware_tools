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

#include "trajectory_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>

namespace autoware::planning_data_analyzer::metrics
{

double point_time_s(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

namespace
{

double lerp(const double lhs, const double rhs, const double ratio)
{
  return lhs + (rhs - lhs) * ratio;
}

}  // namespace

autoware_planning_msgs::msg::TrajectoryPoint interpolate_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & previous,
  const autoware_planning_msgs::msg::TrajectoryPoint & next, const double horizon_s)
{
  const double previous_t = point_time_s(previous);
  const double next_t = point_time_s(next);
  const double ratio = next_t > previous_t
                         ? std::clamp((horizon_s - previous_t) / (next_t - previous_t), 0.0, 1.0)
                         : 0.0;

  auto interpolated = previous;
  interpolated.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio);
  interpolated.longitudinal_velocity_mps =
    lerp(previous.longitudinal_velocity_mps, next.longitudinal_velocity_mps, ratio);
  interpolated.lateral_velocity_mps =
    lerp(previous.lateral_velocity_mps, next.lateral_velocity_mps, ratio);
  interpolated.acceleration_mps2 = lerp(previous.acceleration_mps2, next.acceleration_mps2, ratio);
  interpolated.heading_rate_rps = lerp(previous.heading_rate_rps, next.heading_rate_rps, ratio);
  interpolated.front_wheel_angle_rad =
    lerp(previous.front_wheel_angle_rad, next.front_wheel_angle_rad, ratio);
  interpolated.rear_wheel_angle_rad =
    lerp(previous.rear_wheel_angle_rad, next.rear_wheel_angle_rad, ratio);
  interpolated.time_from_start = rclcpp::Duration::from_seconds(horizon_s);
  return interpolated;
}

autoware_planning_msgs::msg::Trajectory truncate_trajectory_by_horizon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double horizon_s)
{
  constexpr double kTimeEpsilon = 1.0e-6;
  if (horizon_s <= 0.0 || trajectory.points.empty()) {
    return trajectory;
  }

  autoware_planning_msgs::msg::Trajectory truncated;
  truncated.header = trajectory.header;
  truncated.points.reserve(trajectory.points.size());

  for (const auto & point : trajectory.points) {
    const double t = point_time_s(point);
    if (t <= horizon_s + kTimeEpsilon) {
      truncated.points.push_back(point);
      continue;
    }

    if (!truncated.points.empty()) {
      const double previous_t = point_time_s(truncated.points.back());
      if (previous_t < horizon_s - kTimeEpsilon) {
        truncated.points.push_back(
          interpolate_trajectory_point(truncated.points.back(), point, horizon_s));
      }
    }
    if (truncated.points.empty()) {
      truncated.points.push_back(point);
    }
    return truncated;
  }

  return truncated;
}

}  // namespace autoware::planning_data_analyzer::metrics

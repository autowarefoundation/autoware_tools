// Copyright 2025 TIER IV, Inc.
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

#include "deviation_metrics.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace metrics
{

double calculate_distance_2d(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return autoware_utils_geometry::calc_distance2d(p1, p2);
}

DisplacementErrors calculate_displacement_errors(
  const std::vector<geometry_msgs::msg::Point> & predicted,
  const std::vector<geometry_msgs::msg::Point> & ground_truth)
{
  DisplacementErrors result;

  if (predicted.empty() || predicted.size() != ground_truth.size()) {
    return result;
  }

  result.point_wise_errors.resize(predicted.size());

  // Calculate 2D distance at each point
  for (size_t i = 0; i < predicted.size(); ++i) {
    result.point_wise_errors[i] = calculate_distance_2d(predicted[i], ground_truth[i]);
  }

  // ADE: Average Displacement Error
  result.ade = std::accumulate(
                 result.point_wise_errors.begin(), result.point_wise_errors.end(), 0.0) /
               result.point_wise_errors.size();

  // FDE: Final Displacement Error (error at last point)
  result.fde = result.point_wise_errors.back();

  return result;
}

std::pair<double, double> calculate_errors_in_vehicle_frame(
  const geometry_msgs::msg::Pose & trajectory_pose,
  const geometry_msgs::msg::Pose & ground_truth_pose)
{
  // Get yaw angle from ground truth
  const double gt_yaw = tf2::getYaw(ground_truth_pose.orientation);

  // Position difference in global coordinate frame
  const double dx_global = trajectory_pose.position.x - ground_truth_pose.position.x;
  const double dy_global = trajectory_pose.position.y - ground_truth_pose.position.y;

  // Transform to vehicle coordinate frame (rotate by -gt_yaw)
  const double cos_yaw = std::cos(-gt_yaw);
  const double sin_yaw = std::sin(-gt_yaw);

  const double dx_vehicle = dx_global * cos_yaw - dy_global * sin_yaw;  // longitudinal
  const double dy_vehicle = dx_global * sin_yaw + dy_global * cos_yaw;  // lateral

  return std::make_pair(dx_vehicle, dy_vehicle);
}

FrenetErrors calculate_frenet_errors(
  const std::vector<geometry_msgs::msg::Pose> & predicted,
  const std::vector<geometry_msgs::msg::Pose> & ground_truth)
{
  FrenetErrors result;

  if (predicted.empty() || predicted.size() != ground_truth.size()) {
    return result;
  }

  result.lateral_errors.resize(predicted.size());
  result.longitudinal_errors.resize(predicted.size());

  // Calculate vehicle frame errors at each point
  for (size_t i = 0; i < predicted.size(); ++i) {
    const auto [lon_err, lat_err] =
      calculate_errors_in_vehicle_frame(predicted[i], ground_truth[i]);
    result.longitudinal_errors[i] = lon_err;
    result.lateral_errors[i] = lat_err;
  }

  // Calculate statistics
  result.lateral_stats = calculate_statistics(result.lateral_errors);
  result.longitudinal_stats = calculate_statistics(result.longitudinal_errors);

  return result;
}

}  // namespace metrics

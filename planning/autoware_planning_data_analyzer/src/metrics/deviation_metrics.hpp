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

#ifndef DEVIATION_METRICS_HPP_
#define DEVIATION_METRICS_HPP_

#include "metric_types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

#include <utility>
#include <vector>

namespace metrics
{

/**
 * @brief ADE/FDE calculation results
 */
struct DisplacementErrors
{
  double ade = 0.0;                       // Average Displacement Error
  double fde = 0.0;                       // Final Displacement Error
  std::vector<double> point_wise_errors;  // 2D distance at each point

  nlohmann::json to_json() const
  {
    return {{"ade", ade}, {"fde", fde}, {"point_wise_errors", point_wise_errors}};
  }
};

/**
 * @brief Errors in Frenet coordinate frame
 */
struct FrenetErrors
{
  std::vector<double> lateral_errors;       // Lateral errors
  std::vector<double> longitudinal_errors;  // Longitudinal errors
  Statistics lateral_stats;
  Statistics longitudinal_stats;

  nlohmann::json to_json() const
  {
    return {
      {"lateral_errors", lateral_errors},
      {"longitudinal_errors", longitudinal_errors},
      {"lateral_statistics", lateral_stats.to_json()},
      {"longitudinal_statistics", longitudinal_stats.to_json()}};
  }
};

/**
 * @brief Calculate 2D Euclidean distance (pure function)
 *
 * Uses autoware_utils_geometry::calc_distance2d
 *
 * @param p1 Point 1
 * @param p2 Point 2
 * @return 2D distance [m]
 */
double calculate_distance_2d(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

/**
 * @brief Calculate displacement errors (ADE/FDE) (pure function)
 *
 * @param predicted Predicted trajectory points
 * @param ground_truth Ground truth trajectory points
 * @return ADE/FDE and related metrics
 *
 * Precondition: predicted.size() == ground_truth.size() > 0
 */
DisplacementErrors calculate_displacement_errors(
  const std::vector<geometry_msgs::msg::Point> & predicted,
  const std::vector<geometry_msgs::msg::Point> & ground_truth);

/**
 * @brief Calculate lateral and longitudinal errors in vehicle coordinate frame (pure function)
 *
 * @param trajectory_pose Predicted trajectory pose
 * @param ground_truth_pose Ground truth pose
 * @return (longitudinal_error, lateral_error)
 *
 * Coordinate transformation:
 * 1. Calculate difference vector
 * 2. Rotate by ground_truth yaw to convert to vehicle frame
 */
std::pair<double, double> calculate_errors_in_vehicle_frame(
  const geometry_msgs::msg::Pose & trajectory_pose,
  const geometry_msgs::msg::Pose & ground_truth_pose);

/**
 * @brief Calculate Frenet errors for pose sequences (pure function)
 *
 * @param predicted Predicted trajectory poses
 * @param ground_truth Ground truth trajectory poses
 * @return Errors and statistics in Frenet coordinate frame
 */
FrenetErrors calculate_frenet_errors(
  const std::vector<geometry_msgs::msg::Pose> & predicted,
  const std::vector<geometry_msgs::msg::Pose> & ground_truth);

}  // namespace metrics

#endif  // DEVIATION_METRICS_HPP_

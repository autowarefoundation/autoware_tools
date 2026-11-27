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

#include "metrics/geometry/ego_path.hpp"

#include <cmath>

namespace autoware::planning_data_analyzer::metrics
{

EgoPath to_ego_path(const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  EgoPath path;
  path.start_time = rclcpp::Time(trajectory.header.stamp);
  path.points.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    EgoPathPoint path_point;
    path_point.time_from_start = rclcpp::Duration(point.time_from_start);
    path_point.pose = point.pose;
    path_point.speed_mps = std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
    path.points.push_back(path_point);
  }
  return path;
}

}  // namespace autoware::planning_data_analyzer::metrics

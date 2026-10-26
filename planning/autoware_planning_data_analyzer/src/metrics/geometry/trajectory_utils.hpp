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

#ifndef METRICS__GEOMETRY__TRAJECTORY_UTILS_HPP_
#define METRICS__GEOMETRY__TRAJECTORY_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace autoware::planning_data_analyzer::metrics
{

double point_time_s(const autoware_planning_msgs::msg::TrajectoryPoint & point);

autoware_planning_msgs::msg::TrajectoryPoint interpolate_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & previous,
  const autoware_planning_msgs::msg::TrajectoryPoint & next, double horizon_s);

autoware_planning_msgs::msg::Trajectory truncate_trajectory_by_horizon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, double horizon_s);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__TRAJECTORY_UTILS_HPP_

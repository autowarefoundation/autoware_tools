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

#ifndef METRICS__GEOMETRY__EGO_PATH_HPP_
#define METRICS__GEOMETRY__EGO_PATH_HPP_

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

// One pose of the ego along a path, at a time measured from the path's start.
struct EgoPathPoint
{
  // A Duration, not seconds, so that times built from a trajectory stay exact.
  rclcpp::Duration time_from_start{0, 0};
  geometry_msgs::msg::Pose pose;
  // Planar speed: metrics that ask whether the ego stands still read this.
  double speed_mps{0.0};
};

// The poses an ego takes over time: a planned trajectory (open loop) or the path it drove (closed
// loop). start_time is the absolute time of a zero time_from_start; logged objects are looked up
// at start_time + time_from_start.
struct EgoPath
{
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  std::vector<EgoPathPoint> points;
};

// The planned trajectory as an ego path: start_time is the header stamp, and each point keeps its
// time_from_start, pose and speed (the norm of its longitudinal and lateral velocities).
EgoPath to_ego_path(const autoware_planning_msgs::msg::Trajectory & trajectory);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__EGO_PATH_HPP_

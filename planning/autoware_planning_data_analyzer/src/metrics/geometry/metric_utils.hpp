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

#ifndef METRICS__GEOMETRY__METRIC_UTILS_HPP_
#define METRICS__GEOMETRY__METRIC_UTILS_HPP_

#include "ego_footprint.hpp"
#include "lanelet_queries.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace autoware::planning_data_analyzer::metrics
{

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

double get_yaw(const geometry_msgs::msg::Quaternion & orientation);

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

const autoware_perception_msgs::msg::PredictedPath * highest_confidence_path(
  const autoware_perception_msgs::msg::PredictedObject & object);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__METRIC_UTILS_HPP_

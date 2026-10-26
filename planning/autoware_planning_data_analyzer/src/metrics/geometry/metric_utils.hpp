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

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <vector>

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

std::string object_id_to_string(const unique_identifier_msgs::msg::UUID & object_id, bool valid);

geometry_msgs::msg::Point to_msg_point(
  const autoware_utils_geometry::Point2d & point, double z = 0.0);

geometry_msgs::msg::Point to_msg_point(const geometry_msgs::msg::Pose & pose);

std::vector<geometry_msgs::msg::Point> polygon_to_points(
  const autoware_utils_geometry::Polygon2d & polygon, double z);

std::vector<std::vector<geometry_msgs::msg::Point>> overlap_polygons_to_points(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const autoware_utils_geometry::Polygon2d & object_polygon, double z);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__METRIC_UTILS_HPP_

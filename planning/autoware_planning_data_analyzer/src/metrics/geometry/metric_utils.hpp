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

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

double get_yaw(const geometry_msgs::msg::Quaternion & orientation);

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint);

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

lanelet::ConstLanelets collect_route_relevant_lanelets(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<RouteHandler> & route_handler);

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line);

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

const autoware_perception_msgs::msg::PredictedPath * highest_confidence_path(
  const autoware_perception_msgs::msg::PredictedObject & object);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__METRIC_UTILS_HPP_

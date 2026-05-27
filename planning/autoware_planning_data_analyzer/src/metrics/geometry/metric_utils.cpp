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

#include "metric_utils.hpp"

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return vehicle_info.vehicle_length_m > 0.0 && vehicle_info.vehicle_width_m > 0.0;
}

double get_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
}

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  // NAVSIM-style rear check: treat an object as behind only when it is more than
  // 150 degrees away from the ego forward axis.
  constexpr double kBehindAngleThresholdRad = 5.0 * M_PI / 6.0;

  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1.0e-6) {
    return false;
  }

  const double cos_angle =
    std::clamp((std::cos(yaw) * dx + std::sin(yaw) * dy) / distance, -1.0, 1.0);
  return std::acos(cos_angle) > kBehindAngleThresholdRad;
}

const autoware_perception_msgs::msg::PredictedPath * highest_confidence_path(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto it = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  return it == object.kinematics.predicted_paths.end() ? nullptr : &(*it);
}

std::string object_id_to_string(
  const unique_identifier_msgs::msg::UUID & object_id, const bool valid)
{
  if (!valid) {
    return "invalid";
  }
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : object_id.uuid) {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

geometry_msgs::msg::Point to_msg_point(
  const autoware_utils_geometry::Point2d & point, const double z)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = z;
  return msg;
}

geometry_msgs::msg::Point to_msg_point(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point msg;
  msg.x = pose.position.x;
  msg.y = pose.position.y;
  msg.z = pose.position.z;
  return msg;
}

std::vector<geometry_msgs::msg::Point> polygon_to_points(
  const autoware_utils_geometry::Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size());
  for (const auto & point : polygon.outer()) {
    points.push_back(to_msg_point(point, z));
  }
  return points;
}

std::vector<std::vector<geometry_msgs::msg::Point>> overlap_polygons_to_points(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const autoware_utils_geometry::Polygon2d & object_polygon, const double z)
{
  std::vector<autoware_utils_geometry::Polygon2d> intersections;
  boost::geometry::intersection(ego_polygon, object_polygon, intersections);
  std::vector<std::vector<geometry_msgs::msg::Point>> polygons;
  polygons.reserve(intersections.size());
  for (const auto & intersection : intersections) {
    if (intersection.outer().size() >= 4U && boost::geometry::area(intersection) > 1.0e-6) {
      polygons.push_back(polygon_to_points(intersection, z));
    }
  }
  return polygons;
}

}  // namespace autoware::planning_data_analyzer::metrics

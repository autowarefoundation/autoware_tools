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

#include "ttc_within_bound.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

constexpr double kStoppedSpeedThreshold = 5.0e-3;
constexpr std::array<double, 4> kFutureProjectionOffsetsSec{{0.0, 0.3, 0.6, 0.9}};

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return vehicle_info.vehicle_length_m > 0.0 && vehicle_info.vehicle_width_m > 0.0;
}

double get_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(orientation);
}

tf2::Vector3 get_velocity_in_world_coordinate(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const double yaw = get_yaw(point.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return tf2::Vector3(
    c * point.longitudinal_velocity_mps - s * point.lateral_velocity_mps,
    s * point.longitudinal_velocity_mps + c * point.lateral_velocity_mps, 0.0);
}

geometry_msgs::msg::Pose project_pose(
  const geometry_msgs::msg::Pose & pose, const tf2::Vector3 & velocity_world, const double delta_t)
{
  auto projected = pose;
  projected.position.x += velocity_world.x() * delta_t;
  projected.position.y += velocity_world.y() * delta_t;
  return projected;
}

Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto local_footprint = vehicle_info.createFootprint(0.0);
  Polygon2d polygon;
  polygon.outer() = autoware_utils_geometry::transform_vector(
    local_footprint, autoware_utils_geometry::pose2transform(pose));
  boost::geometry::correct(polygon);
  return polygon;
}

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady()) {
    return std::nullopt;
  }

  lanelet::ConstLanelet closest_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_lanelet)) {
    return closest_lanelet;
  }

  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      return lanelet;
    }
  }

  return std::nullopt;
}

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  const auto lanelet = find_reference_lanelet(pose, route_handler);
  return lanelet.has_value() &&
         autoware::experimental::lanelet2_utils::is_intersection_lanelet(*lanelet);
}

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

bool is_agent_ahead(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return forward_offset_in_ego_frame(ego_pose, object_pose) > 0.0;
}

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return forward_offset_in_ego_frame(ego_pose, object_pose) < 0.0;
}

const autoware_perception_msgs::msg::PredictedPath * highest_confidence_path(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto it = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  return it == object.kinematics.predicted_paths.end() ? nullptr : &(*it);
}

std::optional<geometry_msgs::msg::Pose> interpolate_object_pose(
  const autoware_perception_msgs::msg::PredictedObject & object, const double query_time_s)
{
  if (query_time_s <= 0.0) {
    return object.kinematics.initial_pose_with_covariance.pose;
  }

  const auto * path = highest_confidence_path(object);
  if (!path || path->path.size() < 2) {
    return std::nullopt;
  }

  const double dt = rclcpp::Duration(path->time_step).seconds();
  if (dt <= 0.0) {
    return std::nullopt;
  }

  const double max_time = dt * static_cast<double>(path->path.size() - 1);
  if (query_time_s > max_time) {
    return std::nullopt;
  }

  const std::size_t index =
    std::min(static_cast<std::size_t>(query_time_s / dt), path->path.size() - 2);
  const double t_i = static_cast<double>(index) * dt;
  const double ratio = std::clamp((query_time_s - t_i) / dt, 0.0, 1.0);
  return autoware_utils_geometry::calc_interpolated_pose(
    path->path.at(index), path->path.at(index + 1), ratio);
}

}  // namespace

TTCWithinBoundResult calculate_ttc_within_bound(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<PredictedObjects> & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  TTCWithinBoundResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (!objects) {
    result.reason = "unavailable_no_objects_message";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  result.available = true;
  result.score = 1.0;
  result.reason = "available";

  if (objects->objects.empty()) {
    return result;
  }

  std::unordered_set<std::size_t> collided_object_indices;
  for (const auto & point : trajectory.points) {
    const auto velocity_world = get_velocity_in_world_coordinate(point);
    const double speed = std::hypot(velocity_world.x(), velocity_world.y());
    if (speed < kStoppedSpeedThreshold) {
      continue;
    }

    const bool ego_in_intersection = is_pose_in_intersection(point.pose, route_handler);

    for (const double future_offset_s : kFutureProjectionOffsetsSec) {
      const double query_time_s =
        rclcpp::Duration(point.time_from_start).seconds() + future_offset_s;
      const auto projected_pose = project_pose(point.pose, velocity_world, future_offset_s);
      const auto ego_polygon = create_pose_footprint(projected_pose, vehicle_info);

      for (std::size_t object_idx = 0; object_idx < objects->objects.size(); ++object_idx) {
        if (collided_object_indices.count(object_idx) != 0U) {
          continue;
        }

        const auto & object = objects->objects.at(object_idx);
        const auto object_pose = interpolate_object_pose(object, query_time_s);
        if (!object_pose.has_value()) {
          continue;
        }

        const auto object_polygon =
          autoware_utils_geometry::to_polygon2d(*object_pose, object.shape);
        if (!boost::geometry::intersects(ego_polygon, object_polygon)) {
          continue;
        }

        if (
          is_agent_ahead(point.pose, *object_pose) ||
          (ego_in_intersection && !is_agent_behind(point.pose, *object_pose))) {
          result.score = 0.0;
          result.reason = "collision_within_bound";
          result.infraction_time_s = rclcpp::Duration(point.time_from_start).seconds();
          return result;
        }

        collided_object_indices.insert(object_idx);
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

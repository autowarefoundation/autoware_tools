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

#include "no_at_fault_collision.hpp"

#include "metrics/geometry/metric_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
namespace bg = boost::geometry;

constexpr double kStoppedSpeedThreshold = 5.0e-2;

enum class CollisionType {
  StoppedEgo,
  StoppedTrack,
  ActiveRear,
  ActiveFront,
  ActiveLateral,
};

struct ObjectState
{
  geometry_msgs::msg::Pose pose;
  double speed_mps{0.0};
  Polygon2d polygon;
};

struct PredictedObjectCache
{
  const autoware_perception_msgs::msg::PredictedObject * object{};
  const autoware_perception_msgs::msg::PredictedPath * path{};
  double dt{0.0};
  double max_time{0.0};
};

struct EgoAreaFlags
{
  bool multiple_lanes{false};
  bool non_drivable_area{false};
};

double ego_speed(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
}

std::vector<PredictedObjectCache> build_object_caches(const PredictedObjects & objects)
{
  std::vector<PredictedObjectCache> caches;
  caches.reserve(objects.objects.size());

  for (const auto & object : objects.objects) {
    PredictedObjectCache cache;
    cache.object = &object;
    cache.path = highest_confidence_path(object);
    if (cache.path && cache.path->path.size() >= 2) {
      cache.dt = rclcpp::Duration(cache.path->time_step).seconds();
      if (cache.dt > 0.0) {
        cache.max_time = cache.dt * static_cast<double>(cache.path->path.size() - 1);
      } else {
        cache.path = nullptr;
        cache.dt = 0.0;
      }
    } else {
      cache.path = nullptr;
    }
    caches.push_back(cache);
  }

  return caches;
}

std::optional<ObjectState> interpolate_object_state(
  const PredictedObjectCache & cache, const double query_time_s)
{
  if (!cache.object) {
    return std::nullopt;
  }
  ObjectState state;

  if (query_time_s <= 0.0) {
    state.pose = cache.object->kinematics.initial_pose_with_covariance.pose;
    const auto & twist = cache.object->kinematics.initial_twist_with_covariance.twist.linear;
    state.speed_mps = std::hypot(twist.x, twist.y);
    state.polygon = autoware_utils_geometry::to_polygon2d(state.pose, cache.object->shape);
    return state;
  }
  if (!cache.path || query_time_s > cache.max_time) {
    return std::nullopt;
  }

  const std::size_t index =
    std::min(static_cast<std::size_t>(query_time_s / cache.dt), cache.path->path.size() - 2);
  const double t_i = static_cast<double>(index) * cache.dt;
  const double ratio = std::clamp((query_time_s - t_i) / cache.dt, 0.0, 1.0);
  state.pose = autoware_utils_geometry::calc_interpolated_pose(
    cache.path->path.at(index), cache.path->path.at(index + 1), ratio);

  const auto & p0 = cache.path->path.at(index).position;
  const auto & p1 = cache.path->path.at(index + 1).position;
  state.speed_mps = std::hypot(p1.x - p0.x, p1.y - p0.y) / cache.dt;
  state.polygon = autoware_utils_geometry::to_polygon2d(state.pose, cache.object->shape);
  return state;
}

bool front_bumper_intersects(
  const geometry_msgs::msg::Pose & ego_pose, const Polygon2d & object_polygon,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto transform = [&](const double x_local, const double y_local) {
    return Point2d{
      ego_pose.position.x + c * x_local - s * y_local,
      ego_pose.position.y + s * x_local + c * y_local};
  };

  bg::model::linestring<Point2d> front_bumper;
  front_bumper.push_back(
    transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m));
  front_bumper.push_back(
    transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m));
  return bg::intersects(front_bumper, object_polygon);
}

CollisionType classify_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point, const ObjectState & object_state,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (ego_speed(ego_point) <= kStoppedSpeedThreshold) {
    return CollisionType::StoppedEgo;
  }
  if (object_state.speed_mps <= kStoppedSpeedThreshold) {
    return CollisionType::StoppedTrack;
  }
  if (is_agent_behind(ego_point.pose, object_state.pose)) {
    return CollisionType::ActiveRear;
  }
  if (front_bumper_intersects(ego_point.pose, object_state.polygon, vehicle_info)) {
    return CollisionType::ActiveFront;
  }
  return CollisionType::ActiveLateral;
}

bool footprint_intersects_lanelet(
  const Polygon2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  return !bg::disjoint(footprint, lanelet.polygon2d().basicPolygon());
}

std::optional<EgoAreaFlags> compute_ego_area_flags(
  const geometry_msgs::msg::Pose & pose, const Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler) {
    return std::nullopt;
  }
  if (!route_handler->isHandlerReady()) {
    return std::nullopt;
  }

  std::size_t intersecting_route_lanelets = 0;
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (!route_handler->isRouteLanelet(lanelet)) {
      continue;
    }
    if (footprint_intersects_lanelet(ego_polygon, lanelet)) {
      ++intersecting_route_lanelets;
    }
  }

  EgoAreaFlags flags;
  flags.non_drivable_area = intersecting_route_lanelets == 0U;
  flags.multiple_lanes = intersecting_route_lanelets > 1U;
  return flags;
}

bool is_agent_type(const autoware_perception_msgs::msg::PredictedObject & object)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  const auto label = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  return autoware::object_recognition_utils::isVehicle(label) ||
         label == ObjectClassification::PEDESTRIAN || label == ObjectClassification::ANIMAL;
}

}  // namespace

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<PredictedObjects> & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  NoAtFaultCollisionResult result;

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

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const auto object_caches = build_object_caches(*objects);

  for (const auto & point : trajectory.points) {
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto ego_polygon = create_pose_footprint(point.pose, local_footprint);

    for (const auto & object_cache : object_caches) {
      const auto object_state = interpolate_object_state(object_cache, query_time_s);
      if (!object_state.has_value()) {
        continue;
      }
      if (!bg::intersects(ego_polygon, object_state->polygon)) {
        continue;
      }

      const auto collision_type = classify_collision(point, *object_state, vehicle_info);

      const bool front_or_stopped_track = collision_type == CollisionType::ActiveFront ||
                                          collision_type == CollisionType::StoppedTrack;
      const bool lateral_collision = collision_type == CollisionType::ActiveLateral;

      if (front_or_stopped_track) {
        result.score = is_agent_type(*object_cache.object) ? 0.0 : 0.5;
        result.reason = is_agent_type(*object_cache.object) ? "at_fault_collision_with_agent"
                                                            : "at_fault_collision_with_non_agent";
        result.infraction_time_s = query_time_s;
        return result;
      }

      if (lateral_collision) {
        const auto ego_area_flags = compute_ego_area_flags(point.pose, ego_polygon, route_handler);
        if (!ego_area_flags.has_value()) {
          result.available = false;
          result.score = 0.0;
          result.reason = !route_handler
                            ? "unavailable_no_route_handler_for_lateral_assessment"
                            : "unavailable_route_handler_not_ready_for_lateral_assessment";
          return result;
        }

        if (ego_area_flags->multiple_lanes || ego_area_flags->non_drivable_area) {
          result.score = is_agent_type(*object_cache.object) ? 0.0 : 0.5;
          result.reason = is_agent_type(*object_cache.object)
                            ? "at_fault_lateral_collision_with_agent"
                            : "at_fault_lateral_collision_with_non_agent";
          result.infraction_time_s = query_time_s;
          return result;
        }
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

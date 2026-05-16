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

#include "metrics/geometry/ego_footprint.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

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

struct EgoAreaFlags
{
  bool multiple_lanes{false};
  bool non_drivable_area{false};
};

double ego_speed(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
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

bool is_track_stopped(const InterpolatedLoggedObject & object_state)
{
  return !is_agent_classification(object_state.classification) ||
         object_state.speed_mps <= kStoppedSpeedThreshold;
}

CollisionType classify_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const InterpolatedLoggedObject & object_state,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (ego_speed(ego_point) <= kStoppedSpeedThreshold) {
    return CollisionType::StoppedEgo;
  }
  if (is_track_stopped(object_state)) {
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
  if (!route_handler || !route_handler->isHandlerReady()) {
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

NoAtFaultCollisionResult make_at_fault_result(
  const InterpolatedLoggedObject & object, const double time_s, const bool lateral_collision)
{
  const bool agent = is_agent_classification(object.classification);
  NoAtFaultCollisionResult result;
  result.available = true;
  result.score = agent ? 0.0 : 0.5;
  result.infraction_time_s = time_s;
  result.reason = lateral_collision ? (agent ? "at_fault_lateral_collision_with_agent"
                                             : "at_fault_lateral_collision_with_non_agent")
                                    : (agent ? "at_fault_collision_with_agent"
                                             : "at_fault_collision_with_non_agent");
  return result;
}

}  // namespace

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<LoggedObjectTrack> * object_tracks)
{
  NoAtFaultCollisionResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (future_objects.empty() && !object_tracks) {
    result.reason = "unavailable_no_future_objects";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  result.available = true;
  result.score = 1.0;
  result.reason = "available";

  const auto local_object_tracks =
    object_tracks ? std::vector<LoggedObjectTrack>{} : build_logged_object_tracks(future_objects);
  const auto & tracks = object_tracks ? *object_tracks : local_object_tracks;
  if (tracks.empty()) {
    return result;
  }

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  std::set<std::array<uint8_t, 16>> collided_object_ids;
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);

  for (const auto & point : trajectory.points) {
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto ego_polygon = create_pose_footprint(point.pose, local_footprint);

    for (const auto & object_track : tracks) {
      if (
        object_track.has_valid_object_id &&
        collided_object_ids.count(object_track.object_id) > 0U) {
        continue;
      }

      const auto object_state = interpolate_logged_object_state(object_track, query_time);
      if (!object_state.has_value() || is_unknown_classification(object_state->classification)) {
        continue;
      }
      if (!bg::intersects(ego_polygon, object_state->polygon)) {
        continue;
      }
      if (object_state->has_valid_object_id) {
        collided_object_ids.insert(object_state->object_id);
      }

      const auto collision_type = classify_collision(point, *object_state, vehicle_info);
      const bool front_or_stopped_track = collision_type == CollisionType::ActiveFront ||
                                          collision_type == CollisionType::StoppedTrack;
      const bool lateral_collision = collision_type == CollisionType::ActiveLateral;

      if (front_or_stopped_track) {
        return make_at_fault_result(*object_state, query_time_s, false);
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
          return make_at_fault_result(*object_state, query_time_s, true);
        }
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

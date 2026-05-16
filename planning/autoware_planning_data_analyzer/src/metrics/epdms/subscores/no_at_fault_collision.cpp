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
#include <array>
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

struct AtFaultCollision
{
  double score{1.0};
  std::string reason{"available"};
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

AtFaultCollision make_at_fault_collision(
  const InterpolatedLoggedObject & object, const bool lateral_collision)
{
  const bool agent = is_agent_classification(object.classification);
  return AtFaultCollision{
    agent ? 0.0 : 0.5, lateral_collision ? (agent ? "at_fault_lateral_collision_with_agent"
                                                  : "at_fault_lateral_collision_with_non_agent")
                                         : (agent ? "at_fault_collision_with_agent"
                                                  : "at_fault_collision_with_non_agent")};
}

}  // namespace

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<TrajectoryFootprintEvaluation> * footprint_evaluations,
  const std::vector<LoggedObjectTrack> * object_tracks)
{
  NoAtFaultCollisionResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (future_objects.empty()) {
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

  const auto local_evaluations =
    footprint_evaluations ? std::vector<TrajectoryFootprintEvaluation>{}
                          : evaluate_trajectory_footprints(trajectory, vehicle_info, route_handler);
  const auto & evaluations = footprint_evaluations ? *footprint_evaluations : local_evaluations;
  if (evaluations.size() != trajectory.points.size()) {
    result.available = false;
    result.reason = "unavailable_invalid_footprint";
    result.score = 0.0;
    return result;
  }

  std::set<std::array<uint8_t, 16>> collided_object_ids;
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);

  for (size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points.at(index);
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto & ego_polygon = evaluations.at(index).ego_polygon;

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
        const auto at_fault_collision = make_at_fault_collision(*object_state, false);
        if (at_fault_collision.score < result.score) {
          result.score = at_fault_collision.score;
          result.reason = at_fault_collision.reason;
          result.infraction_time_s = query_time_s;
        }
        continue;
      }

      if (lateral_collision) {
        const auto & ego_area_evaluation = evaluations.at(index).ego_area_evaluation;
        if (!ego_area_evaluation.has_value()) {
          result.available = false;
          result.score = 0.0;
          result.reason = !route_handler
                            ? "unavailable_no_route_handler_for_lateral_assessment"
                            : "unavailable_route_handler_not_ready_for_lateral_assessment";
          return result;
        }

        if (
          ego_area_evaluation->flags.multiple_lanes ||
          ego_area_evaluation->flags.non_drivable_area) {
          const auto at_fault_collision = make_at_fault_collision(*object_state, true);
          if (at_fault_collision.score < result.score) {
            result.score = at_fault_collision.score;
            result.reason = at_fault_collision.reason;
            result.infraction_time_s = query_time_s;
          }
        }
        continue;
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

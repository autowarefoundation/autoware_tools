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

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
namespace bg = boost::geometry;

// Strict NAVSIM-style physical-stop threshold for NC classification; this is intentionally
// tighter than general perception stopped-vehicle thresholds to keep creeping objects active.
constexpr double kStoppedVelocityThresholdMps = 5.0e-2;

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

struct CollisionClassification
{
  CollisionType type{CollisionType::ActiveLateral};
  bool ego_stopped{false};
  bool track_stopped{false};
  bool behind{false};
  bool front_hit{false};
  std::vector<geometry_msgs::msg::Point> front_bumper;
};

std::string collision_type_to_string(const CollisionType type)
{
  switch (type) {
    case CollisionType::StoppedEgo:
      return "STOPPED_EGO";
    case CollisionType::StoppedTrack:
      return "STOPPED_TRACK";
    case CollisionType::ActiveRear:
      return "ACTIVE_REAR";
    case CollisionType::ActiveFront:
      return "ACTIVE_FRONT";
    case CollisionType::ActiveLateral:
      return "ACTIVE_LATERAL";
  }
  return "NONE";
}

double ego_speed(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
}

std::vector<geometry_msgs::msg::Point> front_bumper_points(
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return {
    autoware_utils_geometry::calc_offset_pose(
      ego_pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m, 0.0, 0.0)
      .position,
    autoware_utils_geometry::calc_offset_pose(
      ego_pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m, 0.0, 0.0)
      .position};
}

bool front_bumper_intersects(
  const std::vector<geometry_msgs::msg::Point> & front_bumper_points,
  const Polygon2d & object_polygon)
{
  if (front_bumper_points.size() < 2U) {
    return false;
  }
  bg::model::linestring<Point2d> front_bumper;
  front_bumper.push_back(Point2d{front_bumper_points.at(0).x, front_bumper_points.at(0).y});
  front_bumper.push_back(Point2d{front_bumper_points.at(1).x, front_bumper_points.at(1).y});
  return bg::intersects(front_bumper, object_polygon);
}

bool is_track_stopped(const InterpolatedLoggedObject & object_state)
{
  return !is_agent_classification(object_state.classification) ||
         object_state.speed_mps <= kStoppedVelocityThresholdMps;
}

CollisionClassification classify_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const InterpolatedLoggedObject & object_state,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  CollisionClassification collision;
  collision.ego_stopped = ego_speed(ego_point) <= kStoppedVelocityThresholdMps;
  collision.track_stopped = is_track_stopped(object_state);
  collision.behind = is_agent_behind(ego_point.pose, object_state.pose);
  collision.front_bumper = front_bumper_points(ego_point.pose, vehicle_info);
  collision.front_hit = front_bumper_intersects(collision.front_bumper, object_state.polygon);
  if (collision.ego_stopped) {
    collision.type = CollisionType::StoppedEgo;
    return collision;
  }
  if (collision.track_stopped) {
    collision.type = CollisionType::StoppedTrack;
    return collision;
  }
  if (collision.behind) {
    collision.type = CollisionType::ActiveRear;
    return collision;
  }
  if (collision.front_hit) {
    collision.type = CollisionType::ActiveFront;
    return collision;
  }
  return collision;
}

AtFaultCollision make_at_fault_collision(
  const InterpolatedLoggedObject & object, const bool lateral_collision)
{
  const bool agent = is_agent_classification(object.classification);
  const double score = agent ? 0.0 : 0.5;
  const std::string collision_scope =
    lateral_collision ? "at_fault_lateral_collision" : "at_fault_collision";
  const std::string object_kind = agent ? "with_agent" : "with_non_agent";
  return AtFaultCollision{score, collision_scope + "_" + object_kind};
}

NoAtFaultCollisionDebugEvent make_debug_event(
  const double query_time_s, const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const Polygon2d & ego_polygon, const InterpolatedLoggedObject & object_state,
  const CollisionClassification & collision)
{
  const double debug_surface_z = ego_point.pose.position.z;
  NoAtFaultCollisionDebugEvent event;
  event.time_s = query_time_s;
  event.object_id = object_id_to_string(object_state.object_id, object_state.has_valid_object_id);
  event.object_label =
    autoware::object_recognition_utils::convertLabelToString(object_state.classification);
  event.collision_type = collision_type_to_string(collision.type);
  event.agent = is_agent_classification(object_state.classification);
  event.ego_stopped = collision.ego_stopped;
  event.track_stopped = collision.track_stopped;
  event.behind = collision.behind;
  event.front_hit = collision.front_hit;
  event.ego_center = to_msg_point(ego_point.pose);
  event.object_center = to_msg_point(object_state.pose);
  event.ego_footprint = polygon_to_points(ego_polygon, debug_surface_z);
  event.object_footprint = polygon_to_points(object_state.polygon, debug_surface_z);
  event.front_bumper = collision.front_bumper;
  return event;
}

void fill_horizon_debug_footprints(
  NoAtFaultCollisionDebugInfo & debug_info,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<LoggedObjectTrack> & object_tracks,
  const std::vector<TrajectoryFootprintEvaluation> & footprint_evaluations)
{
  if (debug_info.events.empty()) {
    return;
  }

  std::unordered_set<std::string> engaged_objects;
  for (const auto & event : debug_info.events) {
    engaged_objects.insert(event.object_id);
  }
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);
  for (size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points.at(index);
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const double query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto & ego_polygon = footprint_evaluations.at(index).ego_polygon;
    bool ego_collision = false;
    bool ego_at_fault = false;
    for (const auto & object_track : object_tracks) {
      const auto object_id =
        object_id_to_string(object_track.object_id, object_track.has_valid_object_id);
      if (engaged_objects.count(object_id) == 0U) {
        continue;
      }
      const auto object_state = interpolate_logged_object_state(object_track, query_time);
      if (!object_state.has_value()) {
        continue;
      }
      const bool intersects = bg::intersects(ego_polygon, object_state->polygon);
      const bool object_at_fault = std::any_of(
        debug_info.events.begin(), debug_info.events.end(),
        [&](const auto & event) { return event.object_id == object_id && event.at_fault; });
      ego_collision = ego_collision || intersects;
      ego_at_fault = ego_at_fault || (intersects && object_at_fault);

      NoAtFaultCollisionHorizonFootprint object_footprint;
      object_footprint.time_s = query_time_s;
      object_footprint.object_id = object_id;
      object_footprint.object_label =
        autoware::object_recognition_utils::convertLabelToString(object_state->classification);
      object_footprint.collision = intersects;
      object_footprint.at_fault = intersects && object_at_fault;
      object_footprint.footprint = polygon_to_points(object_state->polygon, point.pose.position.z);
      debug_info.object_horizon_footprints.push_back(std::move(object_footprint));

      if (intersects) {
        for (const auto & overlap_polygon : overlap_polygons_to_points(
               ego_polygon, object_state->polygon, point.pose.position.z)) {
          NoAtFaultCollisionOverlapArea overlap_area;
          overlap_area.time_s = query_time_s;
          overlap_area.object_id = object_id;
          overlap_area.object_label =
            autoware::object_recognition_utils::convertLabelToString(object_state->classification);
          overlap_area.at_fault = object_at_fault;
          overlap_area.polygon = overlap_polygon;
          debug_info.overlap_areas.push_back(std::move(overlap_area));
        }
      }
    }
    NoAtFaultCollisionHorizonFootprint ego_footprint;
    ego_footprint.time_s = query_time_s;
    ego_footprint.object_id = "ego";
    ego_footprint.object_label = "EGO";
    ego_footprint.collision = ego_collision;
    ego_footprint.at_fault = ego_at_fault;
    ego_footprint.footprint = polygon_to_points(ego_polygon, point.pose.position.z);
    debug_info.ego_horizon_footprints.push_back(std::move(ego_footprint));
  }
}

}  // namespace

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<LoggedObjectTrack> & object_tracks,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<TrajectoryFootprintEvaluation> & footprint_evaluations,
  const bool collect_debug)
{
  NoAtFaultCollisionResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  result.available = true;
  result.score = 1.0;
  result.reason = "available";

  if (object_tracks.empty()) {
    return result;
  }

  if (footprint_evaluations.size() != trajectory.points.size()) {
    result.available = false;
    result.reason = "unavailable_invalid_footprint";
    result.score = 0.0;
    return result;
  }

  std::unordered_set<unique_identifier_msgs::msg::UUID, UuidHash> collided_object_ids;
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);

  auto record_at_fault_collision = [&](
                                     const InterpolatedLoggedObject & object_state,
                                     const AtFaultCollision & at_fault_collision,
                                     const double query_time_s) {
    if (object_state.has_valid_object_id) {
      collided_object_ids.insert(object_state.object_id);
    }
    if (at_fault_collision.score < result.score) {
      result.score = at_fault_collision.score;
      result.reason = at_fault_collision.reason;
      result.infraction_time_s = query_time_s;
    }
  };

  for (size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points.at(index);
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto & ego_polygon = footprint_evaluations.at(index).ego_polygon;

    for (const auto & object_track : object_tracks) {
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

      const auto collision = classify_collision(point, *object_state, vehicle_info);
      auto debug_event =
        make_debug_event(query_time_s, point, ego_polygon, *object_state, collision);
      const bool front_or_stopped_track = collision.type == CollisionType::ActiveFront ||
                                          collision.type == CollisionType::StoppedTrack;
      const bool lateral_collision = collision.type == CollisionType::ActiveLateral;

      if (front_or_stopped_track) {
        const auto at_fault_collision = make_at_fault_collision(*object_state, false);
        debug_event.at_fault = true;
        debug_event.event_score = at_fault_collision.score;
        debug_event.reason = at_fault_collision.reason;
        result.debug_info.events.push_back(debug_event);
        record_at_fault_collision(*object_state, at_fault_collision, query_time_s);
        continue;
      }

      if (lateral_collision) {
        const auto & ego_area_evaluation = footprint_evaluations.at(index).ego_area_evaluation;
        if (!ego_area_evaluation.has_value()) {
          debug_event.reason = !route_handler
                                 ? "unavailable_no_route_handler_for_lateral_assessment"
                                 : "unavailable_route_handler_not_ready_for_lateral_assessment";
          result.debug_info.events.push_back(debug_event);
          result.available = false;
          result.score = 0.0;
          result.reason = !route_handler
                            ? "unavailable_no_route_handler_for_lateral_assessment"
                            : "unavailable_route_handler_not_ready_for_lateral_assessment";
          return result;
        }

        debug_event.multiple_lanes = ego_area_evaluation->flags.multiple_lanes;
        debug_event.non_drivable_area = ego_area_evaluation->flags.non_drivable_area;
        if (
          ego_area_evaluation->flags.multiple_lanes ||
          ego_area_evaluation->flags.non_drivable_area) {
          const auto at_fault_collision = make_at_fault_collision(*object_state, true);
          debug_event.at_fault = true;
          debug_event.event_score = at_fault_collision.score;
          debug_event.reason = at_fault_collision.reason;
          record_at_fault_collision(*object_state, at_fault_collision, query_time_s);
        }
        result.debug_info.events.push_back(debug_event);
        continue;
      }

      result.debug_info.events.push_back(debug_event);
    }
  }

  if (collect_debug) {
    fill_horizon_debug_footprints(
      result.debug_info, trajectory, object_tracks, footprint_evaluations);
  }
  return result;
}

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler, const bool collect_debug)
{
  if (future_objects.empty()) {
    NoAtFaultCollisionResult result;
    result.reason = "unavailable_no_future_objects";
    return result;
  }
  const auto object_tracks = build_logged_object_tracks(future_objects);
  const auto footprint_evaluations =
    evaluate_trajectory_footprints(trajectory, vehicle_info, route_handler);
  return calculate_no_at_fault_collision(
    trajectory, object_tracks, vehicle_info, route_handler, footprint_evaluations, collect_debug);
}

}  // namespace autoware::planning_data_analyzer::metrics

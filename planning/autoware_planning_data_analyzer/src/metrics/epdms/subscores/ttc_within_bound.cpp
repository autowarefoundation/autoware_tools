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

#include "metrics/geometry/ego_footprint.hpp"
#include "metrics/geometry/lanelet_queries.hpp"
#include "metrics/geometry/metric_utils.hpp"
#include "metrics/geometry/object_tracks.hpp"

#include <tf2/LinearMath/Vector3.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

constexpr double kStoppedSpeedThreshold = 5.0e-3;
constexpr std::array<double, 4> kFutureProjectionOffsetsSec{{0.0, 0.3, 0.6, 0.9}};
constexpr double kAheadAngleThresholdRad = M_PI / 6.0;
constexpr double kBehindAngleThresholdRad = 5.0 * M_PI / 6.0;

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

double relative_agent_angle(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1.0e-6) {
    return std::numeric_limits<double>::infinity();
  }

  const double cos_angle =
    std::clamp((std::cos(yaw) * dx + std::sin(yaw) * dy) / distance, -1.0, 1.0);
  return std::acos(cos_angle);
}

bool is_agent_ahead_nuplan(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return relative_agent_angle(ego_pose, object_pose) < kAheadAngleThresholdRad;
}

bool is_agent_behind_nuplan(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return relative_agent_angle(ego_pose, object_pose) > kBehindAngleThresholdRad;
}

}  // namespace

TTCWithinBoundResult calculate_ttc_within_bound(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<TrajectoryFootprintEvaluation> * footprint_evaluations,
  const std::vector<LoggedObjectTrack> * object_tracks)
{
  TTCWithinBoundResult result;

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

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const auto local_evaluations =
    footprint_evaluations
      ? std::vector<TrajectoryFootprintEvaluation>{}
      : evaluate_trajectory_footprints(trajectory, vehicle_info, route_handler, nullptr, true);
  const auto & evaluations = footprint_evaluations ? *footprint_evaluations : local_evaluations;
  if (!evaluations.empty() && evaluations.size() != trajectory.points.size()) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_invalid_footprint";
    return result;
  }

  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);
  std::unordered_set<unique_identifier_msgs::msg::UUID, UuidHash> collided_object_ids;

  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points.at(index);
    const auto velocity_world = get_velocity_in_world_coordinate(point);
    const double speed = std::hypot(velocity_world.x(), velocity_world.y());
    if (speed < kStoppedSpeedThreshold) {
      continue;
    }

    bool multiple_lanes = false;
    bool non_drivable_area = false;
    if (
      !evaluations.empty() && index < evaluations.size() &&
      evaluations.at(index).ego_area_evaluation.has_value()) {
      const auto & flags = evaluations.at(index).ego_area_evaluation->flags;
      multiple_lanes = flags.multiple_lanes;
      non_drivable_area = flags.non_drivable_area;
    }
    const bool ego_in_intersection =
      !evaluations.empty() && index < evaluations.size() &&
          evaluations.at(index).ego_area_evaluation.has_value() &&
          evaluations.at(index).ego_area_evaluation->flags.intersection_context_available
        ? evaluations.at(index).ego_area_evaluation->flags.in_intersection
        : is_pose_in_intersection(point.pose, route_handler);
    const bool bad_or_intersection = multiple_lanes || non_drivable_area || ego_in_intersection;
    const double time_s = rclcpp::Duration(point.time_from_start).seconds();

    for (const double future_offset_s : kFutureProjectionOffsetsSec) {
      const double query_time_s = time_s + future_offset_s;
      const auto query_time = trajectory_start_time + rclcpp::Duration::from_seconds(query_time_s);
      const auto projected_pose = project_pose(point.pose, velocity_world, future_offset_s);
      const auto ego_polygon = create_pose_footprint(projected_pose, local_footprint);

      for (const auto & object_track : tracks) {
        if (
          object_track.has_valid_object_id &&
          collided_object_ids.count(object_track.object_id) > 0U) {
          continue;
        }

        const auto object_state = interpolate_logged_object_state(object_track, query_time);
        if (!object_state.has_value()) {
          continue;
        }
        if (is_unknown_classification(object_state->classification)) {
          continue;
        }

        if (!boost::geometry::intersects(ego_polygon, object_state->polygon)) {
          continue;
        }

        if (object_state->has_valid_object_id) {
          collided_object_ids.insert(object_state->object_id);
        }

        const bool ahead = is_agent_ahead_nuplan(projected_pose, object_state->pose);
        const bool behind = is_agent_behind_nuplan(projected_pose, object_state->pose);
        if (ahead || (bad_or_intersection && !behind)) {
          result.score = 0.0;
          result.reason = "collision_within_bound";
          result.infraction_time_s = query_time_s;
          return result;
        }
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

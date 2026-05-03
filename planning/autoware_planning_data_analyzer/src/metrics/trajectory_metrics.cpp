// Copyright 2025 TIER IV, Inc.
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

#include "trajectory_metrics.hpp"

#include "metrics/epdms/subscores/drivable_area_compliance.hpp"
#include "metrics/epdms/subscores/driving_direction_compliance.hpp"
#include "metrics/epdms/subscores/history_comfort.hpp"
#include "metrics/epdms/subscores/no_at_fault_collision.hpp"
#include "metrics/epdms/subscores/traffic_light_compliance.hpp"
#include "metrics/epdms/subscores/ttc_within_bound.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

/**
 * @brief Get velocity in world coordinate frame from trajectory point
 * Reference: autoware_trajectory_ranker/src/metrics/metrics_utils.cpp
 * @param point Trajectory point with velocities in vehicle frame
 * @return Velocity vector in world frame
 */
tf2::Vector3 get_velocity_in_world_coordinate(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const auto & pose = point.pose;
  const double yaw = get_yaw(pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Rotate velocity from vehicle frame to world frame
  const double vx_world =
    cos_yaw * point.longitudinal_velocity_mps - sin_yaw * point.lateral_velocity_mps;
  const double vy_world =
    sin_yaw * point.longitudinal_velocity_mps + cos_yaw * point.lateral_velocity_mps;

  return tf2::Vector3(vx_world, vy_world, 0.0);
}

/**
 * @brief Calculate time to collision between two trajectory points
 * Reference: autoware_trajectory_ranker/src/metrics/metrics_utils.cpp
 * @param point1 First trajectory point
 * @param point2 Second trajectory point
 * @return Time to collision [s], or infinity if no collision
 */
double calculate_ttc_between_points(
  const autoware_planning_msgs::msg::TrajectoryPoint & point1,
  const autoware_planning_msgs::msg::TrajectoryPoint & point2)
{
  constexpr double eps = 1e-6;

  // Calculate displacement vector
  const auto & pos1 = point1.pose.position;
  const auto & pos2 = point2.pose.position;
  const tf2::Vector3 displacement(pos2.x - pos1.x, pos2.y - pos1.y, 0.0);
  const double distance = displacement.length();

  if (distance < eps) {
    return 0.0;
  }

  const auto dir = displacement.normalized();

  // Get velocities in world frame
  const auto v1 = get_velocity_in_world_coordinate(point1);
  const auto v2 = get_velocity_in_world_coordinate(point2);

  // Calculate relative velocity along displacement direction
  const double relative_velocity = tf2::tf2Dot(dir, v1) - tf2::tf2Dot(dir, v2);

  if (std::abs(relative_velocity) < eps) {
    return std::numeric_limits<double>::max();
  }

  return distance / relative_velocity;
}

/**
 * @brief Calculate time to collision with a predicted object at specific time
 * Reference: autoware_trajectory_ranker/src/metrics/metrics_utils.cpp
 * @param ego_point Ego trajectory point
 * @param duration Time offset for object prediction
 * @param object Predicted object with future paths
 * @param max_ttc_value Maximum TTC value to return [s]
 * @return Time to collision [s], capped at max_ttc_value
 */
double calculate_time_to_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point, const rclcpp::Duration & duration,
  const autoware_perception_msgs::msg::PredictedObject & object, const double max_ttc_value)
{
  // Find the predicted path with highest confidence
  const auto max_confidence_path = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

  if (max_confidence_path == object.kinematics.predicted_paths.end()) {
    return max_ttc_value;
  }

  const auto & object_path = max_confidence_path->path;

  // Handle case with no predicted path
  if (object_path.size() < 2) {
    if (duration.seconds() == 0.0) {
      autoware_planning_msgs::msg::TrajectoryPoint object_point;
      object_point.pose = object.kinematics.initial_pose_with_covariance.pose;
      object_point.longitudinal_velocity_mps =
        object.kinematics.initial_twist_with_covariance.twist.linear.x;
      object_point.lateral_velocity_mps =
        object.kinematics.initial_twist_with_covariance.twist.linear.y;
      double ttc = calculate_ttc_between_points(ego_point, object_point);
      return std::min(ttc, max_ttc_value);
    }
    return max_ttc_value;
  }

  const double dt = rclcpp::Duration(max_confidence_path->time_step).seconds();
  if (dt <= 0.0) {
    return max_ttc_value;
  }

  const double max_time = dt * static_cast<double>(object_path.size() - 1);
  const double query_time = duration.seconds();

  if (query_time < 0.0 || query_time > max_time) {
    return max_ttc_value;
  }

  // Interpolate object position at query time
  const size_t nearest_index =
    std::min(static_cast<size_t>(query_time / dt), object_path.size() - 2);
  const double t_i = static_cast<double>(nearest_index) * dt;
  const double ratio = std::clamp((query_time - t_i) / dt, 0.0, 1.0);

  const auto object_pose = autoware_utils_geometry::calc_interpolated_pose(
    object_path.at(nearest_index), object_path.at(nearest_index + 1), ratio);

  // Calculate object velocity from path segment
  const auto & pos_i = object_path.at(nearest_index).position;
  const auto & pos_i1 = object_path.at(nearest_index + 1).position;
  const tf2::Vector3 segment(pos_i1.x - pos_i.x, pos_i1.y - pos_i.y, 0.0);
  const double segment_length = segment.length();

  autoware_planning_msgs::msg::TrajectoryPoint obj_point;
  obj_point.pose = object_pose;

  if (segment_length > 1e-6) {
    const auto dir_w = segment / segment_length;
    const double v = segment_length / dt;

    // Transform velocity from world frame to vehicle frame
    const double yaw = get_yaw(obj_point.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double vx_w = dir_w.x() * v;
    const double vy_w = dir_w.y() * v;

    obj_point.longitudinal_velocity_mps = c * vx_w + s * vy_w;
    obj_point.lateral_velocity_mps = -s * vx_w + c * vy_w;
  } else {
    obj_point.longitudinal_velocity_mps = 0.0;
    obj_point.lateral_velocity_mps = 0.0;
  }

  const double ttc = calculate_ttc_between_points(ego_point, obj_point);
  return std::min(ttc, max_ttc_value);
}

bool is_pose_in_route_lane(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady()) {
    return false;
  }

  lanelet::ConstLanelet closest_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_lanelet)) {
    return true;
  }

  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      return true;
    }
  }

  return false;
}

}  // namespace

TrajectoryPointMetrics calculate_trajectory_point_metrics(
  const std::shared_ptr<SynchronizedData> & sync_data,
  const std::shared_ptr<RouteHandler> & route_handler,
  const HistoryComfortParameters & history_comfort_params,
  const LaneKeepingParameters & lane_keeping_params,
  const DrivingDirectionComplianceParameters & driving_direction_params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  TrajectoryPointMetrics metrics;

  if (!sync_data || !sync_data->trajectory) {
    return metrics;
  }

  const auto & trajectory = *sync_data->trajectory;
  const size_t num_points = trajectory.points.size();

  // Initialize vectors
  metrics.ttc_values.resize(num_points, std::numeric_limits<double>::max());
  metrics.lateral_deviations.resize(num_points, 0.0);
  metrics.travel_distances.resize(num_points, 0.0);

  if (num_points == 0U) {
    return metrics;
  }

  calculate_history_comfort_metrics(trajectory, history_comfort_params, metrics);

  const auto ttc_within_bound =
    calculate_ttc_within_bound(trajectory, sync_data->objects, vehicle_info, route_handler);
  metrics.time_to_collision_within_bound = ttc_within_bound.score;
  metrics.time_to_collision_within_bound_available = ttc_within_bound.available;
  metrics.time_to_collision_within_bound_reason = ttc_within_bound.reason;
  metrics.time_to_collision_infraction_time_s = ttc_within_bound.infraction_time_s;
  const auto no_at_fault_collision =
    calculate_no_at_fault_collision(trajectory, sync_data->objects, vehicle_info, route_handler);
  metrics.no_at_fault_collision = no_at_fault_collision.score;
  metrics.no_at_fault_collision_available = no_at_fault_collision.available;
  metrics.no_at_fault_collision_reason = no_at_fault_collision.reason;
  metrics.time_to_at_fault_collision_s = no_at_fault_collision.infraction_time_s;
  if (!route_handler) {
    metrics.driving_direction_compliance_reason = "unavailable_no_route_handler";
  } else if (!route_handler->isHandlerReady()) {
    metrics.driving_direction_compliance_reason = "unavailable_route_handler_not_ready";
  } else {
    std::vector<DrivingDirectionEvaluationPoint> driving_direction_evaluation_points;
    driving_direction_evaluation_points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      double progress_m = 0.0;
      if (i > 0) {
        progress_m = autoware_utils_geometry::calc_distance2d(
          trajectory.points.at(i - 1).pose.position, trajectory.points.at(i).pose.position);
      }
      const auto & point = trajectory.points.at(i);
      driving_direction_evaluation_points.push_back(
        DrivingDirectionEvaluationPoint{
          rclcpp::Duration(point.time_from_start).seconds(), progress_m,
          !is_pose_in_route_lane(point.pose, route_handler),
          is_pose_in_intersection(point.pose, route_handler)});
    }

    const auto ddc_result = calculate_driving_direction_compliance(
      driving_direction_evaluation_points, driving_direction_params);
    metrics.driving_direction_compliance = ddc_result.score;
    metrics.driving_direction_compliance_available = ddc_result.available;
    metrics.driving_direction_compliance_reason = ddc_result.reason;
    metrics.max_oncoming_progress_m = ddc_result.max_oncoming_progress_m;
  }

  if (!route_handler) {
    metrics.drivable_area_compliance_reason = "unavailable_no_route_handler";
    metrics.traffic_light_compliance_reason = "unavailable_no_route_handler";
  } else if (!route_handler->isHandlerReady()) {
    metrics.drivable_area_compliance_reason = "unavailable_route_handler_not_ready";
    metrics.traffic_light_compliance_reason = "unavailable_route_handler_not_ready";
  } else {
    const auto drivable_lanelets = collect_route_relevant_lanelets(trajectory, route_handler);
    const auto drivable_area_compliance =
      calculate_drivable_area_compliance(trajectory, drivable_lanelets, vehicle_info);
    metrics.drivable_area_compliance = drivable_area_compliance.score;
    metrics.drivable_area_compliance_available = drivable_area_compliance.available;
    metrics.drivable_area_compliance_reason = drivable_area_compliance.reason;

    const auto traffic_light_compliance = calculate_traffic_light_compliance(
      trajectory, sync_data->traffic_signals, route_handler, vehicle_info);
    metrics.traffic_light_compliance = traffic_light_compliance.score;
    metrics.traffic_light_compliance_available = traffic_light_compliance.available;
    metrics.traffic_light_compliance_reason = traffic_light_compliance.reason;
  }

  // Calculate TTC for each point (based on autoware_trajectory_ranker implementation)
  constexpr double max_ttc_value = 10.0;  // Maximum TTC value in seconds
  if (sync_data->objects) {
    for (size_t i = 0; i < num_points; ++i) {
      double min_ttc = std::numeric_limits<double>::max();

      const auto & ego_point = trajectory.points[i];
      const auto time = ego_point.time_from_start;

      // Check TTC with all objects
      for (const auto & object : sync_data->objects->objects) {
        const double ttc = calculate_time_to_collision(ego_point, time, object, max_ttc_value);
        if (std::isfinite(ttc) && ttc >= 0.0) {
          min_ttc = std::min(min_ttc, ttc);
        }
      }

      if (!std::isfinite(min_ttc)) {
        min_ttc = max_ttc_value;
      }
      metrics.ttc_values[i] = std::min(min_ttc, max_ttc_value);
    }
  }

  std::vector<LaneKeepingEvaluationPoint> lane_keeping_evaluation_points;
  lane_keeping_evaluation_points.reserve(num_points);

  // Calculate lateral deviation from the local route lane at each pose.
  if (!route_handler) {
    metrics.lane_keeping_reason = "unavailable_no_route_handler";
  } else if (!route_handler->isHandlerReady()) {
    metrics.lane_keeping_reason = "unavailable_route_handler_not_ready";
  } else {
    for (size_t i = 0; i < num_points; ++i) {
      const auto & point = trajectory.points[i];
      const auto reference_lanelet = find_reference_lanelet(point.pose, route_handler);
      if (!reference_lanelet.has_value()) {
        metrics.lateral_deviations[i] = std::numeric_limits<double>::quiet_NaN();
        lane_keeping_evaluation_points.push_back(
          LaneKeepingEvaluationPoint{point.time_from_start, metrics.lateral_deviations[i], false});
      } else {
        metrics.lateral_deviations[i] =
          autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
            reference_lanelet.value(), point.pose);
        lane_keeping_evaluation_points.push_back(
          LaneKeepingEvaluationPoint{
            point.time_from_start, metrics.lateral_deviations[i],
            autoware::experimental::lanelet2_utils::is_intersection_lanelet(
              reference_lanelet.value())});
      }
    }
  }
  const auto has_finite_lane_keeping_sample = std::any_of(
    lane_keeping_evaluation_points.begin(), lane_keeping_evaluation_points.end(),
    [](const auto & evaluation_point) {
      return std::isfinite(evaluation_point.lateral_deviation);
    });
  if (has_finite_lane_keeping_sample) {
    metrics.lane_keeping =
      calculate_lane_keeping_score(lane_keeping_evaluation_points, lane_keeping_params);
    metrics.lane_keeping_available = true;
    metrics.lane_keeping_reason = "available";
  } else if (metrics.lane_keeping_reason == "unavailable") {
    metrics.lane_keeping_reason = "unavailable_no_reference_lanelet";
  }

  // Calculate travel distances using motion_utils
  for (size_t i = 0; i < num_points; ++i) {
    metrics.travel_distances[i] =
      autoware::motion_utils::calcSignedArcLength(trajectory.points, 0, i);
  }

  return metrics;
}

}  // namespace autoware::planning_data_analyzer::metrics

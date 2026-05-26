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

#include "epdms_debug_writer.hpp"

#include "metrics/geometry/ego_footprint.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/duration.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kDebugMarkerLifetimeS = 0.2;

std::string debug_topic(const std::string & subscore, const std::string & name)
{
  return "/debug/epdms/" + subscore + "/" + name;
}

std::string trajectory_debug_topic(const std::string & name)
{
  return debug_topic("trajectory", name);
}

std_msgs::msg::ColorRGBA make_color(
  const float red, const float green, const float blue, const float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

double point_time_s(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

double lerp(const double lhs, const double rhs, const double ratio)
{
  return lhs + (rhs - lhs) * ratio;
}

autoware_planning_msgs::msg::TrajectoryPoint interpolate_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & previous,
  const autoware_planning_msgs::msg::TrajectoryPoint & next, const double horizon_s)
{
  const double previous_t = point_time_s(previous);
  const double next_t = point_time_s(next);
  const double ratio = next_t > previous_t
                         ? std::clamp((horizon_s - previous_t) / (next_t - previous_t), 0.0, 1.0)
                         : 0.0;

  auto interpolated = previous;
  interpolated.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio);
  interpolated.longitudinal_velocity_mps =
    lerp(previous.longitudinal_velocity_mps, next.longitudinal_velocity_mps, ratio);
  interpolated.lateral_velocity_mps =
    lerp(previous.lateral_velocity_mps, next.lateral_velocity_mps, ratio);
  interpolated.acceleration_mps2 = lerp(previous.acceleration_mps2, next.acceleration_mps2, ratio);
  interpolated.heading_rate_rps = lerp(previous.heading_rate_rps, next.heading_rate_rps, ratio);
  interpolated.front_wheel_angle_rad =
    lerp(previous.front_wheel_angle_rad, next.front_wheel_angle_rad, ratio);
  interpolated.rear_wheel_angle_rad =
    lerp(previous.rear_wheel_angle_rad, next.rear_wheel_angle_rad, ratio);
  interpolated.time_from_start = rclcpp::Duration::from_seconds(horizon_s);
  return interpolated;
}

autoware_planning_msgs::msg::Trajectory truncate_trajectory_by_horizon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double horizon_s)
{
  constexpr double kTimeEpsilon = 1.0e-6;
  if (horizon_s <= 0.0 || trajectory.points.empty()) {
    return trajectory;
  }

  autoware_planning_msgs::msg::Trajectory truncated;
  truncated.header = trajectory.header;
  truncated.points.reserve(trajectory.points.size());

  for (const auto & point : trajectory.points) {
    const double t = point_time_s(point);
    if (t <= horizon_s + kTimeEpsilon) {
      truncated.points.push_back(point);
      continue;
    }

    if (!truncated.points.empty()) {
      const double previous_t = point_time_s(truncated.points.back());
      if (previous_t < horizon_s - kTimeEpsilon) {
        truncated.points.push_back(
          interpolate_trajectory_point(truncated.points.back(), point, horizon_s));
      }
    }
    if (truncated.points.empty()) {
      truncated.points.push_back(point);
    }
    return truncated;
  }

  return truncated;
}

Marker make_delete_all_marker(const rclcpp::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.action = Marker::DELETEALL;
  return marker;
}

Marker make_marker_base(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  const std_msgs::msg::ColorRGBA & marker_color, const double lifetime_s)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color = marker_color;
  marker.lifetime = rclcpp::Duration::from_seconds(lifetime_s);
  return marker;
}

std::vector<geometry_msgs::msg::Point> closed_line_strip_points(
  std::vector<geometry_msgs::msg::Point> points)
{
  if (points.size() < 2U) {
    return points;
  }
  const auto & first = points.front();
  const auto & last = points.back();
  if (first.x != last.x || first.y != last.y || first.z != last.z) {
    points.push_back(first);
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> lift_points(
  std::vector<geometry_msgs::msg::Point> points, const double z_offset)
{
  for (auto & point : points) {
    point.z += z_offset;
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> square_marker_points(
  const geometry_msgs::msg::Point & center, const double half_extent)
{
  geometry_msgs::msg::Point p0 = center;
  geometry_msgs::msg::Point p1 = center;
  geometry_msgs::msg::Point p2 = center;
  geometry_msgs::msg::Point p3 = center;
  p0.x -= half_extent;
  p0.y -= half_extent;
  p1.x += half_extent;
  p1.y -= half_extent;
  p2.x += half_extent;
  p2.y += half_extent;
  p3.x -= half_extent;
  p3.y += half_extent;
  return {p0, p1, p2, p3, p0};
}

std::vector<geometry_msgs::msg::Point> polygon_to_msg_points(
  const autoware_utils_geometry::Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size() + 1U);
  for (const auto & point : polygon.outer()) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  if (
    !points.empty() &&
    (points.front().x != points.back().x || points.front().y != points.back().y)) {
    points.push_back(points.front());
  }
  return points;
}

Marker make_line_strip_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  std::vector<geometry_msgs::msg::Point> points, const std_msgs::msg::ColorRGBA & marker_color,
  const double width, const bool close_line, const double lifetime_s, const double z_offset = 0.0)
{
  auto marker = make_marker_base(stamp, ns, id, marker_color, lifetime_s);
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = width;
  marker.points = close_line ? closed_line_strip_points(std::move(points)) : std::move(points);
  marker.points = lift_points(std::move(marker.points), z_offset);
  return marker;
}

Marker make_filled_polygon_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  std::vector<geometry_msgs::msg::Point> polygon_points,
  const std_msgs::msg::ColorRGBA & marker_color, const double lifetime_s,
  const double z_offset = 0.0)
{
  auto marker = make_marker_base(stamp, ns, id, marker_color, lifetime_s);
  marker.type = Marker::TRIANGLE_LIST;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  if (polygon_points.size() >= 2U) {
    const auto & first = polygon_points.front();
    const auto & last = polygon_points.back();
    if (first.x == last.x && first.y == last.y && first.z == last.z) {
      polygon_points.pop_back();
    }
  }
  if (polygon_points.size() < 3U) {
    return marker;
  }
  marker.points.reserve((polygon_points.size() - 2U) * 3U);
  for (std::size_t index = 1; index + 1U < polygon_points.size(); ++index) {
    marker.points.push_back(polygon_points.front());
    marker.points.push_back(polygon_points.at(index));
    marker.points.push_back(polygon_points.at(index + 1U));
  }
  marker.points = lift_points(std::move(marker.points), z_offset);
  return marker;
}

void write_string(
  const nlohmann::json & json, const std::string & topic, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & timestamp)
{
  std_msgs::msg::String msg;
  msg.data = json.dump();
  bag_writer.write(msg, topic, timestamp);
}

void write_array(
  const std::vector<double> & values, const std::string & topic, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & timestamp)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = values;
  bag_writer.write(msg, topic, timestamp);
}

void add_marker_topic(
  std::vector<std::pair<std::string, std::string>> & topics, const std::string & topic)
{
  topics.emplace_back(topic, "visualization_msgs/msg/MarkerArray");
}

void add_string_topic(
  std::vector<std::pair<std::string, std::string>> & topics, const std::string & topic)
{
  topics.emplace_back(topic, "std_msgs/msg/String");
}

void add_array_topic(
  std::vector<std::pair<std::string, std::string>> & topics, const std::string & topic)
{
  topics.emplace_back(topic, "std_msgs/msg/Float64MultiArray");
}

const NoAtFaultCollisionDebugEvent * find_worst_nc_event(
  const NoAtFaultCollisionDebugInfo & debug_info)
{
  const NoAtFaultCollisionDebugEvent * worst_event = nullptr;
  for (const auto & event : debug_info.events) {
    if (!worst_event || event.event_score < worst_event->event_score) {
      worst_event = &event;
      continue;
    }
    if (event.event_score == worst_event->event_score && event.at_fault && !worst_event->at_fault) {
      worst_event = &event;
    }
  }
  return worst_event;
}

nlohmann::json nc_debug_event_to_json(const NoAtFaultCollisionDebugEvent & event)
{
  return nlohmann::json{
    {"time_s", event.time_s},
    {"object_id", event.object_id},
    {"object_label", event.object_label},
    {"collision_type", event.collision_type},
    {"reason", event.reason},
    {"agent", event.agent},
    {"at_fault", event.at_fault},
    {"score", event.event_score},
    {"ego_stopped", event.ego_stopped},
    {"track_stopped", event.track_stopped},
    {"behind", event.behind},
    {"front_hit", event.front_hit},
    {"multiple_lanes", event.multiple_lanes},
    {"non_drivable_area", event.non_drivable_area}};
}

nlohmann::json nc_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const NoAtFaultCollisionDebugInfo & debug_info,
  const NoAtFaultCollisionDebugEvent * worst_event, const rclcpp::Time & timestamp)
{
  nlohmann::json objects = nlohmann::json::array();
  std::set<std::string> seen_objects;
  for (const auto & event : debug_info.events) {
    if (!seen_objects.insert(event.object_id).second) {
      continue;
    }
    objects.push_back(
      {{"object_id", event.object_id},
       {"label", event.object_label},
       {"collision_type", event.collision_type},
       {"first_collision_time_s", event.time_s}});
  }

  nlohmann::json events = nlohmann::json::array();
  for (const auto & event : debug_info.events) {
    auto event_json = nc_debug_event_to_json(event);
    event_json["trajectory_stamp_sec"] = timestamp.seconds();
    event_json["event_stamp_sec"] = timestamp.seconds() + event.time_s;
    events.push_back(std::move(event_json));
  }

  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.no_at_fault_collision},
    {"reason", metrics.no_at_fault_collision_reason},
    {"worst_time_s", metrics.time_to_at_fault_collision_s},
    {"worst_event_stamp_sec", std::isfinite(metrics.time_to_at_fault_collision_s)
                                ? timestamp.seconds() + metrics.time_to_at_fault_collision_s
                                : std::numeric_limits<double>::quiet_NaN()},
    {"event_count", debug_info.events.size()},
    {"at_fault_event_count", std::count_if(
                               debug_info.events.begin(), debug_info.events.end(),
                               [](const auto & event) { return event.at_fault; })},
    {"worst_object_id", worst_event ? worst_event->object_id : "invalid"},
    {"worst_object_label", worst_event ? worst_event->object_label : "UNKNOWN"},
    {"worst_collision_type", worst_event ? worst_event->collision_type : "NONE"},
    {"objects", std::move(objects)},
    {"events", std::move(events)}};
}

nlohmann::json dac_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const DrivableAreaComplianceDebugInfo & debug_info,
  const rclcpp::Time & timestamp)
{
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.drivable_area_compliance},
    {"reason", metrics.drivable_area_compliance_reason},
    {"first_failure_time_s", debug_info.first_failure_time_s},
    {"failure_stamp_sec", std::isfinite(debug_info.first_failure_time_s)
                            ? timestamp.seconds() + debug_info.first_failure_time_s
                            : std::numeric_limits<double>::quiet_NaN()},
    {"failing_corner_indices", debug_info.failing_corner_indices},
    {"corner_count_inside", debug_info.corner_count_inside},
    {"route_candidate_count", debug_info.route_candidate_count},
    {"road_candidate_count", debug_info.road_candidate_count},
    {"shoulder_candidate_count", debug_info.shoulder_candidate_count},
    {"intersection_candidate_count", debug_info.intersection_candidate_count},
    {"hatched_road_marking_candidate_count", debug_info.hatched_road_marking_candidate_count},
    {"road_border_line_count", debug_info.road_border_line_count},
    {"road_border_fallback_used", debug_info.road_border_fallback_used},
    {"road_border_side_test_count", debug_info.road_border_side_test_count},
    {"road_border_side_accept_count", debug_info.road_border_side_accept_count},
    {"parking_candidate_count", debug_info.parking_candidate_count}};
}

nlohmann::json ddc_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const DrivingDirectionComplianceDebugInfo & debug_info,
  const rclcpp::Time & timestamp)
{
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.driving_direction_compliance},
    {"reason", metrics.driving_direction_compliance_reason},
    {"max_oncoming_progress_m", metrics.max_oncoming_progress_m},
    {"worst_window_start_s", debug_info.worst_window_start_time_s},
    {"worst_window_end_s", debug_info.worst_window_end_time_s},
    {"window_progress_m", debug_info.window_progress_m},
    {"sample_count", debug_info.worst_window_sample_count}};
}

nlohmann::json tlc_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const TrafficLightComplianceDebugInfo & debug_info,
  const rclcpp::Time & timestamp)
{
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.traffic_light_compliance},
    {"reason", metrics.traffic_light_compliance_reason},
    {"first_failure_time_s", debug_info.first_failure_time_s},
    {"failure_stamp_sec", std::isfinite(debug_info.first_failure_time_s)
                            ? timestamp.seconds() + debug_info.first_failure_time_s
                            : std::numeric_limits<double>::quiet_NaN()},
    {"intended_movement",
     debug_info.intended_movement.has_value() ? *debug_info.intended_movement : "unknown"},
    {"regulatory_element_ids", debug_info.regulatory_element_ids},
    {"selected_lane_ids", debug_info.selected_lane_ids},
    {"stop_line_ids", debug_info.stop_line_ids},
    {"selected_stop_line_count", debug_info.selected_stop_line_count}};
}

std::string ttc_area_condition_string(const TTCWithinBoundDebugEvent & event)
{
  if (event.ahead) {
    return "ahead";
  }
  std::vector<std::string> conditions;
  if (event.multiple_lanes) conditions.push_back("multiple_lanes");
  if (event.non_drivable_area) conditions.push_back("non_drivable_area");
  if (event.intersection) conditions.push_back("intersection");
  if (conditions.empty()) {
    return event.bad_or_intersection ? "bad_or_intersection" : "other";
  }
  std::ostringstream oss;
  for (std::size_t index = 0; index < conditions.size(); ++index) {
    if (index > 0U) oss << "+";
    oss << conditions.at(index);
  }
  return oss.str();
}

nlohmann::json ttc_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const TTCWithinBoundDebugInfo & debug_info,
  const rclcpp::Time & timestamp)
{
  nlohmann::json events = nlohmann::json::array();
  for (const auto & event : debug_info.events) {
    events.push_back(
      {{"trajectory_stamp_sec", timestamp.seconds()},
       {"event_stamp_sec", timestamp.seconds() + event.query_time_s},
       {"time_s", event.time_s},
       {"future_offset_s", event.future_offset_s},
       {"query_time_s", event.query_time_s},
       {"object_id", event.object_id},
       {"object_label", event.object_label},
       {"area_condition", ttc_area_condition_string(event)},
       {"ahead", event.ahead},
       {"behind", event.behind},
       {"multiple_lanes", event.multiple_lanes},
       {"non_drivable_area", event.non_drivable_area},
       {"intersection", event.intersection},
       {"bad_or_intersection", event.bad_or_intersection}});
  }
  const auto * event = debug_info.events.empty() ? nullptr : &debug_info.events.front();
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.time_to_collision_within_bound},
    {"reason", metrics.time_to_collision_within_bound_reason},
    {"first_failure_time_s", metrics.time_to_collision_infraction_time_s},
    {"failure_stamp_sec", std::isfinite(metrics.time_to_collision_infraction_time_s)
                            ? timestamp.seconds() + metrics.time_to_collision_infraction_time_s
                            : std::numeric_limits<double>::quiet_NaN()},
    {"future_offset_s", event ? event->future_offset_s : 0.0},
    {"object_id", event ? event->object_id : "invalid"},
    {"object_label", event ? event->object_label : "UNKNOWN"},
    {"area_condition", event ? ttc_area_condition_string(*event) : "other"},
    {"ahead", event ? event->ahead : false},
    {"behind", event ? event->behind : false},
    {"multiple_lanes", event ? event->multiple_lanes : false},
    {"non_drivable_area", event ? event->non_drivable_area : false},
    {"intersection", event ? event->intersection : false},
    {"bad_or_intersection", event ? event->bad_or_intersection : false},
    {"events", std::move(events)}};
}

nlohmann::json lk_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const LaneKeepingDebugInfo & debug_info,
  const rclcpp::Time & timestamp)
{
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.lane_keeping},
    {"reason", metrics.lane_keeping_reason},
    {"first_failure_time_s", debug_info.first_failure_time_s},
    {"failure_run_start_s", debug_info.failure_run_start_time_s},
    {"failure_run_end_s", debug_info.failure_run_end_time_s},
    {"max_continuous_violation_time_s", debug_info.max_continuous_violation_time_s},
    {"peak_abs_lateral_deviation_m", debug_info.peak_abs_lateral_deviation_m},
    {"sample_count", debug_info.samples.size()}};
}

nlohmann::json ep_debug_summary_to_json(
  const TrajectoryPointMetrics & metrics, const rclcpp::Time & timestamp)
{
  return nlohmann::json{
    {"trajectory_stamp_sec", timestamp.seconds()},
    {"score", metrics.ego_progress},
    {"available", metrics.ego_progress_available},
    {"reason", metrics.ego_progress_reason},
    {"raw_progress_m", metrics.ego_progress_raw_m},
    {"best_raw_progress_m", metrics.ego_progress_best_raw_m},
    {"masked_progress_m", metrics.ego_progress_denominator_m},
    {"denominator_m", metrics.ego_progress_denominator_m},
    {"multiplicative_mask", metrics.ego_progress_mask},
    {"mask", metrics.ego_progress_mask},
    {"nc", metrics.no_at_fault_collision},
    {"dac", metrics.drivable_area_compliance},
    {"ddc", metrics.driving_direction_compliance},
    {"tlc", metrics.traffic_light_compliance},
    {"threshold_m", 5.0},
    {"proposal_mode", "single_evaluated_trajectory"},
    {"later_work",
     "evaluate the full candidate proposal batch and compute candidate-level NC/DAC/DDC/TLC"}};
}

std_msgs::msg::ColorRGBA nc_horizon_footprint_color(
  const NoAtFaultCollisionHorizonFootprint & footprint, const bool ego)
{
  if (footprint.at_fault) return make_color(1.0F, 0.05F, 0.05F, 1.0F);
  if (footprint.collision) return make_color(1.0F, 0.8F, 0.0F, 1.0F);
  return ego ? make_color(1.0F, 0.55F, 0.0F, 0.65F) : make_color(0.1F, 0.45F, 1.0F, 0.65F);
}

struct HCComponentStatus
{
  std::string name{"pass"};
  double severity{0.0};
};

double threshold_ratio(const double value, const double threshold)
{
  if (threshold <= 0.0 || !std::isfinite(value)) return 0.0;
  return std::abs(value) / threshold;
}

double longitudinal_acceleration_ratio(const double value, const HistoryComfortParameters & params)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (value > params.max_longitudinal_acceleration) {
    return value / params.max_longitudinal_acceleration;
  }
  if (value < params.min_longitudinal_acceleration) {
    return std::abs(value) / std::abs(params.min_longitudinal_acceleration);
  }
  return 0.0;
}

void consider_hc_component(HCComponentStatus & status, const std::string & name, const double ratio)
{
  if (ratio > status.severity) {
    status.name = name;
    status.severity = ratio;
  }
}

HCComponentStatus hc_component_status_at(
  const TrajectoryPointMetrics & metrics, const HistoryComfortParameters & params,
  const std::size_t index)
{
  HCComponentStatus status;
  if (index < metrics.longitudinal_accelerations.size()) {
    consider_hc_component(
      status, "ax",
      longitudinal_acceleration_ratio(metrics.longitudinal_accelerations.at(index), params));
  }
  if (index < metrics.lateral_accelerations.size()) {
    consider_hc_component(
      status, "ay",
      threshold_ratio(metrics.lateral_accelerations.at(index), params.max_lateral_acceleration));
  }
  if (index < metrics.jerk_magnitudes.size()) {
    consider_hc_component(
      status, "jerk",
      threshold_ratio(metrics.jerk_magnitudes.at(index), params.max_jerk_magnitude));
  }
  if (index < metrics.longitudinal_jerks.size()) {
    consider_hc_component(
      status, "jx",
      threshold_ratio(metrics.longitudinal_jerks.at(index), params.max_longitudinal_jerk));
  }
  if (index < metrics.yaw_rates.size()) {
    consider_hc_component(
      status, "yaw_rate", threshold_ratio(metrics.yaw_rates.at(index), params.max_yaw_rate));
  }
  if (index < metrics.yaw_accelerations.size()) {
    consider_hc_component(
      status, "yaw_accel",
      threshold_ratio(metrics.yaw_accelerations.at(index), params.max_yaw_acceleration));
  }
  if (status.severity <= 1.0) {
    status.name = "pass";
  }
  return status;
}

std_msgs::msg::ColorRGBA hc_component_color(const std::string & component, const bool peak)
{
  if (peak) {
    return make_color(1.0F, 0.0F, 0.65F, 1.0F);
  }
  return component == "pass" ? make_color(1.0F, 0.55F, 0.0F, 0.45F)
                             : make_color(1.0F, 0.05F, 0.05F, 0.95F);
}

}  // namespace

void add_epdms_debug_result_topics(
  std::vector<std::pair<std::string, std::string>> & topics,
  const EpdmsDebugEnabledMetrics & enabled)
{
  add_marker_topic(topics, trajectory_debug_topic("planned_horizon_4s"));
  add_marker_topic(topics, trajectory_debug_topic("gt_horizon_4s"));

  if (enabled.no_at_fault_collision) {
    add_string_topic(topics, debug_topic("nc", "collision_summary"));
    add_marker_topic(topics, debug_topic("nc", "ego_footprints"));
    add_marker_topic(topics, debug_topic("nc", "object_footprints"));
    add_marker_topic(topics, debug_topic("nc", "overlap_areas"));
  }
  if (enabled.drivable_area_compliance) {
    add_string_topic(topics, debug_topic("dac", "violation_summary"));
    add_marker_topic(topics, debug_topic("dac", "ego_footprints"));
    add_marker_topic(topics, debug_topic("dac", "admissible_road_areas"));
    add_marker_topic(topics, debug_topic("dac", "admissible_shoulder_areas"));
    add_marker_topic(topics, debug_topic("dac", "admissible_intersection_areas"));
    add_marker_topic(topics, debug_topic("dac", "admissible_hatched_road_markings"));
    add_marker_topic(topics, debug_topic("dac", "admissible_parking_areas"));
    add_marker_topic(topics, debug_topic("dac", "road_border_lines"));
    add_marker_topic(topics, debug_topic("dac", "road_border_side_test_segments"));
    add_marker_topic(topics, debug_topic("dac", "road_border_gap_segments"));
    add_marker_topic(topics, debug_topic("dac", "semantic_boundary_points"));
    add_marker_topic(topics, debug_topic("dac", "road_border_closest_points"));
    add_marker_topic(topics, debug_topic("dac", "corner_projection_points"));
    add_marker_topic(topics, debug_topic("dac", "road_border_plus_samples"));
    add_marker_topic(topics, debug_topic("dac", "road_border_minus_samples"));
    add_marker_topic(topics, debug_topic("dac", "road_border_fallback_corners"));
    add_marker_topic(topics, debug_topic("dac", "failing_corners"));
  }
  if (enabled.driving_direction_compliance) {
    add_string_topic(topics, debug_topic("ddc", "violation_summary"));
    add_marker_topic(topics, debug_topic("ddc", "ego_centers"));
    add_marker_topic(topics, debug_topic("ddc", "oncoming_segments"));
    add_marker_topic(topics, debug_topic("ddc", "route_lane_polygons"));
    add_marker_topic(topics, debug_topic("ddc", "intersection_lane_polygons"));
  }
  if (enabled.traffic_light_compliance) {
    add_string_topic(topics, debug_topic("tlc", "violation_summary"));
    add_marker_topic(topics, debug_topic("tlc", "ego_footprints"));
    add_marker_topic(topics, debug_topic("tlc", "stop_lines"));
  }
  if (enabled.time_to_collision_within_bound) {
    add_string_topic(topics, debug_topic("ttc", "violation_summary"));
    add_marker_topic(topics, debug_topic("ttc", "ego_footprints"));
    add_marker_topic(topics, debug_topic("ttc", "object_footprints"));
    add_marker_topic(topics, debug_topic("ttc", "overlap_areas"));
  }
  if (enabled.lane_keeping) {
    add_string_topic(topics, debug_topic("lk", "violation_summary"));
    add_marker_topic(topics, debug_topic("lk", "ego_center_path"));
    add_marker_topic(topics, debug_topic("lk", "reference_centerlines"));
  }
  if (enabled.history_comfort) {
    add_string_topic(topics, debug_topic("hc", "component_status"));
    add_array_topic(topics, debug_topic("hc", "sample_times"));
    add_array_topic(topics, debug_topic("hc", "segments"));
    add_marker_topic(topics, debug_topic("hc", "horizon_footprints"));
  }
  if (enabled.extended_comfort) {
    add_string_topic(topics, debug_topic("ec", "comparison_summary"));
    add_array_topic(topics, debug_topic("ec", "sample_times"));
    add_array_topic(topics, debug_topic("ec", "delta_acceleration"));
    add_array_topic(topics, debug_topic("ec", "delta_jerk"));
    add_array_topic(topics, debug_topic("ec", "delta_yaw_rate"));
    add_array_topic(topics, debug_topic("ec", "delta_yaw_accel"));
  }
  if (enabled.ego_progress) {
    add_string_topic(topics, debug_topic("ep", "progress_summary"));
    add_marker_topic(topics, debug_topic("ep", "route_progress_points"));
    add_marker_topic(topics, debug_topic("ep", "route_reference"));
  }
}

void write_epdms_point_debug_topics_to_bag(
  const TrajectoryPointMetrics & metrics, const EpdmsDebugEnabledMetrics & enabled,
  const HistoryComfortParameters & history_comfort_params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & timestamp)
{
  if (enabled.no_at_fault_collision && !metrics.no_at_fault_collision_debug.events.empty()) {
    const auto & debug = metrics.no_at_fault_collision_debug;
    const auto * worst_event = find_worst_nc_event(debug);
    write_string(
      nc_debug_summary_to_json(metrics, debug, worst_event, timestamp),
      debug_topic("nc", "collision_summary"), bag_writer, timestamp);
    MarkerArray ego_markers;
    MarkerArray object_markers;
    MarkerArray overlap_markers;
    ego_markers.markers.push_back(make_delete_all_marker(timestamp));
    object_markers.markers.push_back(make_delete_all_marker(timestamp));
    overlap_markers.markers.push_back(make_delete_all_marker(timestamp));
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      const double width = footprint.collision ? 0.28 : 0.12;
      ego_markers.markers.push_back(make_line_strip_marker(
        timestamp, "nc_horizon_ego_footprints", marker_id++, footprint.footprint,
        nc_horizon_footprint_color(footprint, true), width, true, kDebugMarkerLifetimeS, 0.12));
    }
    marker_id = 0;
    for (const auto & footprint : debug.object_horizon_footprints) {
      const double width = footprint.collision ? 0.28 : 0.12;
      object_markers.markers.push_back(make_line_strip_marker(
        timestamp, "nc_horizon_object_footprints", marker_id++, footprint.footprint,
        nc_horizon_footprint_color(footprint, false), width, true, kDebugMarkerLifetimeS, 0.18));
    }
    marker_id = 0;
    for (const auto & overlap : debug.overlap_areas) {
      overlap_markers.markers.push_back(make_line_strip_marker(
        timestamp, "nc_horizon_overlap_areas", marker_id++, overlap.polygon,
        overlap.at_fault ? make_color(1.0F, 0.0F, 0.8F, 1.0F) : make_color(1.0F, 0.6F, 0.0F, 1.0F),
        overlap.at_fault ? 0.6 : 0.4, true, kDebugMarkerLifetimeS, 0.28));
    }
    bag_writer.write(ego_markers, debug_topic("nc", "ego_footprints"), timestamp);
    bag_writer.write(object_markers, debug_topic("nc", "object_footprints"), timestamp);
    bag_writer.write(overlap_markers, debug_topic("nc", "overlap_areas"), timestamp);
  }

  if (
    enabled.drivable_area_compliance && metrics.drivable_area_compliance_available &&
    metrics.drivable_area_compliance < 1.0) {
    const auto & debug = metrics.drivable_area_compliance_debug;
    if (std::isfinite(debug.first_failure_time_s)) {
      write_string(
        dac_debug_summary_to_json(metrics, debug, timestamp),
        debug_topic("dac", "violation_summary"), bag_writer, timestamp);
    }
    MarkerArray ego_markers;
    ego_markers.markers.push_back(make_delete_all_marker(timestamp));
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      ego_markers.markers.push_back(make_line_strip_marker(
        timestamp, "dac_horizon_ego_footprints", marker_id++, footprint.footprint,
        footprint.non_drivable_area ? make_color(1.0F, 0.05F, 0.05F, 1.0F)
                                    : make_color(1.0F, 0.55F, 0.0F, 0.6F),
        footprint.non_drivable_area ? 0.22 : 0.12, true, kDebugMarkerLifetimeS, 0.10));
    }
    bag_writer.write(ego_markers, debug_topic("dac", "ego_footprints"), timestamp);
    const auto write_polygons = [&](
                                  const auto & polygons, const std::string & name,
                                  const std_msgs::msg::ColorRGBA & marker_color, const double width,
                                  const bool close_line, const double z_offset) {
      MarkerArray markers;
      markers.markers.push_back(make_delete_all_marker(timestamp));
      int id = 0;
      for (const auto & polygon : polygons) {
        markers.markers.push_back(make_line_strip_marker(
          timestamp, "dac_horizon_" + name, id++, polygon.polygon, marker_color, width, close_line,
          kDebugMarkerLifetimeS, z_offset));
      }
      bag_writer.write(markers, debug_topic("dac", name), timestamp);
    };
    write_polygons(
      debug.admissible_road_areas, "admissible_road_areas", make_color(0.0F, 0.9F, 1.0F, 0.8F),
      0.12, true, 0.02);
    write_polygons(
      debug.admissible_shoulder_areas, "admissible_shoulder_areas",
      make_color(0.95F, 0.75F, 0.2F, 0.85F), 0.12, true, 0.03);
    write_polygons(
      debug.admissible_intersection_areas, "admissible_intersection_areas",
      make_color(0.45F, 0.65F, 1.0F, 0.85F), 0.12, true, 0.04);
    write_polygons(
      debug.admissible_hatched_road_markings, "admissible_hatched_road_markings",
      make_color(1.0F, 0.55F, 0.05F, 0.85F), 0.12, true, 0.05);
    write_polygons(
      debug.admissible_parking_areas, "admissible_parking_areas",
      make_color(0.2F, 1.0F, 0.4F, 0.8F), 0.12, true, 0.06);
    write_polygons(
      debug.road_border_lines, "road_border_lines", make_color(0.75F, 0.15F, 1.0F, 0.95F), 0.10,
      false, 0.08);
    write_polygons(
      debug.road_border_side_test_segments, "road_border_side_test_segments",
      make_color(1.0F, 1.0F, 1.0F, 0.95F), 0.18, false, 0.12);
    write_polygons(
      debug.road_border_gap_segments, "road_border_gap_segments",
      make_color(1.0F, 0.85F, 0.05F, 0.95F), 0.16, false, 0.11);
    const auto write_points = [&](
                                const auto & points, const std::string & name,
                                const std_msgs::msg::ColorRGBA & marker_color,
                                const double half_extent, const double width,
                                const double z_offset) {
      MarkerArray markers;
      markers.markers.push_back(make_delete_all_marker(timestamp));
      int id = 0;
      for (const auto & point : points) {
        markers.markers.push_back(make_line_strip_marker(
          timestamp, "dac_horizon_" + name, id++, square_marker_points(point.point, half_extent),
          marker_color, width, false, kDebugMarkerLifetimeS, z_offset));
      }
      bag_writer.write(markers, debug_topic("dac", name), timestamp);
    };
    write_points(
      debug.semantic_boundary_points, "semantic_boundary_points",
      make_color(1.0F, 0.85F, 0.05F, 1.0F), 0.12, 0.10, 0.13);
    write_points(
      debug.road_border_closest_points, "road_border_closest_points",
      make_color(0.75F, 0.15F, 1.0F, 1.0F), 0.12, 0.10, 0.14);
    write_points(
      debug.corner_projection_points, "corner_projection_points",
      make_color(1.0F, 1.0F, 1.0F, 1.0F), 0.10, 0.08, 0.15);
    write_points(
      debug.road_border_plus_samples, "road_border_plus_samples",
      make_color(0.1F, 0.45F, 1.0F, 1.0F), 0.12, 0.10, 0.13);
    write_points(
      debug.road_border_minus_samples, "road_border_minus_samples",
      make_color(0.15F, 1.0F, 0.35F, 1.0F), 0.12, 0.10, 0.14);
    write_points(
      debug.road_border_fallback_corners, "road_border_fallback_corners",
      make_color(0.75F, 0.15F, 1.0F, 1.0F), 0.14, 0.12, 0.17);
    write_points(
      debug.failing_corners, "failing_corners", make_color(1.0F, 0.0F, 0.8F, 1.0F), 0.18, 0.16,
      0.16);
  }

  if (
    enabled.driving_direction_compliance && metrics.driving_direction_compliance_available &&
    metrics.driving_direction_compliance < 1.0 &&
    !metrics.driving_direction_compliance_debug.samples.empty()) {
    const auto & debug = metrics.driving_direction_compliance_debug;
    write_string(
      ddc_debug_summary_to_json(metrics, debug, timestamp), debug_topic("ddc", "violation_summary"),
      bag_writer, timestamp);
    MarkerArray centers;
    MarkerArray segments;
    centers.markers.push_back(make_delete_all_marker(timestamp));
    segments.markers.push_back(make_delete_all_marker(timestamp));
    std::vector<geometry_msgs::msg::Point> center_points;
    center_points.reserve(debug.samples.size());
    int marker_id = 0;
    for (std::size_t index = 0; index < debug.samples.size(); ++index) {
      const auto & sample = debug.samples.at(index);
      center_points.push_back(sample.ego_center);
      if (index == 0U || sample.counted_progress_m <= 0.0) {
        continue;
      }
      segments.markers.push_back(make_line_strip_marker(
        timestamp, "ddc_oncoming_segments", marker_id++,
        {debug.samples.at(index - 1U).ego_center, sample.ego_center},
        make_color(1.0F, 0.05F, 0.05F, 1.0F), 0.18, false, kDebugMarkerLifetimeS, 0.12));
    }
    if (center_points.size() >= 2U) {
      centers.markers.push_back(make_line_strip_marker(
        timestamp, "ddc_ego_centers", 0, center_points, make_color(1.0F, 0.55F, 0.0F, 0.8F), 0.12,
        false, kDebugMarkerLifetimeS, 0.08));
    }
    bag_writer.write(centers, debug_topic("ddc", "ego_centers"), timestamp);
    bag_writer.write(segments, debug_topic("ddc", "oncoming_segments"), timestamp);
    const auto write_ddc_polygons = [&](
                                      const auto & polygons, const std::string & name,
                                      const std_msgs::msg::ColorRGBA & marker_color,
                                      const double width, const double z_offset) {
      MarkerArray markers;
      markers.markers.push_back(make_delete_all_marker(timestamp));
      int id = 0;
      for (const auto & polygon : polygons) {
        markers.markers.push_back(make_line_strip_marker(
          timestamp, "ddc_" + name, id++, polygon.polygon, marker_color, width, true,
          kDebugMarkerLifetimeS, z_offset));
      }
      bag_writer.write(markers, debug_topic("ddc", name), timestamp);
    };
    write_ddc_polygons(
      debug.route_lane_polygons, "route_lane_polygons", make_color(0.0F, 0.9F, 1.0F, 0.85F), 0.18,
      0.16);
    write_ddc_polygons(
      debug.intersection_lane_polygons, "intersection_lane_polygons",
      make_color(0.2F, 1.0F, 0.4F, 0.90F), 0.20, 0.22);
  }

  if (
    enabled.traffic_light_compliance && metrics.traffic_light_compliance_available &&
    metrics.traffic_light_compliance < 1.0) {
    const auto & debug = metrics.traffic_light_compliance_debug;
    if (std::isfinite(debug.first_failure_time_s)) {
      write_string(
        tlc_debug_summary_to_json(metrics, debug, timestamp),
        debug_topic("tlc", "violation_summary"), bag_writer, timestamp);
    }
    MarkerArray ego_markers;
    MarkerArray stop_line_markers;
    ego_markers.markers.push_back(make_delete_all_marker(timestamp));
    stop_line_markers.markers.push_back(make_delete_all_marker(timestamp));
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      const bool failed = std::abs(footprint.time_s - debug.first_failure_time_s) < 1.0e-3;
      ego_markers.markers.push_back(make_line_strip_marker(
        timestamp, "tlc_ego_footprints", marker_id++, footprint.polygon,
        failed ? make_color(1.0F, 0.05F, 0.05F, 1.0F) : make_color(1.0F, 0.55F, 0.0F, 0.60F),
        failed ? 0.22 : 0.12, true, kDebugMarkerLifetimeS, 0.12));
    }
    marker_id = 0;
    for (const auto & line : debug.stop_lines) {
      stop_line_markers.markers.push_back(make_line_strip_marker(
        timestamp, "tlc_stop_lines", marker_id++, line.polygon,
        make_color(1.0F, 0.05F, 0.05F, 0.95F), 0.18, false, kDebugMarkerLifetimeS, 0.28));
    }
    bag_writer.write(ego_markers, debug_topic("tlc", "ego_footprints"), timestamp);
    bag_writer.write(stop_line_markers, debug_topic("tlc", "stop_lines"), timestamp);
  }

  if (
    enabled.time_to_collision_within_bound && metrics.time_to_collision_within_bound_available &&
    metrics.time_to_collision_within_bound < 1.0 &&
    !metrics.time_to_collision_within_bound_debug.events.empty()) {
    const auto & debug = metrics.time_to_collision_within_bound_debug;
    write_string(
      ttc_debug_summary_to_json(metrics, debug, timestamp), debug_topic("ttc", "violation_summary"),
      bag_writer, timestamp);
    MarkerArray ego_markers;
    MarkerArray object_markers;
    MarkerArray overlap_markers;
    ego_markers.markers.push_back(make_delete_all_marker(timestamp));
    object_markers.markers.push_back(make_delete_all_marker(timestamp));
    overlap_markers.markers.push_back(make_delete_all_marker(timestamp));
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      const bool prefix = footprint.prefix;
      const double width = prefix ? 0.08 : (footprint.overlap ? 0.28 : 0.12);
      const auto marker_color = prefix
                                  ? make_color(1.0F, 0.55F, 0.0F, 0.28F)
                                  : (footprint.overlap ? make_color(1.0F, 0.05F, 0.05F, 1.0F)
                                                       : make_color(0.95F, 0.28F, 0.0F, 0.88F));
      ego_markers.markers.push_back(make_line_strip_marker(
        timestamp, "ttc_ego_footprints", marker_id++, footprint.footprint, marker_color, width,
        true, kDebugMarkerLifetimeS, 0.12));
    }
    marker_id = 0;
    for (const auto & footprint : debug.object_horizon_footprints) {
      const bool prefix = footprint.prefix;
      const double width = prefix ? 0.08 : (footprint.overlap ? 0.28 : 0.12);
      const auto marker_color = prefix
                                  ? make_color(0.1F, 0.45F, 1.0F, 0.28F)
                                  : (footprint.overlap ? make_color(1.0F, 0.05F, 0.05F, 1.0F)
                                                       : make_color(0.02F, 0.12F, 0.70F, 0.88F));
      object_markers.markers.push_back(make_line_strip_marker(
        timestamp, "ttc_object_footprints", marker_id++, footprint.footprint, marker_color, width,
        true, kDebugMarkerLifetimeS, 0.18));
    }
    marker_id = 0;
    for (const auto & overlap : debug.overlap_areas) {
      overlap_markers.markers.push_back(make_line_strip_marker(
        timestamp, "ttc_overlap_areas", marker_id++, overlap.polygon,
        make_color(1.0F, 0.05F, 0.05F, 1.0F), 0.40, true, kDebugMarkerLifetimeS, 0.24));
    }
    bag_writer.write(ego_markers, debug_topic("ttc", "ego_footprints"), timestamp);
    bag_writer.write(object_markers, debug_topic("ttc", "object_footprints"), timestamp);
    bag_writer.write(overlap_markers, debug_topic("ttc", "overlap_areas"), timestamp);
  }

  if (
    enabled.lane_keeping && metrics.lane_keeping_available && metrics.lane_keeping < 1.0 &&
    !metrics.lane_keeping_debug.samples.empty()) {
    const auto & debug = metrics.lane_keeping_debug;
    write_string(
      lk_debug_summary_to_json(metrics, debug, timestamp), debug_topic("lk", "violation_summary"),
      bag_writer, timestamp);
    MarkerArray path_markers;
    MarkerArray centerline_markers;
    path_markers.markers.push_back(make_delete_all_marker(timestamp));
    centerline_markers.markers.push_back(make_delete_all_marker(timestamp));
    int marker_id = 0;
    for (std::size_t index = 1; index < debug.samples.size(); ++index) {
      const auto & previous = debug.samples.at(index - 1U);
      const auto & sample = debug.samples.at(index);
      const bool violating = sample.over_threshold && !sample.is_in_intersection;
      const auto marker_color = sample.in_failure_run       ? make_color(1.0F, 0.12F, 0.12F, 1.0F)
                                : sample.is_in_intersection ? make_color(0.2F, 1.0F, 0.4F, 0.95F)
                                : violating                 ? make_color(1.0F, 0.65F, 0.0F, 0.95F)
                                                            : make_color(1.0F, 0.55F, 0.0F, 0.85F);
      const double width = sample.in_failure_run ? 0.18 : 0.12;
      path_markers.markers.push_back(make_line_strip_marker(
        timestamp, "lk_ego_center_path", marker_id++, {previous.ego_center, sample.ego_center},
        marker_color, width, false, kDebugMarkerLifetimeS, 0.08));
    }
    std::set<std::int64_t> seen_lanelet_ids;
    marker_id = 0;
    for (const auto & sample : debug.samples) {
      if (sample.reference_centerline.size() < 2U || sample.reference_lanelet_id < 0) {
        continue;
      }
      if (!seen_lanelet_ids.insert(sample.reference_lanelet_id).second) {
        continue;
      }
      centerline_markers.markers.push_back(make_line_strip_marker(
        timestamp, "lk_reference_centerlines", marker_id++, sample.reference_centerline,
        make_color(0.2F, 1.0F, 0.4F, 0.80F), 0.08, false, kDebugMarkerLifetimeS, 0.12));
    }
    bag_writer.write(path_markers, debug_topic("lk", "ego_center_path"), timestamp);
    bag_writer.write(centerline_markers, debug_topic("lk", "reference_centerlines"), timestamp);
  }

  if (enabled.history_comfort && !metrics.history_comfort_sample_times.empty()) {
    std_msgs::msg::String summary_msg;
    summary_msg.data = metrics.history_comfort_debug_summary;
    bag_writer.write(summary_msg, debug_topic("hc", "component_status"), timestamp);
    write_array(
      metrics.history_comfort_sample_times, debug_topic("hc", "sample_times"), bag_writer,
      timestamp);
    write_array(
      metrics.history_comfort_segment_ids, debug_topic("hc", "segments"), bag_writer, timestamp);
    MarkerArray markers;
    markers.markers.push_back(make_delete_all_marker(timestamp));
    const auto local_footprint = vehicle_info.createFootprint(0.0);
    std::vector<HCComponentStatus> statuses;
    statuses.reserve(metrics.history_comfort_sample_poses.size());
    std::size_t peak_index = 0U;
    double peak_severity = 0.0;
    for (std::size_t index = 0; index < metrics.history_comfort_sample_poses.size(); ++index) {
      auto status = hc_component_status_at(metrics, history_comfort_params, index);
      if (status.severity > peak_severity) {
        peak_severity = status.severity;
        peak_index = index;
      }
      statuses.push_back(std::move(status));
    }
    int marker_id = 0;
    for (std::size_t index = 0; index < metrics.history_comfort_sample_poses.size(); ++index) {
      const auto & pose = metrics.history_comfort_sample_poses.at(index);
      const auto footprint = create_pose_footprint(pose, local_footprint);
      const bool peak = peak_severity > 1.0 && index == peak_index;
      const auto & status = statuses.at(index);
      const bool failed = status.severity > 1.0;
      const double width = peak ? 0.34 : (failed ? 0.22 : 0.10);
      const double z_offset = peak ? 0.24 : (failed ? 0.16 : 0.08);
      markers.markers.push_back(make_line_strip_marker(
        timestamp, "hc_horizon_footprints", marker_id++,
        polygon_to_msg_points(footprint, pose.position.z), hc_component_color(status.name, peak),
        width, true, kDebugMarkerLifetimeS, z_offset));
    }
    bag_writer.write(markers, debug_topic("hc", "horizon_footprints"), timestamp);
  }

  if (enabled.ego_progress) {
    write_string(
      ep_debug_summary_to_json(metrics, timestamp), debug_topic("ep", "progress_summary"),
      bag_writer, timestamp);
    MarkerArray route_points;
    route_points.markers.push_back(make_delete_all_marker(timestamp));
    route_points.markers.push_back(make_line_strip_marker(
      timestamp, "ep_start_point", 0, square_marker_points(metrics.ego_progress_start_point, 0.35),
      make_color(0.1F, 0.9F, 1.0F, 1.0F), 0.18, true, kDebugMarkerLifetimeS, 0.22));
    route_points.markers.push_back(make_line_strip_marker(
      timestamp, "ep_end_point", 1, square_marker_points(metrics.ego_progress_end_point, 0.35),
      make_color(0.0F, 1.0F, 0.35F, 1.0F), 0.18, true, kDebugMarkerLifetimeS, 0.24));
    route_points.markers.push_back(make_line_strip_marker(
      timestamp, "ep_progress_chord", 2,
      {metrics.ego_progress_start_point, metrics.ego_progress_end_point},
      make_color(0.0F, 0.8F, 1.0F, 0.9F), 0.16, false, kDebugMarkerLifetimeS, 0.20));
    MarkerArray route_reference;
    route_reference.markers.push_back(make_delete_all_marker(timestamp));
    if (metrics.ego_progress_route_reference_points.size() >= 2U) {
      route_reference.markers.push_back(make_line_strip_marker(
        timestamp, "ep_route_reference", 0, metrics.ego_progress_route_reference_points,
        make_color(1.0F, 0.85F, 0.15F, 0.85F), 0.12, false, kDebugMarkerLifetimeS, 0.14));
    }
    bag_writer.write(route_points, debug_topic("ep", "route_progress_points"), timestamp);
    bag_writer.write(route_reference, debug_topic("ep", "route_reference"), timestamp);
  }
}

void write_epdms_extended_comfort_debug_topics_to_bag(
  const std::string & summary, const std::vector<double> & sample_times,
  const std::vector<double> & delta_acceleration, const std::vector<double> & delta_jerk,
  const std::vector<double> & delta_yaw_rate, const std::vector<double> & delta_yaw_accel,
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp)
{
  if (summary.empty()) {
    return;
  }
  std_msgs::msg::String summary_msg;
  summary_msg.data = summary;
  bag_writer.write(summary_msg, debug_topic("ec", "comparison_summary"), timestamp);
  write_array(sample_times, debug_topic("ec", "sample_times"), bag_writer, timestamp);
  write_array(delta_acceleration, debug_topic("ec", "delta_acceleration"), bag_writer, timestamp);
  write_array(delta_jerk, debug_topic("ec", "delta_jerk"), bag_writer, timestamp);
  write_array(delta_yaw_rate, debug_topic("ec", "delta_yaw_rate"), bag_writer, timestamp);
  write_array(delta_yaw_accel, debug_topic("ec", "delta_yaw_accel"), bag_writer, timestamp);
}

void write_epdms_trajectory_horizon_debug_topics_to_bag(
  const autoware_planning_msgs::msg::Trajectory & planned_trajectory,
  const autoware_planning_msgs::msg::Trajectory & ground_truth_trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & timestamp)
{
  constexpr double kDebugHorizonSeconds = 4.0;
  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const auto planned_horizon =
    truncate_trajectory_by_horizon(planned_trajectory, kDebugHorizonSeconds);
  const auto gt_horizon =
    truncate_trajectory_by_horizon(ground_truth_trajectory, kDebugHorizonSeconds);

  MarkerArray planned_markers;
  MarkerArray gt_markers;
  planned_markers.markers.push_back(make_delete_all_marker(timestamp));
  gt_markers.markers.push_back(make_delete_all_marker(timestamp));

  int32_t marker_id = 0;
  for (const auto & point : planned_horizon.points) {
    const auto footprint = create_pose_footprint(point.pose, local_footprint);
    planned_markers.markers.push_back(make_filled_polygon_marker(
      timestamp, "trajectory_planned_horizon_4s", marker_id++,
      polygon_to_msg_points(footprint, point.pose.position.z + 0.04),
      make_color(1.0F, 0.55F, 0.0F, 0.35F), kDebugMarkerLifetimeS, 0.0));
  }

  marker_id = 0;
  for (const auto & point : gt_horizon.points) {
    const auto footprint = create_pose_footprint(point.pose, local_footprint);
    gt_markers.markers.push_back(make_filled_polygon_marker(
      timestamp, "trajectory_gt_horizon_4s", marker_id++,
      polygon_to_msg_points(footprint, point.pose.position.z + 0.08),
      make_color(0.2F, 1.0F, 0.4F, 0.35F), kDebugMarkerLifetimeS, 0.0));
  }

  bag_writer.write(planned_markers, trajectory_debug_topic("planned_horizon_4s"), timestamp);
  bag_writer.write(gt_markers, trajectory_debug_topic("gt_horizon_4s"), timestamp);
}

}  // namespace autoware::planning_data_analyzer::metrics

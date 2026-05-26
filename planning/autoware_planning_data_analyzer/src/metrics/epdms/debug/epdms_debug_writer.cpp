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

#include <nlohmann/json.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kDebugMarkerLifetimeS = 0.2;
constexpr double kDebugZOffset = 0.15;

std::string debug_topic(const std::string & subscore, const std::string & name)
{
  return "/debug/epdms/" + subscore + "/" + name;
}

std::string trajectory_debug_topic(const std::string & name)
{
  return debug_topic("trajectory", name);
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA msg;
  msg.r = r;
  msg.g = g;
  msg.b = b;
  msg.a = a;
  return msg;
}

const auto kOrange = color(1.0F, 0.55F, 0.05F, 0.55F);
const auto kDeepOrange = color(0.95F, 0.25F, 0.0F, 0.75F);
const auto kBlue = color(0.10F, 0.45F, 1.0F, 0.55F);
const auto kDeepBlue = color(0.0F, 0.1F, 0.9F, 0.75F);
const auto kRed = color(1.0F, 0.0F, 0.0F, 0.8F);
const auto kGreen = color(0.0F, 0.9F, 0.25F, 0.65F);
const auto kYellow = color(1.0F, 0.9F, 0.0F, 0.75F);
const auto kPink = color(1.0F, 0.0F, 0.55F, 0.75F);
const auto kGray = color(0.55F, 0.55F, 0.55F, 0.4F);

Marker make_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id, const int type,
  const std_msgs::msg::ColorRGBA & marker_color, const double scale)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = marker_color;
  marker.lifetime = rclcpp::Duration::from_seconds(kDebugMarkerLifetimeS);
  return marker;
}

geometry_msgs::msg::Point lifted(geometry_msgs::msg::Point point)
{
  point.z += kDebugZOffset;
  return point;
}

Marker make_line_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & marker_color, const double width)
{
  auto marker = make_marker(stamp, ns, id, Marker::LINE_STRIP, marker_color, width);
  marker.points.reserve(points.size());
  for (const auto & point : points) {
    marker.points.push_back(lifted(point));
  }
  return marker;
}

Marker make_point_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  const geometry_msgs::msg::Point & point, const std_msgs::msg::ColorRGBA & marker_color,
  const double scale)
{
  auto marker = make_marker(stamp, ns, id, Marker::SPHERE, marker_color, scale);
  marker.pose.position = lifted(point);
  return marker;
}

Marker make_polygon_marker(
  const rclcpp::Time & stamp, const std::string & ns, const int id,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & marker_color)
{
  auto marker = make_marker(stamp, ns, id, Marker::TRIANGLE_LIST, marker_color, 1.0);
  if (points.size() < 3U) {
    return marker;
  }
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    marker.points.push_back(lifted(points.front()));
    marker.points.push_back(lifted(points.at(index)));
    marker.points.push_back(lifted(points.at(index + 1U)));
  }
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
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp)
{
  if (enabled.no_at_fault_collision && !metrics.no_at_fault_collision_debug.events.empty()) {
    const auto & debug = metrics.no_at_fault_collision_debug;
    write_string(
      {{"score", metrics.no_at_fault_collision},
       {"reason", metrics.no_at_fault_collision_reason},
       {"event_count", debug.events.size()}},
      debug_topic("nc", "collision_summary"), bag_writer, timestamp);
    MarkerArray ego_markers;
    MarkerArray object_markers;
    MarkerArray overlap_markers;
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      ego_markers.markers.push_back(make_polygon_marker(
        timestamp, "nc_ego", marker_id++, footprint.footprint,
        footprint.at_fault ? kRed : kOrange));
    }
    marker_id = 0;
    for (const auto & footprint : debug.object_horizon_footprints) {
      object_markers.markers.push_back(make_polygon_marker(
        timestamp, "nc_object", marker_id++, footprint.footprint,
        footprint.at_fault ? kRed : kBlue));
    }
    marker_id = 0;
    for (const auto & overlap : debug.overlap_areas) {
      overlap_markers.markers.push_back(make_polygon_marker(
        timestamp, "nc_overlap", marker_id++, overlap.polygon, overlap.at_fault ? kRed : kYellow));
    }
    bag_writer.write(ego_markers, debug_topic("nc", "ego_footprints"), timestamp);
    bag_writer.write(object_markers, debug_topic("nc", "object_footprints"), timestamp);
    bag_writer.write(overlap_markers, debug_topic("nc", "overlap_areas"), timestamp);
  }

  if (enabled.drivable_area_compliance) {
    const auto & debug = metrics.drivable_area_compliance_debug;
    if (std::isfinite(debug.first_failure_time_s)) {
      write_string(
        {{"score", metrics.drivable_area_compliance},
         {"reason", metrics.drivable_area_compliance_reason},
         {"first_failure_time_s", debug.first_failure_time_s},
         {"failing_corner_indices", debug.failing_corner_indices},
         {"road_border_fallback_used", debug.road_border_fallback_used}},
        debug_topic("dac", "violation_summary"), bag_writer, timestamp);
    }
    MarkerArray ego_markers;
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      ego_markers.markers.push_back(make_polygon_marker(
        timestamp, "dac_ego", marker_id++, footprint.footprint,
        footprint.non_drivable_area ? kRed : kOrange));
    }
    bag_writer.write(ego_markers, debug_topic("dac", "ego_footprints"), timestamp);
    const auto write_polygons = [&](
                                  const auto & polygons, const std::string & name,
                                  const std_msgs::msg::ColorRGBA & marker_color) {
      MarkerArray markers;
      int id = 0;
      for (const auto & polygon : polygons) {
        markers.markers.push_back(
          make_polygon_marker(timestamp, "dac_" + name, id++, polygon.polygon, marker_color));
      }
      bag_writer.write(markers, debug_topic("dac", name), timestamp);
    };
    write_polygons(debug.admissible_road_areas, "admissible_road_areas", kGreen);
    write_polygons(debug.admissible_shoulder_areas, "admissible_shoulder_areas", kYellow);
    write_polygons(debug.admissible_intersection_areas, "admissible_intersection_areas", kGreen);
    write_polygons(
      debug.admissible_hatched_road_markings, "admissible_hatched_road_markings", kYellow);
    write_polygons(debug.admissible_parking_areas, "admissible_parking_areas", kGreen);
    write_polygons(debug.road_border_lines, "road_border_lines", kGray);
    write_polygons(debug.road_border_side_test_segments, "road_border_side_test_segments", kPink);
    write_polygons(debug.road_border_gap_segments, "road_border_gap_segments", kYellow);
    const auto write_points = [&](
                                const auto & points, const std::string & name,
                                const std_msgs::msg::ColorRGBA & marker_color) {
      MarkerArray markers;
      int id = 0;
      for (const auto & point : points) {
        markers.markers.push_back(
          make_point_marker(timestamp, "dac_" + name, id++, point.point, marker_color, 0.25));
      }
      bag_writer.write(markers, debug_topic("dac", name), timestamp);
    };
    write_points(debug.semantic_boundary_points, "semantic_boundary_points", kGreen);
    write_points(debug.road_border_closest_points, "road_border_closest_points", kPink);
    write_points(debug.corner_projection_points, "corner_projection_points", kYellow);
    write_points(debug.road_border_plus_samples, "road_border_plus_samples", kBlue);
    write_points(debug.road_border_minus_samples, "road_border_minus_samples", kDeepBlue);
    write_points(debug.road_border_fallback_corners, "road_border_fallback_corners", kGreen);
    write_points(debug.failing_corners, "failing_corners", kRed);
  }

  if (
    enabled.driving_direction_compliance &&
    !metrics.driving_direction_compliance_debug.samples.empty()) {
    const auto & debug = metrics.driving_direction_compliance_debug;
    write_string(
      {{"score", metrics.driving_direction_compliance},
       {"reason", metrics.driving_direction_compliance_reason},
       {"window_progress_m", debug.window_progress_m},
       {"worst_window_start_s", debug.worst_window_start_time_s},
       {"worst_window_end_s", debug.worst_window_end_time_s}},
      debug_topic("ddc", "violation_summary"), bag_writer, timestamp);
    MarkerArray centers;
    MarkerArray segments;
    int id = 0;
    for (std::size_t index = 0; index < debug.samples.size(); ++index) {
      const auto & sample = debug.samples.at(index);
      centers.markers.push_back(make_point_marker(
        timestamp, "ddc_ego_center", id++, sample.ego_center,
        sample.in_oncoming_traffic ? kRed : kOrange, 0.2));
      if (index > 0U && sample.counted_progress_m > 0.0) {
        segments.markers.push_back(make_line_marker(
          timestamp, "ddc_oncoming", static_cast<int>(segments.markers.size()),
          {debug.samples.at(index - 1U).ego_center, sample.ego_center}, kRed, 0.2));
      }
    }
    bag_writer.write(centers, debug_topic("ddc", "ego_centers"), timestamp);
    bag_writer.write(segments, debug_topic("ddc", "oncoming_segments"), timestamp);
    const auto write_ddc_polygons = [&](const auto & polygons, const std::string & name) {
      MarkerArray markers;
      int marker_id = 0;
      for (const auto & polygon : polygons) {
        markers.markers.push_back(
          make_polygon_marker(timestamp, "ddc_" + name, marker_id++, polygon.polygon, kGreen));
      }
      bag_writer.write(markers, debug_topic("ddc", name), timestamp);
    };
    write_ddc_polygons(debug.route_lane_polygons, "route_lane_polygons");
    write_ddc_polygons(debug.intersection_lane_polygons, "intersection_lane_polygons");
  }

  if (enabled.traffic_light_compliance) {
    const auto & debug = metrics.traffic_light_compliance_debug;
    if (std::isfinite(debug.first_failure_time_s)) {
      write_string(
        {{"score", metrics.traffic_light_compliance},
         {"reason", metrics.traffic_light_compliance_reason},
         {"first_failure_time_s", debug.first_failure_time_s},
         {"selected_stop_line_count", debug.selected_stop_line_count}},
        debug_topic("tlc", "violation_summary"), bag_writer, timestamp);
    }
    MarkerArray ego_markers;
    MarkerArray stop_line_markers;
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      const bool failed = std::abs(footprint.time_s - debug.first_failure_time_s) < 1.0e-3;
      ego_markers.markers.push_back(make_polygon_marker(
        timestamp, "tlc_ego", marker_id++, footprint.polygon, failed ? kRed : kOrange));
    }
    marker_id = 0;
    for (const auto & line : debug.stop_lines) {
      stop_line_markers.markers.push_back(
        make_line_marker(timestamp, "tlc_stop_line", marker_id++, line.polygon, kRed, 0.35));
    }
    bag_writer.write(ego_markers, debug_topic("tlc", "ego_footprints"), timestamp);
    bag_writer.write(stop_line_markers, debug_topic("tlc", "stop_lines"), timestamp);
  }

  if (
    enabled.time_to_collision_within_bound &&
    !metrics.time_to_collision_within_bound_debug.events.empty()) {
    const auto & debug = metrics.time_to_collision_within_bound_debug;
    write_string(
      {{"score", metrics.time_to_collision_within_bound},
       {"reason", metrics.time_to_collision_within_bound_reason},
       {"event_count", debug.events.size()}},
      debug_topic("ttc", "violation_summary"), bag_writer, timestamp);
    MarkerArray ego_markers;
    MarkerArray object_markers;
    MarkerArray overlap_markers;
    int marker_id = 0;
    for (const auto & footprint : debug.ego_horizon_footprints) {
      const auto marker_color =
        footprint.failing ? kRed : (footprint.prefix ? kOrange : kDeepOrange);
      ego_markers.markers.push_back(
        make_polygon_marker(timestamp, "ttc_ego", marker_id++, footprint.footprint, marker_color));
    }
    marker_id = 0;
    for (const auto & footprint : debug.object_horizon_footprints) {
      const auto marker_color = footprint.failing ? kRed : (footprint.prefix ? kBlue : kDeepBlue);
      object_markers.markers.push_back(make_polygon_marker(
        timestamp, "ttc_object", marker_id++, footprint.footprint, marker_color));
    }
    marker_id = 0;
    for (const auto & overlap : debug.overlap_areas) {
      overlap_markers.markers.push_back(make_polygon_marker(
        timestamp, "ttc_overlap", marker_id++, overlap.polygon, overlap.failing ? kRed : kYellow));
    }
    bag_writer.write(ego_markers, debug_topic("ttc", "ego_footprints"), timestamp);
    bag_writer.write(object_markers, debug_topic("ttc", "object_footprints"), timestamp);
    bag_writer.write(overlap_markers, debug_topic("ttc", "overlap_areas"), timestamp);
  }

  if (enabled.lane_keeping && !metrics.lane_keeping_debug.samples.empty()) {
    const auto & debug = metrics.lane_keeping_debug;
    write_string(
      {{"score", metrics.lane_keeping},
       {"reason", metrics.lane_keeping_reason},
       {"first_failure_time_s", debug.first_failure_time_s},
       {"max_continuous_violation_time_s", debug.max_continuous_violation_time_s},
       {"peak_abs_lateral_deviation_m", debug.peak_abs_lateral_deviation_m}},
      debug_topic("lk", "violation_summary"), bag_writer, timestamp);
    MarkerArray path_markers;
    MarkerArray centerline_markers;
    for (std::size_t index = 1; index < debug.samples.size(); ++index) {
      const auto & sample = debug.samples.at(index);
      const auto marker_color = sample.in_failure_run       ? kRed
                                : sample.is_in_intersection ? kGreen
                                : sample.over_threshold     ? kYellow
                                                            : kOrange;
      path_markers.markers.push_back(make_line_marker(
        timestamp, "lk_ego_center_path", static_cast<int>(index),
        {debug.samples.at(index - 1U).ego_center, sample.ego_center}, marker_color, 0.2));
    }
    int marker_id = 0;
    for (const auto & sample : debug.samples) {
      if (!sample.reference_centerline.empty()) {
        centerline_markers.markers.push_back(make_line_marker(
          timestamp, "lk_reference_centerline", marker_id++, sample.reference_centerline, kGreen,
          0.12));
      }
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
    int marker_id = 0;
    for (const auto & pose : metrics.history_comfort_sample_poses) {
      markers.markers.push_back(
        make_point_marker(timestamp, "hc_horizon", marker_id++, pose.position, kOrange, 0.25));
    }
    bag_writer.write(markers, debug_topic("hc", "horizon_footprints"), timestamp);
  }

  if (enabled.ego_progress) {
    write_string(
      {{"score", metrics.ego_progress},
       {"reason", metrics.ego_progress_reason},
       {"raw_progress_m", metrics.ego_progress_raw_m},
       {"best_raw_progress_m", metrics.ego_progress_best_raw_m},
       {"mask", metrics.ego_progress_mask},
       {"denominator_m", metrics.ego_progress_denominator_m}},
      debug_topic("ep", "progress_summary"), bag_writer, timestamp);
    MarkerArray route_points;
    route_points.markers.push_back(
      make_point_marker(timestamp, "ep_start", 0, metrics.ego_progress_start_point, kGreen, 0.35));
    route_points.markers.push_back(
      make_point_marker(timestamp, "ep_end", 1, metrics.ego_progress_end_point, kRed, 0.35));
    MarkerArray route_reference;
    route_reference.markers.push_back(make_line_marker(
      timestamp, "ep_route_reference", 0, metrics.ego_progress_route_reference_points, kGreen,
      0.15));
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
  rosbag2_cpp::Writer & bag_writer, const rclcpp::Time & timestamp)
{
  const auto write_points = [&](
                              const auto & trajectory, const std::string & topic,
                              const std_msgs::msg::ColorRGBA & marker_color) {
    MarkerArray markers;
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto & point : trajectory.points) {
      const double time_s = rclcpp::Duration(point.time_from_start).seconds();
      if (time_s > 4.0 + 1.0e-6) {
        break;
      }
      points.push_back(point.pose.position);
    }
    markers.markers.push_back(make_line_marker(timestamp, topic, 0, points, marker_color, 0.25));
    bag_writer.write(markers, trajectory_debug_topic(topic), timestamp);
  };
  write_points(planned_trajectory, "planned_horizon_4s", kOrange);
  write_points(ground_truth_trajectory, "gt_horizon_4s", kGreen);
}

}  // namespace autoware::planning_data_analyzer::metrics

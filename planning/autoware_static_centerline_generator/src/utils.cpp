// Copyright 2022 Tier IV, Inc.
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

#include "utils.hpp"

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::static_centerline_generator
{
namespace
{
lanelet::Point3d createPoint3d(const double x, const double y, const double z)
{
  lanelet::Point3d point(lanelet::utils::getId());

  // position
  point.x() = x;
  point.y() = y;
  point.z() = z;

  // attributes
  point.setAttribute("local_x", x);
  point.setAttribute("local_y", y);
  // NOTE: It seems that the attribute "ele" is assigned automatically.

  return point;
}
}  // namespace

namespace utils
{
geometry_msgs::msg::Pose create_pose(
  const double px, const double py, const double pz, const double qx, const double qy,
  const double qz, const double qw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

nav_msgs::msg::Odometry::ConstSharedPtr convert_to_odometry(const geometry_msgs::msg::Pose & pose)
{
  auto odometry_ptr = std::make_shared<nav_msgs::msg::Odometry>();
  odometry_ptr->pose.pose = pose;
  return odometry_ptr;
}

rclcpp::QoS create_transient_local_qos()
{
  return rclcpp::QoS{1}.transient_local();
}

lanelet::ConstLanelets get_lanelets_from_ids(
  const RouteHandler & route_handler, const std::vector<lanelet::Id> & lane_ids)
{
  lanelet::ConstLanelets lanelets;
  for (const lanelet::Id lane_id : lane_ids) {
    const auto lanelet = route_handler.getLaneletsFromId(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

lanelet::ConstLanelets get_lanelets_from_route(
  const RouteHandler & route_handler, const LaneletRoute & route)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & segment : route.segments) {
    const auto lane_id = segment.preferred_primitive.id;
    const auto lanelet = route_handler.getLaneletsFromId(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

geometry_msgs::msg::Pose get_center_pose(
  const RouteHandler & route_handler, const lanelet::Id lanelet_id)
{
  // get middle idx of the lanelet
  const auto lanelet = route_handler.getLaneletsFromId(lanelet_id);
  const auto center_line = lanelet.centerline();
  if (center_line.size() < 2) {
    throw std::invalid_argument("center_line's size is less than 2.");
  }

  // Note: -1 operation is added considering that the size is 2.
  const size_t middle_point_idx = std::floor((center_line.size() - 1) / 2.0);

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();
  middle_pos.z = center_line[middle_point_idx].z();

  // get next middle position of the lanelet
  geometry_msgs::msg::Point next_middle_pos;
  next_middle_pos.x = center_line[middle_point_idx + 1].x();
  next_middle_pos.y = center_line[middle_point_idx + 1].y();
  next_middle_pos.z = center_line[middle_point_idx + 1].z();

  // calculate middle pose
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position = middle_pos;
  const double yaw = autoware::universe_utils::calcAzimuthAngle(middle_pos, next_middle_pos);
  middle_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);

  return middle_pose;
}

PathWithLaneId get_path_with_lane_id(
  const RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold)
{
  // get center line
  constexpr double s_start = 0.0;
  constexpr double s_end = std::numeric_limits<double>::max();
  auto path_with_lane_id = route_handler.getCenterLinePath(lanelets, s_start, s_end);
  path_with_lane_id.header.frame_id = "map";

  // create planner data
  auto planner_data = std::make_shared<behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<RouteHandler>(route_handler);
  planner_data->self_odometry = convert_to_odometry(start_pose);
  planner_data->parameters.ego_nearest_dist_threshold = ego_nearest_dist_threshold;
  planner_data->parameters.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold;

  // generate drivable area and store it in path with lane id
  constexpr double vehicle_length = 0.0;
  const auto drivable_lanes = behavior_path_planner::utils::generateDrivableLanes(lanelets);
  behavior_path_planner::utils::generateDrivableArea(
    path_with_lane_id, drivable_lanes, false, false, vehicle_length, planner_data);

  return path_with_lane_id;
}

void update_centerline(
  lanelet::LaneletMapPtr lanelet_map_ptr, const std::vector<TrajectoryPoint> & new_centerline,
  const std::vector<lanelet::Id> & centerline_lane_ids)
{
  // get lanelet as reference to update centerline
  std::unordered_map<lanelet::Id, lanelet::Lanelet> lanelet_ref_map;
  for (const lanelet::Id centerline_lane_id : centerline_lane_ids) {
    if (!lanelet_ref_map.count(centerline_lane_id) == 0) {
      continue;
    }
    for (auto & lanelet_ref : lanelet_map_ptr->laneletLayer) {
      if (lanelet_ref.id() == static_cast<int>(centerline_lane_id)) {
        lanelet_ref_map.emplace(centerline_lane_id, lanelet_ref);
        break;
      }
    }
  }

  // create new centerline
  std::unordered_map<lanelet::Id, lanelet::LineString3d> centerline_map;
  for (size_t traj_idx = 0; traj_idx < new_centerline.size(); ++traj_idx) {
    const auto & traj_pos = new_centerline.at(traj_idx).pose.position;
    const lanelet::BasicPoint2d point(traj_pos.x, traj_pos.y);
    const lanelet::Id centerline_lane_id = centerline_lane_ids.at(traj_idx);
    auto & lanelet_ref = lanelet_ref_map.at(centerline_lane_id);

    if (centerline_map.count(centerline_lane_id) == 0) {
      centerline_map.emplace(centerline_lane_id, lanelet::LineString3d{lanelet::utils::getId()});
    }

    // already checked by connect_centerline_to_lanelet, but double check.
    const bool is_inside = lanelet::geometry::inside(lanelet_ref, point);
    if (is_inside) {
      const auto center_point = createPoint3d(traj_pos.x, traj_pos.y, traj_pos.z);

      // set center point
      centerline_map.at(centerline_lane_id).push_back(center_point);
      lanelet_map_ptr->add(center_point);
    }

    if (
      traj_idx == new_centerline.size() - 1 ||
      centerline_lane_id != centerline_lane_ids.at(traj_idx + 1)) {
      const auto & centerline = centerline_map.at(centerline_lane_id);
      if (centerline.size() > 1) {
        lanelet_map_ptr->add(centerline);
        lanelet_ref.setCenterline(centerline);
      }
    }
  }
}

Marker create_footprint_marker(
  const std::string & ns, const LinearRing2d & footprint_poly, const double width, const double r,
  const double g, const double b, const double a, const rclcpp::Time & now, const size_t idx)
{
  auto marker = autoware::universe_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, idx, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware::universe_utils::createMarkerScale(width, 0.0, 0.0),
    autoware::universe_utils::createMarkerColor(r, g, b, a));
  marker.header.stamp = now;
  // TODO(murooka) Ideally, the following is unnecessary for the topic of transient local.
  marker.lifetime = rclcpp::Duration(0, 0);

  for (const auto & point : footprint_poly) {
    geometry_msgs::msg::Point geom_point;
    geom_point.x = point.x();
    geom_point.y = point.y();
    // geom_point.z = point.z();

    marker.points.push_back(geom_point);
  }
  marker.points.push_back(marker.points.front());
  return marker;
}

Marker create_text_marker(
  const std::string & ns, const geometry_msgs::msg::Pose & pose, const double value, const double r,
  const double g, const double b, const double a, const rclcpp::Time & now, const size_t idx)
{
  auto marker = autoware::universe_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, idx, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware::universe_utils::createMarkerScale(0.5, 0.5, 0.5),
    autoware::universe_utils::createMarkerColor(r, g, b, a));
  marker.pose = pose;
  marker.header.stamp = now;
  marker.lifetime = rclcpp::Duration(0, 0);

  std::stringstream ss;
  ss << std::setprecision(2) << value;
  marker.text = ss.str();

  return marker;
}

void create_points_marker(
  MarkerArray & marker_array, const std::string & ns,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & points_vec, const double width,
  const rclcpp::Time & now)
{
  for (size_t i = 0; i < points_vec.size(); ++i) {
    const auto color = (i % 2 == 0)
                         ? autoware::universe_utils::createMarkerColor(0.8, 0.5, 1.0, 0.8)
                         : autoware::universe_utils::createMarkerColor(1.0, 0.3, 0.5, 0.8);
    auto marker = autoware::universe_utils::createDefaultMarker(
      "map", now, ns, i, Marker::LINE_STRIP,
      autoware::universe_utils::createMarkerScale(width, 0.0, 0.0), color);
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.points = points_vec.at(i);
    marker_array.markers.push_back(marker);
  }
}

MarkerArray create_delete_all_marker_array(
  const std::vector<std::string> & ns_vec, const rclcpp::Time & now)
{
  Marker marker;
  marker.header.stamp = now;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  MarkerArray marker_array;
  for (const auto & ns : ns_vec) {
    marker.ns = ns;
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace utils
}  // namespace autoware::static_centerline_generator

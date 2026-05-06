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

#ifndef METRICS__GEOMETRY__LANELET_QUERIES_HPP_
#define METRICS__GEOMETRY__LANELET_QUERIES_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct DrivingDirectionLocalContext
{
  bool in_route_lane_polygon{false};
  bool in_lane_margin_only{false};
  bool in_intersection{false};
  lanelet::ConstLanelets route_lanelets;
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
};

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

lanelet::ConstLanelets collect_route_relevant_lanelets(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line);

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

bool is_pose_in_route_lane_polygon(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

std::optional<DrivingDirectionLocalContext> compute_driving_direction_local_context(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__LANELET_QUERIES_HPP_

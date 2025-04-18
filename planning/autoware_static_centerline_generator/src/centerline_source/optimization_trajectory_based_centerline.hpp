// Copyright 2024 TIER IV, Inc.
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

#ifndef CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT
#define CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT

#include "autoware/behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"
#include "autoware/behavior_path_goal_planner_module/fixed_goal_planner_base.hpp"
#include "autoware/path_generator/common_structs.hpp"
#include "autoware/path_generator/node.hpp"
#include "autoware/path_generator/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::static_centerline_generator
{
class OptimizationTrajectoryBasedCenterline
{
public:
  OptimizationTrajectoryBasedCenterline() = default;
  explicit OptimizationTrajectoryBasedCenterline(rclcpp::Node & node);
  std::vector<TrajectoryPoint> generate_centerline_with_optimization(
    rclcpp::Node & node, std::shared_ptr<RouteHandler> & route_handler_ptr,
    const std::vector<lanelet::Id> & route_lane_ids, LaneletMapBin::ConstSharedPtr & map_bin_ptr);

private:
  std::vector<TrajectoryPoint> optimize_trajectory(
    const PathWithLaneId & raw_path_with_lane_id, const Path & raw_path,
    std::shared_ptr<RouteHandler> & route_handler_ptr,
    LaneletMapBin::ConstSharedPtr & map_bin_ptr,
    const Pose & start_pose, const Pose & goal_pose) const;

  // publisher
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_raw_path_with_lane_id_{nullptr};
  rclcpp::Publisher<Path>::SharedPtr pub_raw_path_{nullptr};

  // data required for goal method
  std::pair<Pose, Pose> get_start_and_goal_pose(
    rclcpp::Node & node, const RouteHandler & route_handler,
    const std::vector<lanelet::Id> & route_lane_ids) const;
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute> create_route(
    std::shared_ptr<RouteHandler> & route_handler_ptr,
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) const;
  std::shared_ptr<autoware::path_generator::PathGenerator> create_path_generator_node(
    const geometry_msgs::msg::Pose current_pose, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
    std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute> route_ptr) const;
  std::shared_ptr<autoware::behavior_path_planner::PlannerData> create_behavior_path_planner_data(
    std::shared_ptr<RouteHandler> & route_handler_ptr) const;
  std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner> create_fixed_goal_planner(
    const PathWithLaneId & raw_path_with_lane_id) const;
  path_generator::Params create_params(
    std::shared_ptr<autoware::path_generator::PathGenerator> & path_generator_node) const;

  PathWithLaneId modify_goal_connection(
    const PathWithLaneId & raw_path_with_lane_id, const Path & raw_path,
    std::shared_ptr<RouteHandler> & route_handler_ptr, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const Pose & start_pose, const Pose & goal_pose) const;

  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute> route_ptr;
  std::string goal_method;
  double refine_goal_search_radius_range;
};
}  // namespace autoware::static_centerline_generator
// clang-format off
#endif  // CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT
// clang-format on

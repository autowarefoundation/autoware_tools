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

#include "centerline_source/optimization_trajectory_based_centerline.hpp"

#include "autoware/behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"
#include "autoware/behavior_path_goal_planner_module/fixed_goal_planner_base.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_optimizer/node.hpp"
#include "autoware/path_smoother/elastic_band_smoother.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"
#include "static_centerline_generator_node.hpp"
#include "utils.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::static_centerline_generator
{
namespace
{
rclcpp::NodeOptions create_node_options()
{
  return rclcpp::NodeOptions{};
}

Path convert_to_path(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.left_bound = path_with_lane_id.left_bound;
  path.right_bound = path_with_lane_id.right_bound;
  for (const auto & point : path_with_lane_id.points) {
    path.points.push_back(point.point);
  }

  return path;
}

std::vector<TrajectoryPoint> convert_to_trajectory_points(const PathWithLaneId & path_with_lane_id)
{
  Trajectory traj;
  for (const auto & point : path_with_lane_id.points) {
    TrajectoryPoint traj_point;
    traj_point.pose = point.point.pose;
    traj_point.longitudinal_velocity_mps = point.point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = point.point.lateral_velocity_mps;
    traj_point.heading_rate_rps = point.point.heading_rate_rps;

    traj.points.push_back(traj_point);
  }

  return autoware::motion_utils::convertToTrajectoryPointArray(traj);
}

std_msgs::msg::Header create_header(const rclcpp::Time & now)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = now;
  return header;
}
}  // namespace

OptimizationTrajectoryBasedCenterline::OptimizationTrajectoryBasedCenterline(rclcpp::Node & node)
{
  pub_raw_path_with_lane_id_ = node.create_publisher<PathWithLaneId>(
    "~/input_centerline", utils::create_transient_local_qos());
  pub_raw_path_ =
    node.create_publisher<Path>("~/debug/raw_centerline", utils::create_transient_local_qos());
  pub_iterative_path_ =
    node.create_publisher<Path>("~/debug/iterative_path", utils::create_transient_local_qos());
  pub_iterative_smoothed_traj_ = node.create_publisher<Trajectory>(
    "~/debug/iterative_smoothed_trajectory", utils::create_transient_local_qos());
  pub_iterative_optimized_traj_ = node.create_publisher<Trajectory>(
    "~/debug/iterative_optimized_trajectory", utils::create_transient_local_qos());
}

std::vector<TrajectoryPoint>
OptimizationTrajectoryBasedCenterline::generate_centerline_with_optimization(
  rclcpp::Node & node, std::shared_ptr<RouteHandler> & route_handler_ptr,
  LaneletMapBin::ConstSharedPtr & map_bin_ptr, const LaneletRoute & route)
{
  // get ego nearest search parameters and resample interval in behavior_path_planner
  const double ego_nearest_dist_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_yaw_threshold");
  const double behavior_path_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "output_path_interval");
  const double behavior_vel_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "behavior_output_path_interval");

  // update route_handler for the behavior_path_planner
  const auto route_lanelets = utils::get_lanelets_from_route(*route_handler_ptr, route);
  route_handler_ptr->setRoute(route);

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = [&]() {
    const auto non_resampled_path_with_lane_id = utils::get_path_with_lane_id(
      *route_handler_ptr, route_lanelets, route.start_pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
    return autoware::motion_utils::resamplePath(
      non_resampled_path_with_lane_id, behavior_path_interval);
  }();
  pub_raw_path_with_lane_id_->publish(raw_path_with_lane_id);
  RCLCPP_INFO(node.get_logger(), "Calculated raw path with lane id and published.");

  // visualize path
  // NOTE: path_with_lane_id is not used since the tier4_planning_rviz_plugin has an issue
  //       to die.
  const auto raw_path = [&]() {
    const auto non_resampled_path = convert_to_path(raw_path_with_lane_id);
    return autoware::motion_utils::resamplePath(non_resampled_path, behavior_vel_interval);
  }();
  pub_raw_path_->publish(raw_path);
  RCLCPP_INFO(node.get_logger(), "Converted to path and published.");

  // smooth trajectory and road collision avoidance
  const auto optimized_traj_points =
    optimize_trajectory(node, raw_path_with_lane_id, route_handler_ptr, map_bin_ptr, route);
  RCLCPP_INFO(
    node.get_logger(),
    "Smoothed trajectory and made it collision free with the road and published.");

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> OptimizationTrajectoryBasedCenterline::optimize_trajectory(
  rclcpp::Node & node, const PathWithLaneId & raw_path_with_lane_id,
  std::shared_ptr<RouteHandler> & route_handler_ptr, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const LaneletRoute & route) const
{
  const int wait_time_during_planning_iteration =
    autoware::universe_utils::getOrDeclareParameter<int>(
      node, "debug.wait_time_during_planning_iteration");

  // create an instance of elastic band and model predictive trajectory.
  const auto eb_path_smoother_ptr =
    autoware::path_smoother::ElasticBandSmoother(create_node_options()).getElasticBandSmoother();
  const auto mpt_optimizer_ptr =
    autoware::path_optimizer::PathOptimizer(create_node_options()).getMPTOptimizer();

  // NOTE: The optimization is executed every following number of points.
  constexpr int virtual_ego_pose_lon_shift_points_num = 1;

  // NOTE: num_initial_optimization exists to make the both optimizations stable since they may use
  // warm start.
  constexpr int num_initial_optimization = -2;

  // move the virtual_ego_pose forward following the raw_path_with_lane_id every cycle
  // and plan an optimized trajectory
  std::vector<TrajectoryPoint> whole_optimized_traj_points;
  for (int virtual_ego_pose_idx = num_initial_optimization;
       virtual_ego_pose_idx < static_cast<int>(raw_path_with_lane_id.points.size());
       virtual_ego_pose_idx += virtual_ego_pose_lon_shift_points_num) {
    // calculate virtual ego pose for the optimization
    const auto virtual_ego_pose =
      raw_path_with_lane_id.points.at(static_cast<size_t>(std::max(virtual_ego_pose_idx, 0)))
        .point.pose;

    // create path_with_lane_id by goal_method
    const auto path_with_lane_id_with_goal_connection = modify_goal_connection(
      node, raw_path_with_lane_id, route_handler_ptr, map_bin_ptr, route, virtual_ego_pose);
    if (path_with_lane_id_with_goal_connection.points.empty()) {
      continue;
    }

    // NOTE: path_with_lane_id is not used for the visualization since the
    // tier4_planning_rviz_plugin
    //       has an issue to die.
    auto path = convert_to_path(path_with_lane_id_with_goal_connection);
    path.header = create_header(node.get_clock()->now());
    pub_iterative_path_->publish(path);

    // convert trajectory
    const auto traj_points = convert_to_trajectory_points(path_with_lane_id_with_goal_connection);

    // smooth trajectory by elastic band in the autoware_path_smoother package
    const auto smoothed_traj_points =
      eb_path_smoother_ptr->smoothTrajectory(traj_points, virtual_ego_pose);
    pub_iterative_smoothed_traj_->publish(
      autoware::motion_utils::convertToTrajectory(
        smoothed_traj_points, create_header(node.get_clock()->now())));

    // road collision avoidance by model predictive trajectory in the autoware_path_optimizer
    // package
    const autoware::path_optimizer::PlannerData planner_data{
      raw_path_with_lane_id.header, smoothed_traj_points, raw_path_with_lane_id.left_bound,
      raw_path_with_lane_id.right_bound, virtual_ego_pose};
    const auto optimized_traj_points = mpt_optimizer_ptr->optimizeTrajectory(planner_data);
    if (!optimized_traj_points) {
      return whole_optimized_traj_points;
    }
    pub_iterative_optimized_traj_->publish(
      autoware::motion_utils::convertToTrajectory(
        *optimized_traj_points, create_header(node.get_clock()->now())));

    // connect the previously and currently optimized trajectory points
    // 1. generate valid_optimized_traj_points
    // NOTE: We assume that the trajectory before the ego pose is not valid
    //       since it tens to be inner curve due to the bug
    const auto valid_optimized_traj_points = [&]() {
      if (virtual_ego_pose_idx <= 0) {
        return *optimized_traj_points;
      }
      const size_t nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        *optimized_traj_points, virtual_ego_pose, 1.0, 0.35);
      return std::vector<TrajectoryPoint>(
        optimized_traj_points->begin() + nearest_idx, optimized_traj_points->end());
    }();

    // 2. fill whole_optimized_traj_points if it is empty.
    if (whole_optimized_traj_points.empty()) {
      whole_optimized_traj_points = valid_optimized_traj_points;
    }

    // 3. the whole_optimized_traj_points close to the valid_optimized_traj_points is removed, and
    // will be updated.
    if (!valid_optimized_traj_points.empty()) {
      const size_t nearest_segment_idx =
        autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
          whole_optimized_traj_points, valid_optimized_traj_points.front().pose, 1.0, 0.35);

      const std::vector<TrajectoryPoint> extracted_whole_optimized_traj_points{
        whole_optimized_traj_points.begin(),
        whole_optimized_traj_points.begin() + nearest_segment_idx};
      whole_optimized_traj_points = extracted_whole_optimized_traj_points;
    }

    // 4. register valid_optimized_traj_points
    for (const auto & valid_optimized_traj_point : valid_optimized_traj_points) {
      whole_optimized_traj_points.push_back(valid_optimized_traj_point);
    }

    // 5. finish if the valid_optimized_traj_point contains the goal.
    const double dist_to_goal =
      autoware::universe_utils::calcDistance2d(valid_optimized_traj_points.back(), route.goal_pose);
    if (dist_to_goal < 0.1) {
      break;
    }

    // wait for debugging purpose to visualize the iteration.
    if (1e-5 < static_cast<double>(wait_time_during_planning_iteration)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_during_planning_iteration));
    }
  }

  // remove the visualization of iterative trajectories
  Trajectory empty_traj;
  empty_traj.header = create_header(node.get_clock()->now());
  pub_iterative_smoothed_traj_->publish(empty_traj);
  pub_iterative_optimized_traj_->publish(empty_traj);

  Path empty_path;
  empty_path.header = create_header(node.get_clock()->now());
  pub_iterative_path_->publish(empty_path);

  return whole_optimized_traj_points;
}

std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute>
OptimizationTrajectoryBasedCenterline::create_route(
  std::shared_ptr<RouteHandler> & route_handler_ptr, const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & goal_pose) const
{
  // create route_ptr
  auto route_ptr = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();

  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets path_lanelets;
  route_handler_ptr->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets);

  for (const auto & lane : path_lanelets) {
    if (!all_lanelets.empty() && lane.id() == all_lanelets.back().id()) continue;
    all_lanelets.push_back(lane);
  }

  route_handler_ptr->setRouteLanelets(all_lanelets);

  std::vector<autoware_planning_msgs::msg::LaneletSegment> route_sections =
    route_handler_ptr->createMapSegments(all_lanelets);

  // set necessary data
  route_ptr->header.frame_id = "map";
  route_ptr->start_pose = start_pose;
  route_ptr->goal_pose = goal_pose;
  route_ptr->segments = route_sections;
  return route_ptr;
}

void OptimizationTrajectoryBasedCenterline::init_path_generator_node(
  const geometry_msgs::msg::Pose current_pose, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const LaneletRoute & route) const
{
  autoware::path_generator::PathGenerator::InputData path_generator_input;

  if (!path_generator_node_) {
    // initialize node, lanelet map and route
    path_generator_node_ =
      std::make_shared<autoware::path_generator::PathGenerator>(create_node_options());

    // NOTE: no need to register every time
    path_generator_input.lanelet_map_bin_ptr = map_bin_ptr;
    path_generator_input.route_ptr = std::make_shared<LaneletRoute>(route);
  }

  path_generator_input.odometry_ptr = utils::convert_to_odometry(current_pose);
  path_generator_node_->set_planner_data(path_generator_input);
}

std::shared_ptr<autoware::behavior_path_planner::PlannerData>
OptimizationTrajectoryBasedCenterline::create_behavior_path_planner_data(
  rclcpp::Node & node, std::shared_ptr<RouteHandler> & route_handler_ptr) const
{
  // create planner_data
  const auto refine_goal_search_radius_range =
    autoware::universe_utils::getOrDeclareParameter<double>(
      node, "refine_goal_search_radius_range");
  auto planner_data_ = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data_->route_handler = route_handler_ptr;
  planner_data_->parameters.refine_goal_search_radius_range = refine_goal_search_radius_range;
  return planner_data_;
}

std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner>
OptimizationTrajectoryBasedCenterline::create_behavior_path_fixed_goal_planner(
  const PathWithLaneId & raw_path_with_lane_id) const
{
  // create behavior_path_planner
  auto behavior_path_planner =
    std::make_shared<autoware::behavior_path_planner::DefaultFixedGoalPlanner>();

  autoware::behavior_path_planner::BehaviorModuleOutput behavior_module_input_data;
  behavior_module_input_data.path = raw_path_with_lane_id;
  behavior_path_planner->setPreviousModuleOutput(behavior_module_input_data);

  return behavior_path_planner;
}

PathWithLaneId OptimizationTrajectoryBasedCenterline::modify_goal_connection(
  rclcpp::Node & node, const PathWithLaneId & raw_path_with_lane_id,
  std::shared_ptr<RouteHandler> & route_handler_ptr, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const LaneletRoute & route, const Pose & current_pose) const
{
  const auto goal_method =
    autoware::universe_utils::getOrDeclareParameter<std::string>(node, "goal_method");
  if (goal_method == "None") {
    // No goal correction
    return raw_path_with_lane_id;
  }

  // NOTE: The route pointer has to be created to reflect start pose and goal pose
  if (goal_method == "path_generator") {
    init_path_generator_node(current_pose, map_bin_ptr, route);
    const auto param_listener = std::make_shared<::path_generator::ParamListener>(
      path_generator_node_->get_node_parameters_interface());
    const auto generated_path =
      path_generator_node_->generate_path(current_pose, param_listener->get_params());
    if (generated_path) {
      return *generated_path;
    }
    return PathWithLaneId{};
  }
  if (goal_method == "behavior_path_planner") {
    const auto planner_data = create_behavior_path_planner_data(node, route_handler_ptr);
    auto behavior_path_fixed_goal_planner =
      create_behavior_path_fixed_goal_planner(raw_path_with_lane_id);
    return behavior_path_fixed_goal_planner->plan(planner_data).path;
  }
  throw std::logic_error(
    "The goal_method is not supported in autoware_static_centerline_generator.");
}
}  // namespace autoware::static_centerline_generator

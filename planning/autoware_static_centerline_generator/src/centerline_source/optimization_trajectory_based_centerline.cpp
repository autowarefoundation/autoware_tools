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
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "static_centerline_generator_node.hpp"
#include "utils.hpp"

#include <algorithm>
#include <memory>
#include <string>
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
}  // namespace

OptimizationTrajectoryBasedCenterline::OptimizationTrajectoryBasedCenterline(rclcpp::Node & node)
{
  pub_raw_path_with_lane_id_ =
    node.create_publisher<PathWithLaneId>("input_centerline", utils::create_transient_local_qos());
  pub_raw_path_ =
    node.create_publisher<Path>("debug/raw_centerline", utils::create_transient_local_qos());
}

std::vector<TrajectoryPoint>
OptimizationTrajectoryBasedCenterline::generate_centerline_with_optimization(
  rclcpp::Node & node, std::shared_ptr<RouteHandler> & route_handler_ptr,
  const std::vector<lanelet::Id> & route_lane_ids, LaneletMapBin::ConstSharedPtr & map_bin_ptr)
{
  // get ego nearest search parameters, resample interval, search radius range, and goal method
  // in behavior_path_planner
  const double ego_nearest_dist_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_yaw_threshold");
  const double behavior_path_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "output_path_interval");
  const double behavior_vel_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "behavior_output_path_interval");
  refine_goal_search_radius_range = autoware::universe_utils::getOrDeclareParameter<double>(
    node, "refine_goal_search_radius_range");
  goal_method = autoware::universe_utils::getOrDeclareParameter<std::string>(node, "goal_method");

  // get start pose, goal pose, and goal method
  const auto [start_pose, goal_pose] = get_start_and_goal_pose(node, *route_handler_ptr, route_lane_ids);

  // update route_handler
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr, route_lane_ids);
  route_ptr_ = create_route_ptr(route_handler_ptr, start_pose, goal_pose);
  route_handler_ptr->setRouteLanelets(route_lanelets);
  route_handler_ptr->setRoute(*route_ptr_);

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = [&]() {
    const auto non_resampled_path_with_lane_id = utils::get_path_with_lane_id(
      *route_handler_ptr, route_lanelets, start_pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
    return autoware::motion_utils::resamplePath(
      non_resampled_path_with_lane_id, behavior_path_interval);
  }();
  pub_raw_path_with_lane_id_->publish(raw_path_with_lane_id);
  RCLCPP_INFO(node.get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = [&]() {
    const auto non_resampled_path = convert_to_path(raw_path_with_lane_id);
    return autoware::motion_utils::resamplePath(non_resampled_path, behavior_vel_interval);
  }();
  pub_raw_path_->publish(raw_path);
  RCLCPP_INFO(node.get_logger(), "Converted to path and published.");

  // smooth trajectory and road collision avoidance
  const auto optimized_traj_points =
    optimize_trajectory(raw_path_with_lane_id, raw_path, route_handler_ptr, map_bin_ptr, start_pose, goal_pose);
  RCLCPP_INFO(
    node.get_logger(),
    "Smoothed trajectory and made it collision free with the road and published.");

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> OptimizationTrajectoryBasedCenterline::optimize_trajectory(
  const PathWithLaneId & raw_path_with_lane_id, const Path & raw_path,
  std::shared_ptr<RouteHandler> & route_handler_ptr,
  LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const Pose & start_pose, const Pose & goal_pose) const
{
  // create path_with_lane_id by goal_method
  const PathWithLaneId path_with_lane_id_points = modify_goal_connection(
    raw_path_with_lane_id, raw_path, route_handler_ptr, map_bin_ptr, start_pose);

  // convert trajectory
  const auto traj_points = convert_to_trajectory_points(path_with_lane_id_points);

  // create an instance of elastic band and model predictive trajectory.
  const auto eb_path_smoother_ptr =
    autoware::path_smoother::ElasticBandSmoother(create_node_options()).getElasticBandSmoother();
  const auto mpt_optimizer_ptr =
    autoware::path_optimizer::PathOptimizer(create_node_options()).getMPTOptimizer();

  // NOTE: The optimization is executed every valid_optimized_traj_points_num points.
  constexpr int valid_optimized_traj_points_num = 10;
  const int traj_segment_num = traj_points.size() / valid_optimized_traj_points_num;

  // NOTE: num_initial_optimization exists to make the both optimizations stable since they may use
  // warm start.
  constexpr int num_initial_optimization = 2;

  std::vector<TrajectoryPoint> whole_optimized_traj_points;
  for (int virtual_ego_pose_idx = -num_initial_optimization;
       virtual_ego_pose_idx < traj_segment_num; ++virtual_ego_pose_idx) {
    // calculate virtual ego pose for the optimization
    constexpr int virtual_ego_pose_offset_idx = 1;
    const auto virtual_ego_pose =
      traj_points
        .at(
          valid_optimized_traj_points_num * std::max(virtual_ego_pose_idx, 0) +
          virtual_ego_pose_offset_idx)
        .pose;

    // create path_with_lane_id by goal_method
    const auto path_with_lane_id_points = modify_goal_connection(
      raw_path_with_lane_id, raw_path, route_handler_ptr, map_bin_ptr, virtual_ego_pose);

    // convert trajectory
    const auto traj_points = convert_to_trajectory_points(path_with_lane_id_points);

    // smooth trajectory by elastic band in the autoware_path_smoother package
    const auto smoothed_traj_points =
      eb_path_smoother_ptr->smoothTrajectory(traj_points, virtual_ego_pose);

    // road collision avoidance by model predictive trajectory in the autoware_path_optimizer
    // package
    const autoware::path_optimizer::PlannerData planner_data{
      raw_path.header, smoothed_traj_points, raw_path.left_bound, raw_path.right_bound,
      virtual_ego_pose};
    const auto optimized_traj_points = mpt_optimizer_ptr->optimizeTrajectory(planner_data);

    if (!optimized_traj_points) {
      return whole_optimized_traj_points;
    }

    // connect the previously and currently optimized trajectory points
    for (size_t j = 0; j < whole_optimized_traj_points.size(); ++j) {
      const double dist = autoware::universe_utils::calcDistance2d(
        whole_optimized_traj_points.at(j), optimized_traj_points->front());
      if (dist < 0.5) {
        const std::vector<TrajectoryPoint> extracted_whole_optimized_traj_points{
          whole_optimized_traj_points.begin(),
          whole_optimized_traj_points.begin() + std::max(j, 1UL) - 1};
        whole_optimized_traj_points = extracted_whole_optimized_traj_points;
        break;
      }
    }
    for (size_t j = 0; j < optimized_traj_points->size(); ++j) {
      whole_optimized_traj_points.push_back(optimized_traj_points->at(j));
    }
  }

  return whole_optimized_traj_points;
}

std::pair<Pose, Pose> OptimizationTrajectoryBasedCenterline::get_start_and_goal_pose(
  rclcpp::Node & node, const RouteHandler & route_handler,
  const std::vector<lanelet::Id> & route_lane_ids) const
{
  // get start pose, goal pose, goal method, and search radius range
  const std::vector<double> start_pose_input =
    autoware::universe_utils::getOrDeclareParameter<std::vector<double>>(node, "start_pose");
  const bool start_pose_check = std::all_of(
    start_pose_input.begin(), start_pose_input.end(), [](double x) { return x == 0.0; });
  const Pose start_pose = start_pose_check
    ? utils::get_center_pose(route_handler, route_lane_ids.front())
    : utils::create_pose(start_pose_input[0], start_pose_input[1], start_pose_input[2],
      start_pose_input[3], start_pose_input[4], start_pose_input[5], start_pose_input[6]);

  const std::vector<double> goal_pose_input =
    autoware::universe_utils::getOrDeclareParameter<std::vector<double>>(node, "end_pose");
  const Pose goal_pose = utils::create_pose(
    goal_pose_input[0], goal_pose_input[1], goal_pose_input[2], goal_pose_input[3],
    goal_pose_input[4], goal_pose_input[5], goal_pose_input[6]);
  return {start_pose, goal_pose};
}

<<<<<<< HEAD
std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>>
OptimizationTrajectoryBasedCenterline::create_route(
  const Path & raw_path, std::shared_ptr<RouteHandler> & route_handler_ptr,
  const Pose & start_pose, const Pose & goal_pose) const
{
  // create route_ptr
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>> route_ptr =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>>();
=======
std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute>
OptimizationTrajectoryBasedCenterline::create_route(
  std::shared_ptr<RouteHandler> & route_handler_ptr,
  const Pose & start_pose, const Pose & goal_pose) const
{
  // create route_ptr_
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute> route_ptr_ =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
>>>>>>> c5d1573 (refactoring version 1)

  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets path_lanelets;
  if (!route_handler_ptr->planPathLaneletsBetweenCheckpoints(
        start_pose, goal_pose, &path_lanelets)) { /* do nothing */
  }

  for (const auto & lane : path_lanelets) {
    if (!all_lanelets.empty() && lane.id() == all_lanelets.back().id()) continue;
    all_lanelets.push_back(lane);
  }

  route_handler_ptr->setRouteLanelets(all_lanelets);
  std::vector<autoware_planning_msgs::msg::LaneletSegment> route_sections =
    route_handler_ptr->createMapSegments(all_lanelets);

  // set necessary data
  route_ptr->header.frame_id = "map";
  route_ptr->set__start_pose(start_pose);
  route_ptr->set__goal_pose(goal_pose);
  route_ptr->set__segments(route_sections);
  return route_ptr;
}

std::shared_ptr<autoware::path_generator::PathGenerator>
<<<<<<< HEAD
OptimizationTrajectoryBasedCenterline::createPathGeneratorNode(
  const geometry_msgs::msg::Pose current_pose, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>> & route_ptr)
  const
=======
OptimizationTrajectoryBasedCenterline::create_path_generator_node(
  const Pose start_pose, LaneletMapBin::ConstSharedPtr & map_bin_ptr) const
>>>>>>> c5d1573 (refactoring version 1)
{
  const auto path_generator_node =
    std::make_shared<autoware::path_generator::PathGenerator>(create_node_options());

  // set data in path_generator
  autoware::path_generator::PathGenerator::InputData path_generator_input;
  path_generator_input.lanelet_map_bin_ptr = map_bin_ptr;
<<<<<<< HEAD
  path_generator_input.odometry_ptr = utils::convert_to_odometry(current_pose);
  path_generator_input.route_ptr = route_ptr;
=======
  path_generator_input.odometry_ptr = utils::convert_to_odometry(start_pose);
  path_generator_input.route_ptr = route_ptr_;
>>>>>>> c5d1573 (refactoring version 1)
  path_generator_node->set_planner_data(path_generator_input);

  return path_generator_node;
}

std::shared_ptr<autoware::behavior_path_planner::PlannerData>
<<<<<<< HEAD
OptimizationTrajectoryBasedCenterline::create_behavior_path_planner_data(
=======
OptimizationTrajectoryBasedCenterline::create_planner_data(
>>>>>>> c5d1573 (refactoring version 1)
  std::shared_ptr<RouteHandler> & route_handler_ptr) const
{
  // create planner_data
  std::shared_ptr<autoware::behavior_path_planner::PlannerData> planner_data_ =
    std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data_->route_handler = route_handler_ptr;
  planner_data_->parameters.refine_goal_search_radius_range = refine_goal_search_radius_range;
  return planner_data_;
}

std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner>
OptimizationTrajectoryBasedCenterline::create_fixed_goal_planner(
  const PathWithLaneId & raw_path_with_lane_id) const
{
  // create behavior_path_planner
  std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner> behavior_path_planner =
    std::make_shared<autoware::behavior_path_planner::DefaultFixedGoalPlanner>();

  autoware::behavior_path_planner::BehaviorModuleOutput behavior_module_input_data;
  behavior_module_input_data.path = raw_path_with_lane_id;
  behavior_path_planner->setPreviousModuleOutput(behavior_module_input_data);

  return behavior_path_planner;
}

path_generator::Params OptimizationTrajectoryBasedCenterline::create_params(
  std::shared_ptr<autoware::path_generator::PathGenerator> & path_generator_node) const
{
  // create params
  auto param_listener_ = std::make_shared<::path_generator::ParamListener>(
    path_generator_node->get_node_parameters_interface());
  return param_listener_->get_params();
}

<<<<<<< HEAD
PathWithLaneId OptimizationTrajectoryBasedCenterline::modify_goal_connection(
=======
PathWithLaneId OptimizationTrajectoryBasedCenterline::goal_path_generate(
>>>>>>> c5d1573 (refactoring version 1)
  const PathWithLaneId & raw_path_with_lane_id, const Path & raw_path,
  std::shared_ptr<RouteHandler> & route_handler_ptr, LaneletMapBin::ConstSharedPtr & map_bin_ptr,
  const Pose & start_pose, const Pose & goal_pose) const
{
  if (goal_method == "None") {
    // No goal correction
<<<<<<< HEAD
    return raw_path_with_lane_id;
=======
    path_with_lane_id_points = raw_path_with_lane_id;
  } else if (goal_method == "path_generator") {
    auto path_generator_node = create_path_generator_node(start_pose, map_bin_ptr);
    auto params = create_params(path_generator_node);
    path_with_lane_id_points =
      *(path_generator_node->generate_path(raw_path.points[0].pose, params));
  } else if (goal_method == "behavior_path_planner") {
    auto planner_data_ = create_planner_data(route_handler_ptr);
    auto behavior_path_planner = create_fixed_goal_planner(raw_path_with_lane_id);
    path_with_lane_id_points = behavior_path_planner->plan(planner_data_).path;
  } else {
    throw std::logic_error(
      "The goal_method is not supported in autoware_static_centerline_generator.");
>>>>>>> c5d1573 (refactoring version 1)
  }

  // NOTE: The route pointer has to be created to reflect start pose and goal pose
  auto route_ptr = create_route(raw_path, route_handler_ptr);
  if (goal_method == "path_generator") {
    auto path_generator_node = createPathGeneratorNode(current_pose, map_bin_ptr, route_ptr);
    const auto params = createParams(path_generator_node);
    return *(path_generator_node->generate_path(raw_path.points[0].pose, params));
  }
  if (goal_method == "behavior_path_planner") {
    route_handler_ptr->setRoute(*route_ptr);
    const auto planner_data = create_behavior_path_planner_data(route_handler_ptr);
    auto fixed_goal_planner = createFixedGoalPlanner(raw_path_with_lane_id);
    return fixed_goal_planner->plan(planner_data).path;
  }
  throw std::logic_error(
    "The goal_method is not supported in autoware_static_centerline_generator.");
}
}  // namespace autoware::static_centerline_generator

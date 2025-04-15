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
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_optimizer/node.hpp"
#include "autoware/path_smoother/elastic_band_smoother.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "static_centerline_generator_node.hpp"
#include "utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
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

Trajectory convert_to_trajectory(const Path & path)
{
  Trajectory traj;
  for (const auto & point : path.points) {
    TrajectoryPoint traj_point;
    traj_point.pose = point.pose;
    traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
    traj_point.heading_rate_rps = point.heading_rate_rps;

    traj.points.push_back(traj_point);
  }
  return traj;
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
  rclcpp::Node & node,
  std::shared_ptr<RouteHandler> & route_handler_ptr_,
  const std::vector<lanelet::Id> & route_lane_ids,
  LaneletMapBin::ConstSharedPtr & map_bin_ptr_)
{
  // set route lanelets
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);
  route_handler_ptr_->setRouteLanelets(route_lanelets);

  // get ego nearest search parameters and resample interval in behavior_path_planner
  const double ego_nearest_dist_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "ego_nearest_yaw_threshold");
  const double behavior_path_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "output_path_interval");
  const double behavior_vel_interval =
    autoware::universe_utils::getOrDeclareParameter<double>(node, "behavior_output_path_interval");

  // get start pose, goal pose, goal method, and search radius range
  const std::vector<double> start_pose_input =
    autoware::universe_utils::getOrDeclareParameter<std::vector<double>>(node, "start_pose");
  bool start_pose_check = std::all_of(start_pose_input.begin(), start_pose_input.end(), [](double x) {return x == 0.0;});
  if (start_pose_check) {
    // optimize centerline inside the lane
    start_pose = utils::get_center_pose(*route_handler_ptr_, route_lane_ids.front());
  } else {
    start_pose = utils::createPose(
      start_pose_input[0], start_pose_input[1], start_pose_input[2], start_pose_input[3], start_pose_input[4], start_pose_input[5], start_pose_input[6]);
  }
  const std::vector<double> goal_pose_input =
    autoware::universe_utils::getOrDeclareParameter<std::vector<double>>(node, "end_pose");
  goal_pose = utils::createPose(
    goal_pose_input[0], goal_pose_input[1], goal_pose_input[2], goal_pose_input[3], goal_pose_input[4], goal_pose_input[5], goal_pose_input[6]);
  const std::string goal_method = autoware::universe_utils::getOrDeclareParameter<std::string>(node, "goal_method");
  const double refine_goal_search_radius_range = autoware::universe_utils::getOrDeclareParameter<double>(node, "search_radius_range");

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = [&]() {
    const auto non_resampled_path_with_lane_id = utils::get_path_with_lane_id(
      *route_handler_ptr_, route_lanelets, start_pose, ego_nearest_dist_threshold,
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
  const auto optimized_traj_points = optimize_trajectory(
    raw_path_with_lane_id, raw_path, route_handler_ptr_, map_bin_ptr_, goal_method, refine_goal_search_radius_range);
  RCLCPP_INFO(
    node.get_logger(),
    "Smoothed trajectory and made it collision free with the road and published.");

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> OptimizationTrajectoryBasedCenterline::optimize_trajectory(
  const PathWithLaneId & raw_path_with_lane_id,
  const Path & raw_path,
  std::shared_ptr<RouteHandler> & route_handler_ptr_,
  LaneletMapBin::ConstSharedPtr & map_bin_ptr_,
  const std::string & goal_method,
  const double & refine_goal_search_radius_range) const
{
  // create path_with_lane_id by goal_method
  PathWithLaneId path_with_lane_id_points;
  if (goal_method == "") {
    // No goal correction
    path_with_lane_id_points = raw_path_with_lane_id;
  }
  else if (goal_method == "path_generator") {
    auto route_ptr_ = createRoutePtr(raw_path, route_handler_ptr_);
    auto path_generator_node = createPathGeneratorNode(start_pose, map_bin_ptr_, route_ptr_);
    auto params = createParams(path_generator_node);
    path_with_lane_id_points = *(path_generator_node->generate_path(raw_path.points[0].pose, params));
  }
  else if (goal_method == "fixed_goal_planner") {
    auto route_ptr_ = createRoutePtr(raw_path, route_handler_ptr_);
    route_handler_ptr_->setRoute(*route_ptr_);
    auto path_generator_node = createPathGeneratorNode(start_pose, map_bin_ptr_, route_ptr_);
    auto params = createParams(path_generator_node);
    auto planner_data_ = createPlannerData(route_handler_ptr_, params, refine_goal_search_radius_range);
    auto fixed_goal_planner = createFixedGoalPlanner(raw_path_with_lane_id);
    path_with_lane_id_points = fixed_goal_planner->plan(planner_data_).path;
  }
  else {
    throw std::logic_error(
      "The goal_method is not supported in autoware_static_centerline_generator.");
  }

  // convert trajectory
  auto path_points = convert_to_path(path_with_lane_id_points);
  const auto traj_points = [&]() {
    const auto raw_traj = convert_to_trajectory(path_points);
    return autoware::motion_utils::convertToTrajectoryPointArray(raw_traj);
  }();

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

    if (goal_method == "") {
      // No goal correction
      path_with_lane_id_points = raw_path_with_lane_id;
    }
    else if (goal_method == "path_generator") {
      auto route_ptr_ = createRoutePtr(raw_path, route_handler_ptr_);
      auto path_generator_node = createPathGeneratorNode(virtual_ego_pose, map_bin_ptr_, route_ptr_);
      auto params = createParams(path_generator_node);
      path_with_lane_id_points = *(path_generator_node->generate_path(raw_path.points[0].pose, params));
    }
    else if (goal_method == "fixed_goal_planner") {
      auto route_ptr_ = createRoutePtr(raw_path, route_handler_ptr_);
      route_handler_ptr_->setRoute(*route_ptr_);
      auto path_generator_node = createPathGeneratorNode(virtual_ego_pose, map_bin_ptr_, route_ptr_);
      auto params = createParams(path_generator_node);
      auto planner_data_ = createPlannerData(route_handler_ptr_, params, refine_goal_search_radius_range);
      auto fixed_goal_planner = createFixedGoalPlanner(raw_path_with_lane_id);
      path_with_lane_id_points = fixed_goal_planner->plan(planner_data_).path;
    }
    else {
      throw std::logic_error(
        "The goal_method is not supported in autoware_static_centerline_generator.");
    }
    auto path_points = convert_to_path(path_with_lane_id_points);
    const auto traj_points = [&]() {
      const auto raw_traj = convert_to_trajectory(path_points);
      return autoware::motion_utils::convertToTrajectoryPointArray(raw_traj);
    }();

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

std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>>
  OptimizationTrajectoryBasedCenterline::createRoutePtr(
  const Path & raw_path, std::shared_ptr<RouteHandler> & route_handler_ptr_) const
{
  // create route_ptr_
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>> route_ptr_ =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>>();
  
  lanelet::ConstLanelets all_lanelets;

  for (size_t i = 1; i < raw_path.points.size(); i++) {
    const auto start_point = raw_path.points.at(i - 1);
    const auto goal_point = raw_path.points.at(i);
    // std::cerr << "iketa goal_pose" << goal_point.pose.position.x << std::endl;
    lanelet::ConstLanelets path_lanelets;
    if(!route_handler_ptr_->planPathLaneletsBetweenCheckpoints(start_point.pose, goal_point.pose, &path_lanelets)) {
      // RCLCPP_WARN(node.get_logger(), "Failed to plan route.");
    }

    for (const auto & lane : path_lanelets) {
      if (!all_lanelets.empty() && lane.id() == all_lanelets.back().id()) continue;
      all_lanelets.push_back(lane);
    }
  }

  route_handler_ptr_->setRouteLanelets(all_lanelets);
  std::vector<autoware_planning_msgs::msg::LaneletSegment> route_sections = route_handler_ptr_->createMapSegments(all_lanelets);

  // set nessary data
  route_ptr_->header.frame_id = "map";
  route_ptr_->set__start_pose(start_pose);
  route_ptr_->set__goal_pose(goal_pose);
  route_ptr_->set__segments(route_sections);
  return route_ptr_;
}

std::shared_ptr<autoware::path_generator::PathGenerator>
  OptimizationTrajectoryBasedCenterline::createPathGeneratorNode(
  const geometry_msgs::msg::Pose current_pose,
  LaneletMapBin::ConstSharedPtr & map_bin_ptr_,
  std::shared_ptr<autoware_planning_msgs::msg::LaneletRoute_<std::allocator<void>>> & route_ptr_) const
{
  // create path_generator_node
  const auto path_generator_dir =
    ament_index_cpp::get_package_share_directory("autoware_path_generator");
  const auto node_options = rclcpp::NodeOptions{}.arguments(
    {"--ros-args", "--params-file",
     path_generator_dir + "/config/path_generator.param.yaml"});
  const auto path_generator_node = std::make_shared<autoware::path_generator::PathGenerator>(node_options);

  // set data in path_generator
  autoware::path_generator::PathGenerator::InputData path_generator_input;
  path_generator_input.lanelet_map_bin_ptr = map_bin_ptr_;
  path_generator_input.odometry_ptr = utils::convert_to_odometry(current_pose);
  path_generator_input.route_ptr = route_ptr_;
  path_generator_node->set_planner_data(path_generator_input);

  return path_generator_node;
}

std::shared_ptr<autoware::behavior_path_planner::PlannerData>
  OptimizationTrajectoryBasedCenterline::createPlannerData(
  std::shared_ptr<RouteHandler> & route_handler_ptr_,
  path_generator::Params & params,
  const double & refine_goal_search_radius_range) const
{
  // create planner_data
  std::shared_ptr<autoware::behavior_path_planner::PlannerData> planner_data_ =
    std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data_->route_handler = route_handler_ptr_;

  if (refine_goal_search_radius_range < 0) {
    // default
    planner_data_->parameters.refine_goal_search_radius_range = params.refine_goal_search_radius_range;
  } else {
    // input value
    planner_data_->parameters.refine_goal_search_radius_range = static_cast<double>(refine_goal_search_radius_range);
  }
  return planner_data_;
}

std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner>
  OptimizationTrajectoryBasedCenterline::createFixedGoalPlanner(
  const PathWithLaneId & raw_path_with_lane_id) const
{
  // create fixed_goal_planner
  std::shared_ptr<autoware::behavior_path_planner::DefaultFixedGoalPlanner> fixed_goal_planner =
    std::make_shared<autoware::behavior_path_planner::DefaultFixedGoalPlanner>();

  autoware::behavior_path_planner::BehaviorModuleOutput behavior_module_input_data;
  behavior_module_input_data.path = raw_path_with_lane_id;
  fixed_goal_planner->setPreviousModuleOutput(behavior_module_input_data);

  return fixed_goal_planner;
}

path_generator::Params OptimizationTrajectoryBasedCenterline::createParams(
  std::shared_ptr<autoware::path_generator::PathGenerator> & path_generator_node) const
{
  // create params
  auto param_listener_ =
    std::make_shared<::path_generator::ParamListener>(path_generator_node->get_node_parameters_interface());
  return param_listener_->get_params();
}
}  // namespace autoware::static_centerline_generator

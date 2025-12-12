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

#include "open_loop_evaluator.hpp"
#include "metrics/trajectory_metrics.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autoware::planning_data_analyzer
{

// Template helper for statistics calculation
template<typename Container>
struct Statistics {
  double mean = 0.0;
  double std_dev = 0.0;
  double max_val = 0.0;
};

template<typename Container>
Statistics<Container> calculate_statistics(const Container& values) {
  Statistics<Container> stats;
  
  if (values.empty()) {
    return stats;
  }
  
  // Calculate mean
  stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  
  // Calculate max
  stats.max_val = *std::max_element(values.begin(), values.end());
  
  // Calculate standard deviation
  double variance = 0.0;
  for (const auto& val : values) {
    variance += (val - stats.mean) * (val - stats.mean);
  }
  stats.std_dev = std::sqrt(variance / values.size());
  
  return stats;
}

// Constructor implementation moved to header file

void OpenLoopEvaluator::evaluate(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
  rosbag2_cpp::Writer * bag_writer)
{
  metrics_list_.clear();
  trajectory_point_metrics_list_.clear();
  // Reset normalized timestamp tracking for new evaluation
  first_bag_timestamp_set_ = false;
  
  // Get the base timestamp for relative time calculation
  rclcpp::Time base_timestamp;
  rclcpp::Time bag_base_timestamp;
  if (!synchronized_data_list.empty()) {
    base_timestamp = synchronized_data_list.front()->timestamp;
    bag_base_timestamp = synchronized_data_list.front()->bag_timestamp;
  } else {
    base_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bag_base_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  
  // Prepare evaluation data with ground truth trajectories
  auto evaluation_data_list = prepare_evaluation_data(synchronized_data_list);
  
  if (evaluation_data_list.empty()) {
    RCLCPP_WARN(logger_, "No valid trajectories with ground truth found for evaluation");
    return;
  }
    
  // Evaluate each trajectory with its ground truth
  for (const auto & eval_data : evaluation_data_list) {
    // Evaluate trajectory
    auto metrics = evaluate_trajectory(eval_data);
    metrics_list_.push_back(metrics);
    
    // Calculate trajectory point metrics
    auto trajectory_metrics = metrics::calculate_trajectory_point_metrics(
      eval_data.synchronized_data, route_handler_);
    trajectory_point_metrics_list_.push_back(trajectory_metrics);
    
    // Save to bag if writer provided
    if (bag_writer) {
      save_metrics_to_bag(metrics, eval_data, *bag_writer);
      
      // Calculate normalized timestamp for trajectory point metrics
      if (!first_bag_timestamp_set_) {
        first_bag_timestamp_ = eval_data.synchronized_data->bag_timestamp;
        first_bag_timestamp_set_ = true;
      }
      const auto relative_duration = eval_data.synchronized_data->bag_timestamp - first_bag_timestamp_;
      const rclcpp::Time normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME) + relative_duration;
      
      save_trajectory_point_metrics_to_bag(trajectory_metrics, *bag_writer, normalized_timestamp);
    } else {
      RCLCPP_WARN(logger_, "No bag writer provided, metrics not saved to bag");
    }
  }
  
  // Calculate summary statistics
  calculate_summary();
  
  RCLCPP_INFO(logger_, "Overall: Mean ADE=%.3fm (±%.3fm), Mean FDE=%.3fm (±%.3fm)",
    summary_.mean_ade, summary_.std_ade,
    summary_.mean_fde, summary_.std_fde);
}

std::vector<OpenLoopEvaluator::EvaluationData> OpenLoopEvaluator::prepare_evaluation_data(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list)
{
  std::vector<EvaluationData> result;
  
  for (const auto & data : synchronized_data_list) {
    // Skip if no trajectory available
    if (!data->trajectory || data->trajectory->points.empty()) {
      continue;
    }
    
    // Generate ground truth trajectory
    auto ground_truth_opt = generate_ground_truth_trajectory(data, synchronized_data_list);
    
    if (!ground_truth_opt.has_value()) {
      RCLCPP_WARN(logger_, "Skipping trajectory at time %f - ground truth generation failed",
        data->timestamp.seconds());
      continue;
    }
    
    // Add valid evaluation data
    result.push_back({data, ground_truth_opt.value()});
  }
  
  return result;
}

std::optional<autoware_planning_msgs::msg::Trajectory>
OpenLoopEvaluator::generate_ground_truth_trajectory(
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  const std::vector<std::shared_ptr<SynchronizedData>> & all_data)
{
  const auto & trajectory = *(trajectory_data->trajectory);

  autoware_planning_msgs::msg::Trajectory ground_truth_trajectory;
  ground_truth_trajectory.header = trajectory.header;
  ground_truth_trajectory.points.reserve(trajectory.points.size());

  // Check if trajectory has proper time_from_start values
  bool has_valid_timing = false;
  if (trajectory.points.size() >= 2) {
    const double last_time = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    has_valid_timing = (last_time > 0.1);  // At least 100ms total duration
  }

  if (!has_valid_timing && !trajectory.points.empty()) {
    // Workaround: Trajectory has invalid time_from_start (all zeros from DLR)
    // Use synchronized data index matching instead
    RCLCPP_WARN_ONCE(logger_, "Trajectory has invalid time_from_start values (all zeros). Using index-based GT matching.");

    // Find GT data closest to trajectory start
    const size_t traj_start_idx = [&]() {
      for (size_t i = 0; i < all_data.size(); ++i) {
        if (all_data[i]->timestamp.nanoseconds() == trajectory_data->timestamp.nanoseconds()) {
          return i;
        }
      }
      return size_t(0);
    }();

    // Match GT by index
    for (size_t idx = 0; idx < trajectory.points.size(); ++idx) {
      const size_t gt_idx = std::min(traj_start_idx + idx, all_data.size() - 1);

      if (gt_idx >= all_data.size() || !all_data[gt_idx]->kinematic_state) {
        return std::nullopt;
      }

      autoware_planning_msgs::msg::TrajectoryPoint gt_point;
      gt_point.pose = all_data[gt_idx]->kinematic_state->pose.pose;
      gt_point.time_from_start = trajectory.points[idx].time_from_start;
      gt_point.longitudinal_velocity_mps = trajectory.points[idx].longitudinal_velocity_mps;
      gt_point.lateral_velocity_mps = trajectory.points[idx].lateral_velocity_mps;
      gt_point.heading_rate_rps = trajectory.points[idx].heading_rate_rps;

      ground_truth_trajectory.points.push_back(gt_point);
    }

    return ground_truth_trajectory;
  }

  // Original time-based interpolation
  for (size_t idx = 0; idx < trajectory.points.size(); ++idx) {
    const auto & traj_point = trajectory.points[idx];

    autoware_planning_msgs::msg::TrajectoryPoint gt_point;

    // For first point (time_from_start=0), use synchronized kinematic_state directly
    // This ensures predicted first point matches current vehicle pose
    if (idx == 0 && trajectory_data->kinematic_state) {
      gt_point.pose = trajectory_data->kinematic_state->pose.pose;
      gt_point.time_from_start = traj_point.time_from_start;
    } else {
      // For subsequent points, interpolate from odometry
      const auto point_time = trajectory_data->timestamp +
        rclcpp::Duration(traj_point.time_from_start);

      // Interpolate ground truth pose at this time
      auto gt_pose_opt = interpolate_ground_truth(point_time, all_data);

      if (!gt_pose_opt.has_value()) {
        // Ground truth not available for this time point - fail entire trajectory
        RCLCPP_DEBUG(logger_, "Ground truth interpolation failed at time %f",
          point_time.seconds());
        return std::nullopt;
      }

      gt_point.pose = gt_pose_opt.value();
      gt_point.time_from_start = traj_point.time_from_start;
    }
    
    // Copy velocity if available from original trajectory  
    gt_point.longitudinal_velocity_mps = traj_point.longitudinal_velocity_mps;
    gt_point.lateral_velocity_mps = traj_point.lateral_velocity_mps;
    gt_point.heading_rate_rps = traj_point.heading_rate_rps;
    
    ground_truth_trajectory.points.push_back(gt_point);
  }

  return ground_truth_trajectory;
}

OpenLoopTrajectoryMetrics OpenLoopEvaluator::evaluate_trajectory(
  const EvaluationData & eval_data)
{
  OpenLoopTrajectoryMetrics metrics;
  
  const auto & trajectory = *(eval_data.synchronized_data->trajectory);
  const auto & ground_truth_trajectory = eval_data.ground_truth_trajectory;
  
  metrics.num_points = trajectory.points.size();
  metrics.trajectory_timestamp = eval_data.synchronized_data->timestamp;
  
  if (metrics.num_points == 0) {
    return metrics;
  }
  
  // Initialize vectors
  metrics.lateral_deviations.resize(metrics.num_points, 0.0);
  metrics.longitudinal_deviations.resize(metrics.num_points, 0.0);
  metrics.displacement_errors.resize(metrics.num_points, 0.0);
  metrics.ground_truth_poses.resize(metrics.num_points);
  metrics.ttc.resize(metrics.num_points, std::numeric_limits<double>::max());
  metrics.ade.resize(metrics.num_points, 0.0);
  
  // Evaluate each trajectory point against precomputed ground truth
  for (size_t i = 0; i < metrics.num_points; ++i) {
    const auto & traj_point = trajectory.points[i];
    const auto & gt_pose = ground_truth_trajectory.points[i].pose;
    
    // Store ground truth pose
    metrics.ground_truth_poses[i] = gt_pose;
    
    // Calculate displacement error
    metrics.displacement_errors[i] = autoware_utils_geometry::calc_distance2d(
      traj_point.pose.position, gt_pose.position);
    metrics.ade[i] = std::accumulate(
      metrics.displacement_errors.begin(), metrics.displacement_errors.begin() + i + 1, 0.0) / (i + 1);
    
    // Calculate errors in vehicle coordinate frame
    const auto [longitudinal_error, lateral_error] = 
      calculate_errors_in_vehicle_frame(traj_point.pose, gt_pose);
    
    metrics.longitudinal_deviations[i] = longitudinal_error;
    metrics.lateral_deviations[i] = lateral_error;
    
    // TTC calculation - using current objects from synchronized data
    // Note: This is a simplified approach using the objects from the current frame
    // In the future, we might want to interpolate objects as well
    // TODO(Phase 2): Re-implement TTC calculation without trajectory_selector_common dependency
    const auto & objects_at_time = eval_data.synchronized_data->objects;


    // Calculate TTC for this trajectory point
    // NOTE: TTC calculation temporarily disabled due to trajectory_selector_common removal
    /*
    if (objects_at_time && !objects_at_time->objects.empty()) {
      double ttc_at_point = std::numeric_limits<double>::max();

      for (const auto & object : objects_at_time->objects) {
        const double ttc = autoware::trajectory_selector::utils::time_to_collision(
          traj_point, traj_point.time_from_start, object);

        if (std::isfinite(ttc) && ttc > 0.0) {
          ttc_at_point = std::min(ttc_at_point, ttc);
        }
      }

      metrics.ttc[i] = ttc_at_point;
    }
    */
    (void)objects_at_time;  // Suppress unused variable warning
  }
    
  // Calculate trajectory duration
  if (!trajectory.points.empty()) {
    metrics.trajectory_duration = 
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
  }
  
  // Set evaluation time window
  metrics.evaluation_start_time = eval_data.synchronized_data->timestamp;
  if (!trajectory.points.empty()) {
    metrics.evaluation_end_time = eval_data.synchronized_data->timestamp + 
      rclcpp::Duration(trajectory.points.back().time_from_start);
  }
  
  return metrics;
}


// calculate_distance_2d moved to base class

std::pair<double, double> OpenLoopEvaluator::calculate_errors_in_vehicle_frame(
  const geometry_msgs::msg::Pose & trajectory_pose,
  const geometry_msgs::msg::Pose & ground_truth_pose)
{
  // Get ground truth yaw angle
  const double gt_yaw = tf2::getYaw(ground_truth_pose.orientation);
  
  // Calculate position difference in global frame
  const double dx_global = trajectory_pose.position.x - ground_truth_pose.position.x;
  const double dy_global = trajectory_pose.position.y - ground_truth_pose.position.y;
  
  // Transform to vehicle coordinate frame (ground truth vehicle frame)
  // Rotate by -gt_yaw to align with vehicle frame
  const double cos_yaw = std::cos(-gt_yaw);
  const double sin_yaw = std::sin(-gt_yaw);
  
  const double dx_vehicle = dx_global * cos_yaw - dy_global * sin_yaw;  // longitudinal
  const double dy_vehicle = dx_global * sin_yaw + dy_global * cos_yaw;  // lateral
  
  return std::make_pair(dx_vehicle, dy_vehicle);
}

std::optional<geometry_msgs::msg::Pose> OpenLoopEvaluator::interpolate_ground_truth(
  const rclcpp::Time & target_time,
  const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data)
{
  if (ground_truth_data.size() < 2) {
    return std::nullopt;
  }
  
  // Find bracketing indices
  size_t lower_idx = 0;
  size_t upper_idx = ground_truth_data.size() - 1;
  
  // Check bounds (use nanoseconds to avoid clock type mismatch)
  const int64_t target_ns = target_time.nanoseconds();
  const int64_t front_ns = ground_truth_data.front()->timestamp.nanoseconds();
  const int64_t back_ns = ground_truth_data.back()->timestamp.nanoseconds();

  if (target_ns < front_ns || target_ns > back_ns) {
    return std::nullopt;
  }

  // Binary search for bracketing indices (use nanoseconds)
  while (upper_idx - lower_idx > 1) {
    const size_t mid_idx = (lower_idx + upper_idx) / 2;
    const int64_t mid_ns = ground_truth_data[mid_idx]->timestamp.nanoseconds();
    if (mid_ns <= target_ns) {
      lower_idx = mid_idx;
    } else {
      upper_idx = mid_idx;
    }
  }
  
  const auto & lower_data = ground_truth_data[lower_idx];
  const auto & upper_data = ground_truth_data[upper_idx];

  // Calculate interpolation ratio (use nanoseconds to avoid clock type mismatch)
  const int64_t lower_ns = lower_data->timestamp.nanoseconds();
  const int64_t upper_ns = upper_data->timestamp.nanoseconds();

  const double dt_total = (upper_ns - lower_ns) / 1e9;
  const double dt_target = (target_ns - lower_ns) / 1e9;
  const double ratio = (dt_total > 0.0) ? (dt_target / dt_total) : 0.0;

  // Interpolate position
  geometry_msgs::msg::Pose interpolated_pose;
  const auto & p1 = lower_data->kinematic_state->pose.pose.position;
  const auto & p2 = upper_data->kinematic_state->pose.pose.position;
  
  interpolated_pose.position.x = p1.x + ratio * (p2.x - p1.x);
  interpolated_pose.position.y = p1.y + ratio * (p2.y - p1.y);
  interpolated_pose.position.z = p1.z + ratio * (p2.z - p1.z);
  
  // For orientation, use slerp (simplified to linear interpolation of yaw for 2D case)
  const double yaw1 = tf2::getYaw(lower_data->kinematic_state->pose.pose.orientation);
  const double yaw2 = tf2::getYaw(upper_data->kinematic_state->pose.pose.orientation);
  
  // Handle angle wrapping
  double yaw_diff = yaw2 - yaw1;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  
  const double interpolated_yaw = yaw1 + ratio * yaw_diff;
  
  // Convert back to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, interpolated_yaw);
  interpolated_pose.orientation = tf2::toMsg(q);
  
  return interpolated_pose;
}

void OpenLoopEvaluator::save_metrics_to_bag(
  const OpenLoopTrajectoryMetrics & metrics,
  const EvaluationData & eval_data,
  rosbag2_cpp::Writer & bag_writer)
{
  const auto & trajectory_data = eval_data.synchronized_data;
  const auto & ground_truth_trajectory = eval_data.ground_truth_trajectory;
  
  // Use a normalized timestamp for bag writing to ensure proper duration
  // Start from 0 and use relative times from the first synchronized data point
  if (!first_bag_timestamp_set_) {
    first_bag_timestamp_ = trajectory_data->bag_timestamp;
    first_bag_timestamp_set_ = true;
  }
  
  // Calculate relative timestamp from the first data point
  const auto relative_duration = trajectory_data->bag_timestamp - first_bag_timestamp_;
  const rclcpp::Time normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME) + relative_duration;
  
  // Write point-wise metrics as Float64MultiArray
  std_msgs::msg::Float64MultiArray array_msg;
  
  // ADE
  array_msg.data = metrics.ade;
  bag_writer.write(array_msg, "/open_loop/metrics/ade", normalized_timestamp);
  
  // FDE
  array_msg.data = metrics.displacement_errors;
  bag_writer.write(array_msg, "/open_loop/metrics/fde", normalized_timestamp);
   
  // Lateral deviations array
  array_msg.data = metrics.lateral_deviations;
  bag_writer.write(array_msg, "/open_loop/metrics/lateral_deviation", normalized_timestamp);
  
  // Longitudinal deviations array
  array_msg.data = metrics.longitudinal_deviations;
  bag_writer.write(array_msg, "/open_loop/metrics/longitudinal_deviation", normalized_timestamp);
  
  // TTC values array
  array_msg.data = metrics.ttc;
  bag_writer.write(array_msg, "/open_loop/metrics/ttc", normalized_timestamp);
  
  // Write minimal TF for visualization (map -> base_link)
  if (trajectory_data->kinematic_state) {
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    
    // Use the normalized timestamp for consistency
    transform.header.stamp = normalized_timestamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    
    // Use kinematic state pose as transform
    transform.transform.translation.x = trajectory_data->kinematic_state->pose.pose.position.x;
    transform.transform.translation.y = trajectory_data->kinematic_state->pose.pose.position.y;
    transform.transform.translation.z = trajectory_data->kinematic_state->pose.pose.position.z;
    transform.transform.rotation = trajectory_data->kinematic_state->pose.pose.orientation;
    
    tf_msg.transforms.push_back(transform);
    
    // Write TF with the normalized timestamp for consistency
    bag_writer.write(tf_msg, "/tf", normalized_timestamp);
    
  }
  
  // Save the original trajectory using base class method
  write_trajectory_to_bag(trajectory_data, bag_writer, normalized_timestamp);
  
  // Save the precomputed ground truth trajectory directly
  autoware_planning_msgs::msg::Trajectory gt_traj_msg = ground_truth_trajectory;
  gt_traj_msg.header.stamp = normalized_timestamp;
  
  if (!gt_traj_msg.points.empty()) {
    bag_writer.write(
      gt_traj_msg, "/open_loop/ground_truth_trajectory",
      normalized_timestamp);
  }
  
  // Save perception objects if available
  if (trajectory_data->objects) {
    autoware_perception_msgs::msg::PredictedObjects objects_msg = *(trajectory_data->objects);
    objects_msg.header.stamp = normalized_timestamp;
    bag_writer.write(
      objects_msg, "/perception/object_recognition/objects",
      normalized_timestamp);
  }
}


void OpenLoopEvaluator::calculate_summary()
{
  summary_ = OpenLoopEvaluationSummary{};
  
  if (metrics_list_.empty()) {
    return;
  }
  
  // Collect valid metrics
  std::vector<double> ade_values;
  std::vector<double> fde_values;
  std::vector<double> lateral_dev_values;
  
  for (const auto & metrics : metrics_list_) {
    // All trajectories in metrics_list_ have valid ground truth (guaranteed by prepare_evaluation_data)
    // Calculate ADE (average displacement error) from point-wise errors
    const double ade = metrics.ade.empty() ? 0.0 : metrics.ade.back();
    ade_values.push_back(ade);
    
    // FDE is the last displacement error
    const double fde = metrics.displacement_errors.empty() ? 0.0 : metrics.displacement_errors.back();
    fde_values.push_back(fde);
    
    // Calculate mean lateral deviation
    const double mean_lateral_dev = std::accumulate(
      metrics.lateral_deviations.begin(), metrics.lateral_deviations.end(), 0.0) / 
      (metrics.lateral_deviations.empty() ? 1.0 : metrics.lateral_deviations.size());
    lateral_dev_values.push_back(std::abs(mean_lateral_dev));
    
    summary_.fully_valid_trajectories++;
  }
  
  summary_.total_trajectories = metrics_list_.size();
  summary_.valid_trajectories = ade_values.size();
  
  if (!ade_values.empty()) {
    // ADE statistics
    const auto ade_stats = calculate_statistics(ade_values);
    summary_.mean_ade = ade_stats.mean;
    summary_.std_ade = ade_stats.std_dev;
    summary_.max_ade = ade_stats.max_val;
    
    // FDE statistics
    const auto fde_stats = calculate_statistics(fde_values);
    summary_.mean_fde = fde_stats.mean;
    summary_.std_fde = fde_stats.std_dev;
    summary_.max_fde = fde_stats.max_val;
    
    // Lateral deviation statistics
    const auto lateral_stats = calculate_statistics(lateral_dev_values);
    summary_.mean_lateral_deviation = lateral_stats.mean;
    summary_.std_lateral_deviation = lateral_stats.std_dev;
    summary_.max_lateral_deviation = lateral_stats.max_val;
    
    // Coverage statistics - always 1.0 since all trajectories have complete ground truth
    summary_.mean_coverage_ratio = 1.0;
  }
  
  // Calculate total evaluation duration
  if (!metrics_list_.empty()) {
    const auto start_time = metrics_list_.front().trajectory_timestamp;
    const auto end_time = metrics_list_.back().evaluation_end_time;
    summary_.total_evaluation_duration = (end_time - start_time).seconds();
  }
}

nlohmann::json OpenLoopEvaluator::get_summary_as_json() const
{
  nlohmann::json j;
  
  j["total_trajectories"] = summary_.total_trajectories;
  j["valid_trajectories"] = summary_.valid_trajectories;
  j["fully_valid_trajectories"] = summary_.fully_valid_trajectories;
  j["mean_coverage_ratio"] = summary_.mean_coverage_ratio;
  j["total_evaluation_duration_sec"] = summary_.total_evaluation_duration;
  
  j["ade"]["mean"] = summary_.mean_ade;
  j["ade"]["std"] = summary_.std_ade;
  j["ade"]["max"] = summary_.max_ade;
  
  j["fde"]["mean"] = summary_.mean_fde;
  j["fde"]["std"] = summary_.std_fde;
  j["fde"]["max"] = summary_.max_fde;
  
  j["lateral_deviation"]["mean"] = summary_.mean_lateral_deviation;
  j["lateral_deviation"]["std"] = summary_.std_lateral_deviation;
  j["lateral_deviation"]["max"] = summary_.max_lateral_deviation;
  
  return j;
}

nlohmann::json OpenLoopEvaluator::get_detailed_results_as_json() const
{
  nlohmann::json j;
  
  j["summary"] = get_summary_as_json();
  
  nlohmann::json trajectories = nlohmann::json::array();
  for (size_t i = 0; i < metrics_list_.size(); ++i) {
    const auto & metrics = metrics_list_[i];
    nlohmann::json traj;
    
    traj["timestamp_sec"] = metrics.trajectory_timestamp.seconds();
    traj["num_points"] = metrics.num_points;
    traj["trajectory_duration_sec"] = metrics.trajectory_duration;
    
    // Calculate aggregate metrics from point-wise data
    const double ade = metrics.ade.empty() ? 0.0 : metrics.ade.back();
    const double fde = metrics.displacement_errors.empty() ? 0.0 : metrics.displacement_errors.back();
    
    traj["ade"] = ade;
    traj["fde"] = fde;
    
    // Calculate lateral deviation statistics
    if (!metrics.lateral_deviations.empty()) {
      const double mean_lateral = std::accumulate(
        metrics.lateral_deviations.begin(), metrics.lateral_deviations.end(), 0.0) / 
        metrics.lateral_deviations.size();
      const double max_lateral = *std::max_element(
        metrics.lateral_deviations.begin(), metrics.lateral_deviations.end());
      
      traj["mean_lateral_deviation"] = std::abs(mean_lateral);
      traj["max_lateral_deviation"] = std::abs(max_lateral);
    }
    
    // Calculate min TTC
    const auto min_ttc_it = std::min_element(metrics.ttc.begin(), metrics.ttc.end());
    const double min_ttc = (min_ttc_it != metrics.ttc.end()) ? *min_ttc_it : std::numeric_limits<double>::max();
    traj["min_ttc"] = min_ttc;
    
    // Include point-wise data if needed
    traj["lateral_deviations"] = metrics.lateral_deviations;
    traj["displacement_errors"] = metrics.displacement_errors;
    
    // Add trajectory point metrics if available
    if (i < trajectory_point_metrics_list_.size()) {
      const auto & point_metrics = trajectory_point_metrics_list_[i];
      traj["trajectory_point_metrics"]["lateral_accelerations"] = point_metrics.lateral_accelerations;
      traj["trajectory_point_metrics"]["longitudinal_jerks"] = point_metrics.longitudinal_jerks;
      traj["trajectory_point_metrics"]["ttc_values"] = point_metrics.ttc_values;
      traj["trajectory_point_metrics"]["lateral_deviations"] = point_metrics.lateral_deviations;
      traj["trajectory_point_metrics"]["travel_distances"] = point_metrics.travel_distances;
    }
    
    trajectories.push_back(traj);
  }
  
  j["trajectories"] = trajectories;
  
  return j;
}

std::vector<std::pair<std::string, std::string>> OpenLoopEvaluator::get_result_topics() const
{
  return {
    {"/open_loop/metrics/ade", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/fde", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/ttc", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/lateral_deviation", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/longitudinal_deviation", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/trajectory_lateral_acceleration", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/trajectory_longitudinal_jerk", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/trajectory_ttc_values", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/trajectory_lateral_deviations", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/trajectory_travel_distances", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/open_loop/ground_truth_trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/perception/object_recognition/objects", "autoware_perception_msgs/msg/PredictedObjects"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
    {"/tf_static", "tf2_msgs/msg/TFMessage"}
  };
}

std::pair<rclcpp::Time, rclcpp::Time> OpenLoopEvaluator::run_evaluation_from_bag(
  const std::string & bag_path,
  rosbag2_cpp::Writer * evaluation_bag_writer,
  const TopicNames & topic_names)
{
  RCLCPP_INFO(logger_, "Running open-loop evaluation for trajectory analysis");
  
  // Use base class method to process bag and get synchronized data
  auto bag_result = process_bag_common(bag_path, evaluation_bag_writer, topic_names);
  
  // Run open-loop evaluation
  if (!bag_result.synchronized_data_list.empty()) {
    if (evaluation_bag_writer) {
      // Create topics for open-loop evaluation
      create_topics_in_bag(*evaluation_bag_writer);
      
      // Write tf_static with normalized timestamp
      write_tf_static_to_bag(evaluation_bag_writer, bag_result.tf_static_msgs);
      
      evaluate(bag_result.synchronized_data_list, evaluation_bag_writer);
    } else {
      evaluate(bag_result.synchronized_data_list, nullptr);
    }
    
    // Get and save evaluation results
    auto summary_json = get_summary_as_json();
    auto detailed_json = get_detailed_results_as_json();
    
    // Save using base class method
    save_json_results(detailed_json, bag_path, "open_loop", "open_loop_evaluation_results");
    
    // Log summary
    RCLCPP_INFO(logger_, "Open-loop evaluation summary:");
    if (summary_json.contains("ade") && summary_json["ade"].contains("mean")) {
      RCLCPP_INFO(logger_, "  Mean ADE: %.3f m", 
        static_cast<double>(summary_json["ade"]["mean"]));
      RCLCPP_INFO(logger_, "  Mean FDE: %.3f m", 
        static_cast<double>(summary_json["fde"]["mean"]));
    }
  }
  
  RCLCPP_INFO(logger_, "Open-loop evaluation complete");
  
  return {bag_result.evaluation_start_time, bag_result.evaluation_end_time};
}

}  // namespace autoware::planning_data_analyzer
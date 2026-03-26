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
#include <autoware_utils_math/normalization.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

// Template helper for statistics calculation
template <typename Container>
struct Statistics
{
  double mean = 0.0;
  double std_dev = 0.0;
  double max_val = 0.0;
};

template <typename Container>
Statistics<Container> calculate_statistics(const Container & values)
{
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
  for (const auto & val : values) {
    variance += (val - stats.mean) * (val - stats.mean);
  }
  stats.std_dev = std::sqrt(variance / values.size());

  return stats;
}

static double compute_mean(const std::vector<double> & v)
{
  return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

static double compute_std_dev(const std::vector<double> & v)
{
  if (v.size() < 2) return 0.0;
  const double m = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  double var = 0.0;
  for (const double x : v) var += (x - m) * (x - m);
  return std::sqrt(var / v.size());
}

static double compute_percentile(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) return 0.0;
  if (sorted.size() == 1) return sorted[0];
  const double idx = p * static_cast<double>(sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = std::min(lo + 1, sorted.size() - 1);
  const double frac = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

static void fill_horizon_json(
  nlohmann::json & j, const std::vector<std::pair<std::string, HorizonMetrics>> & horizons,
  size_t num_points)
{
  for (const auto & [key, hm] : horizons) {
    auto & f = j[key];
    f["Result"]["Total"] = "Success";
    f["Result"]["Frame"] = "Success";
    f["Info"]["ADE"] = hm.ade;
    f["Info"]["FDE"] = hm.fde;
    f["Info"]["AHE"] = hm.ahe;
    f["Info"]["FHE"] = hm.fhe;
    f["Info"]["average_lateral_deviation"] = hm.average_lateral_deviation;
    f["Info"]["max_lateral_deviation"] = hm.max_lateral_deviation;
    f["Info"]["average_longitudinal_deviation"] = hm.average_longitudinal_deviation;
    f["Info"]["max_longitudinal_deviation"] = hm.max_longitudinal_deviation;
    f["Info"]["min_ttc"] = hm.min_ttc;
    if (key == "full") {
      f["Info"]["num_points"] = num_points;
    }
  }
}

static std::string result_summary(bool has_frame, size_t num_points)
{
  if (has_frame) return "Open-loop trajectory metrics generated";
  return num_points <= 1u ? "Too few trajectory points"
                          : "Open-loop trajectory metrics unavailable";
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
    auto trajectory_metrics =
      metrics::calculate_trajectory_point_metrics(eval_data.synchronized_data, route_handler_);
    trajectory_point_metrics_list_.push_back(trajectory_metrics);

    // Save to bag if writer provided
    if (bag_writer) {
      save_metrics_to_bag(metrics, eval_data, *bag_writer);
      save_trajectory_point_metrics_to_bag_with_variant(
        trajectory_metrics, *bag_writer, eval_data.synchronized_data->bag_timestamp);
    } else {
      RCLCPP_WARN(logger_, "No bag writer provided, metrics not saved to bag");
    }
  }

  // Calculate summary statistics
  calculate_summary();
}

std::vector<OpenLoopEvaluator::EvaluationData> OpenLoopEvaluator::prepare_evaluation_data(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list)
{
  std::vector<EvaluationData> result;

  for (const auto & data : synchronized_data_list) {
    if (!data->trajectory) {
      continue;
    }

    const size_t num_pts = data->trajectory->points.size();
    if (num_pts <= 1u) {
      // Include frame in output as failure (no metrics); do not skip.
      result.push_back({data, autoware_planning_msgs::msg::Trajectory{}});
      continue;
    }

    std::optional<autoware_planning_msgs::msg::Trajectory> ground_truth_opt;
    if (gt_source_mode_ == GTSourceMode::GT_TRAJECTORY) {
      // In gt_trajectory mode, tolerate short startup timing gaps by skipping frames
      // where GT topic is missing/empty, instead of aborting the whole evaluation.
      if (!data->ground_truth_trajectory_msg || data->ground_truth_trajectory_msg->points.empty()) {
        RCLCPP_WARN(
          logger_,
          "Skipping trajectory at time %f in gt_trajectory mode - GT topic message was "
          "missing or empty.",
          data->timestamp.seconds());
        continue;
      }
      ground_truth_opt = generate_ground_truth_trajectory_from_topic(data);
      if (!ground_truth_opt.has_value()) {
        // For per-trajectory prediction issues (e.g., too few predicted points, invalid timing,
        // out-of-tolerance alignment), skip this frame and continue evaluating others.
        RCLCPP_WARN(
          logger_,
          "Skipping trajectory at time %f in gt_trajectory mode - predicted trajectory was "
          "invalid for evaluation (GT topic exists).",
          data->timestamp.seconds());
        continue;
      }
    } else {
      ground_truth_opt = generate_ground_truth_trajectory(data, synchronized_data_list);
    }

    if (!ground_truth_opt.has_value()) {
      RCLCPP_WARN(
        logger_, "Skipping trajectory at time %f - ground truth generation failed",
        data->timestamp.seconds());
      continue;
    }

    // Add valid evaluation data
    result.push_back({data, ground_truth_opt.value()});
  }

  return result;
}

std::optional<autoware_planning_msgs::msg::Trajectory>
OpenLoopEvaluator::generate_ground_truth_trajectory_from_topic(
  const std::shared_ptr<SynchronizedData> & trajectory_data) const
{
  if (!trajectory_data->trajectory || !trajectory_data->ground_truth_trajectory_msg) {
    return std::nullopt;
  }

  const auto & predicted = *trajectory_data->trajectory;
  const auto & gt_msg = *trajectory_data->ground_truth_trajectory_msg;
  if (predicted.points.empty() || gt_msg.points.empty()) {
    return std::nullopt;
  }

  autoware_planning_msgs::msg::Trajectory gt_for_eval;
  gt_for_eval.header = predicted.header;
  gt_for_eval.points.reserve(predicted.points.size());

  // In GT topic mode, require valid trajectory timing.
  if (predicted.points.size() < 2) {
    RCLCPP_WARN(logger_, "GT trajectory mode requires at least 2 predicted trajectory points.");
    return std::nullopt;
  }

  const double last_time = rclcpp::Duration(predicted.points.back().time_from_start).seconds();
  if (last_time <= 0.0) {
    RCLCPP_WARN(
      logger_, "GT trajectory mode requires positive trajectory horizon. got=%f", last_time);
    return std::nullopt;
  }

  double prev_time = -std::numeric_limits<double>::infinity();
  for (const auto & pred_point : predicted.points) {
    const double t = rclcpp::Duration(pred_point.time_from_start).seconds();
    if (t + 1e-9 < prev_time) {
      RCLCPP_WARN(
        logger_,
        "GT trajectory mode requires monotonic predicted time_from_start. prev=%f current=%f",
        prev_time, t);
      return std::nullopt;
    }
    prev_time = t;
  }

  for (const auto & pred_point : predicted.points) {
    const auto abs_time =
      rclcpp::Time(predicted.header.stamp) + rclcpp::Duration(pred_point.time_from_start);
    const auto gt_pose = interpolate_ground_truth_from_trajectory(abs_time, gt_msg);
    if (!gt_pose.has_value()) {
      return std::nullopt;
    }

    autoware_planning_msgs::msg::TrajectoryPoint gt_point;
    gt_point.pose = gt_pose.value();
    gt_point.time_from_start = pred_point.time_from_start;
    gt_point.longitudinal_velocity_mps = pred_point.longitudinal_velocity_mps;
    gt_point.lateral_velocity_mps = pred_point.lateral_velocity_mps;
    gt_point.heading_rate_rps = pred_point.heading_rate_rps;
    gt_for_eval.points.push_back(gt_point);
  }
  return gt_for_eval;
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
    RCLCPP_WARN_ONCE(
      logger_,
      "Trajectory has invalid time_from_start values (all zeros). Using index-based GT matching.");

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
      const auto point_time =
        trajectory_data->timestamp + rclcpp::Duration(traj_point.time_from_start);

      // Interpolate ground truth pose at this time
      auto gt_pose_opt = interpolate_ground_truth(point_time, all_data);

      if (!gt_pose_opt.has_value()) {
        // Ground truth not available for this time point - fail entire trajectory
        RCLCPP_DEBUG(logger_, "Ground truth interpolation failed at time %f", point_time.seconds());
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

OpenLoopTrajectoryMetrics OpenLoopEvaluator::evaluate_trajectory(const EvaluationData & eval_data)
{
  OpenLoopTrajectoryMetrics metrics;

  const auto & trajectory = *(eval_data.synchronized_data->trajectory);
  const auto & ground_truth_trajectory = eval_data.ground_truth_trajectory;

  metrics.num_points = trajectory.points.size();
  metrics.trajectory_timestamp = eval_data.synchronized_data->timestamp;
  metrics.bag_timestamp = eval_data.synchronized_data->bag_timestamp;

  if (metrics.num_points <= 1u) {
    if (metrics.num_points == 1u && !trajectory.points.empty()) {
      metrics.trajectory_duration =
        rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    }
    return metrics;
  }

  if (ground_truth_trajectory.points.size() != trajectory.points.size()) {
    return metrics;
  }

  // Initialize vectors
  metrics.lateral_deviations.resize(metrics.num_points, 0.0);
  metrics.longitudinal_deviations.resize(metrics.num_points, 0.0);
  metrics.displacement_errors.resize(metrics.num_points, 0.0);
  metrics.ground_truth_poses.resize(metrics.num_points);
  metrics.ade.resize(metrics.num_points, 0.0);
  metrics.ahe.resize(metrics.num_points, 0.0);
  metrics.heading_errors.resize(metrics.num_points, 0.0);
  metrics.ttc.resize(metrics.num_points, std::numeric_limits<double>::max());

  // Running accumulators for per-horizon aggregate metrics
  std::vector<double> lat_prefix_sum(metrics.num_points);
  std::vector<double> lon_prefix_sum(metrics.num_points);
  std::vector<double> heading_prefix_sum(metrics.num_points);
  std::vector<double> lat_running_abs_max(metrics.num_points);
  std::vector<double> lon_running_abs_max(metrics.num_points);
  std::vector<double> ttc_running_min(metrics.num_points);

  // Evaluate each trajectory point against precomputed ground truth
  double displacement_sum = 0.0;
  double lat_sum = 0.0;
  double lon_sum = 0.0;
  double heading_sum = 0.0;
  double lat_max = 0.0;
  double lon_max = 0.0;
  double ttc_min = std::numeric_limits<double>::max();

  for (size_t i = 0; i < metrics.num_points; ++i) {
    const auto & traj_point = trajectory.points[i];
    const auto & gt_pose = ground_truth_trajectory.points[i].pose;

    metrics.ground_truth_poses[i] = gt_pose;

    metrics.displacement_errors[i] =
      autoware_utils_geometry::calc_distance2d(traj_point.pose.position, gt_pose.position);
    displacement_sum += metrics.displacement_errors[i];
    metrics.ade[i] = displacement_sum / (i + 1);

    const auto [longitudinal_error, lateral_error] =
      calculate_errors_in_vehicle_frame(traj_point.pose, gt_pose);
    const double pred_yaw = tf2::getYaw(traj_point.pose.orientation);
    const double gt_yaw = tf2::getYaw(gt_pose.orientation);
    const double heading_error =
      std::abs(autoware_utils_math::normalize_radian(pred_yaw - gt_yaw));

    metrics.longitudinal_deviations[i] = longitudinal_error;
    metrics.lateral_deviations[i] = lateral_error;
    metrics.heading_errors[i] = heading_error;

    lat_sum += std::abs(lateral_error);
    lon_sum += std::abs(longitudinal_error);
    heading_sum += heading_error;
    metrics.ahe[i] = heading_sum / (i + 1);
    lat_max = std::max(lat_max, std::abs(lateral_error));
    lon_max = std::max(lon_max, std::abs(longitudinal_error));
    lat_prefix_sum[i] = lat_sum;
    lon_prefix_sum[i] = lon_sum;
    heading_prefix_sum[i] = heading_sum;
    lat_running_abs_max[i] = lat_max;
    lon_running_abs_max[i] = lon_max;

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
    (void)objects_at_time;

    ttc_min = std::min(ttc_min, metrics.ttc[i]);
    ttc_running_min[i] = ttc_min;
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

  // Pre-compute per-horizon metrics
  constexpr double kHorizonToleranceSec = 0.1;
  auto compute_horizon = [&](size_t cutoff) -> HorizonMetrics {
    const double n = static_cast<double>(cutoff + 1);
    return {metrics.ade[cutoff],
            metrics.displacement_errors[cutoff],
            heading_prefix_sum[cutoff] / n,
            metrics.heading_errors[cutoff],
            lat_prefix_sum[cutoff] / n,
            lat_running_abs_max[cutoff],
            lon_prefix_sum[cutoff] / n,
            lon_running_abs_max[cutoff],
            ttc_running_min[cutoff]};
  };

  if (metrics.num_points > 1u) {
    metrics.horizon_results.emplace_back("full", compute_horizon(metrics.num_points - 1));
  }

  for (const double horizon : evaluation_horizons_) {
    std::optional<size_t> cutoff;
    for (size_t i = 0; i < metrics.num_points; ++i) {
      const double t = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
      if (t <= horizon + 1e-6) {
        cutoff = i;
      } else {
        break;
      }
    }
    if (!cutoff.has_value()) continue;
    const double actual_t = rclcpp::Duration(trajectory.points[*cutoff].time_from_start).seconds();
    if (horizon - actual_t > kHorizonToleranceSec) continue;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << horizon << "s";
    metrics.horizon_results.emplace_back(oss.str(), compute_horizon(*cutoff));
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
    const size_t mid_idx = lower_idx + (upper_idx - lower_idx) / 2;
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

std::optional<geometry_msgs::msg::Pose> OpenLoopEvaluator::interpolate_ground_truth_from_trajectory(
  const rclcpp::Time & target_time,
  const autoware_planning_msgs::msg::Trajectory & gt_trajectory) const
{
  if (gt_trajectory.points.empty()) {
    return std::nullopt;
  }

  const auto gt_start = rclcpp::Time(gt_trajectory.header.stamp);
  std::vector<rclcpp::Time> gt_abs_times;
  gt_abs_times.reserve(gt_trajectory.points.size());
  for (const auto & pt : gt_trajectory.points) {
    gt_abs_times.push_back(gt_start + rclcpp::Duration(pt.time_from_start));
  }

  const auto target_ns = target_time.nanoseconds();
  const auto front_ns = gt_abs_times.front().nanoseconds();
  const auto back_ns = gt_abs_times.back().nanoseconds();
  const auto tolerance_ns = static_cast<int64_t>(std::llround(gt_sync_tolerance_ms_ * 1e6));
  if (target_ns < front_ns) {
    if (front_ns - target_ns <= tolerance_ns) {
      return gt_trajectory.points.front().pose;
    }
    return std::nullopt;
  }
  if (target_ns > back_ns) {
    if (target_ns - back_ns <= tolerance_ns) {
      return gt_trajectory.points.back().pose;
    }
    return std::nullopt;
  }

  if (target_ns == front_ns) {
    return gt_trajectory.points.front().pose;
  }
  if (target_ns == back_ns) {
    return gt_trajectory.points.back().pose;
  }

  size_t lower_idx = 0;
  size_t upper_idx = gt_abs_times.size() - 1;
  while (upper_idx - lower_idx > 1) {
    const size_t mid = lower_idx + (upper_idx - lower_idx) / 2;
    if (gt_abs_times[mid].nanoseconds() <= target_ns) {
      lower_idx = mid;
    } else {
      upper_idx = mid;
    }
  }

  const auto lower_ns = gt_abs_times[lower_idx].nanoseconds();
  const auto upper_ns = gt_abs_times[upper_idx].nanoseconds();
  const auto lower_dist_ns = target_ns - lower_ns;
  const auto upper_dist_ns = upper_ns - target_ns;
  if (std::min(lower_dist_ns, upper_dist_ns) > tolerance_ns) {
    return std::nullopt;
  }
  const double dt_total = static_cast<double>(upper_ns - lower_ns) / 1e9;
  const double dt_target = static_cast<double>(target_ns - lower_ns) / 1e9;
  const double ratio = dt_total > 0.0 ? dt_target / dt_total : 0.0;

  const auto & p1 = gt_trajectory.points[lower_idx].pose;
  const auto & p2 = gt_trajectory.points[upper_idx].pose;
  geometry_msgs::msg::Pose pose;
  pose.position.x = p1.position.x + ratio * (p2.position.x - p1.position.x);
  pose.position.y = p1.position.y + ratio * (p2.position.y - p1.position.y);
  pose.position.z = p1.position.z + ratio * (p2.position.z - p1.position.z);

  const double yaw1 = tf2::getYaw(p1.orientation);
  const double yaw2 = tf2::getYaw(p2.orientation);
  double yaw_diff = yaw2 - yaw1;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  const double yaw_interp = yaw1 + ratio * yaw_diff;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_interp);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

void OpenLoopEvaluator::save_metrics_to_bag(
  const OpenLoopTrajectoryMetrics & metrics, const EvaluationData & eval_data,
  rosbag2_cpp::Writer & bag_writer)
{
  const auto & ground_truth_trajectory = eval_data.ground_truth_trajectory;
  const auto & trajectory_data = eval_data.synchronized_data;
  const rclcpp::Time message_timestamp = trajectory_data->bag_timestamp;

  // Write point-wise metrics as Float64MultiArray
  std_msgs::msg::Float64MultiArray array_msg;

  // ADE
  array_msg.data = metrics.ade;
  bag_writer.write(array_msg, metric_topic("ade"), message_timestamp);

  // FDE
  array_msg.data = metrics.displacement_errors;
  bag_writer.write(array_msg, metric_topic("fde"), message_timestamp);

  // AHE
  array_msg.data = metrics.ahe;
  bag_writer.write(array_msg, metric_topic("ahe"), message_timestamp);

  // FHE
  array_msg.data = metrics.heading_errors;
  bag_writer.write(array_msg, metric_topic("fhe"), message_timestamp);

  // Lateral deviations array
  array_msg.data = metrics.lateral_deviations;
  bag_writer.write(array_msg, metric_topic("lateral_deviation"), message_timestamp);

  // Longitudinal deviations array
  array_msg.data = metrics.longitudinal_deviations;
  bag_writer.write(array_msg, metric_topic("longitudinal_deviation"), message_timestamp);

  // TTC values array
  array_msg.data = metrics.ttc;
  bag_writer.write(array_msg, metric_topic("ttc"), message_timestamp);

  save_dlr_style_result_to_bag(metrics, eval_data, bag_writer);

  // Save the precomputed ground truth trajectory directly
  autoware_planning_msgs::msg::Trajectory gt_traj_msg = ground_truth_trajectory;
  gt_traj_msg.header.stamp = message_timestamp;

  if (!gt_traj_msg.points.empty()) {
    bag_writer.write(gt_traj_msg, compared_trajectory_topic(), message_timestamp);
  }
}

void OpenLoopEvaluator::save_trajectory_point_metrics_to_bag_with_variant(
  const metrics::TrajectoryPointMetrics & metrics, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & normalized_timestamp) const
{
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.lateral_accelerations;
    bag_writer.write(msg, trajectory_metric_topic("lateral_accelerations"), normalized_timestamp);
  }

  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.longitudinal_jerks;
    bag_writer.write(msg, trajectory_metric_topic("longitudinal_jerks"), normalized_timestamp);
  }

  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.lateral_deviations;
    bag_writer.write(msg, trajectory_metric_topic("lateral_deviations"), normalized_timestamp);
  }

  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.travel_distances;
    bag_writer.write(msg, trajectory_metric_topic("travel_distances"), normalized_timestamp);
  }
}

std::string OpenLoopEvaluator::metric_topic(const std::string & metric_name) const
{
  if (metric_variant_.empty()) {
    return "/open_loop/metrics/" + metric_name;
  }
  return "/open_loop/metrics/" + metric_variant_ + "/" + metric_name;
}

std::string OpenLoopEvaluator::trajectory_metric_topic(const std::string & metric_name) const
{
  if (metric_variant_.empty()) {
    return "/trajectory/" + metric_name;
  }
  return "/trajectory/" + metric_variant_ + "/" + metric_name;
}

std::string OpenLoopEvaluator::compared_trajectory_topic() const
{
  if (metric_variant_.empty()) {
    return "/evaluation/compared_trajectory";
  }
  return "/evaluation/compared_trajectory/" + metric_variant_;
}

std::string OpenLoopEvaluator::dlr_result_topic() const
{
  if (metric_variant_.empty()) {
    return "/driving_log_replayer/time_step_based_trajectory/results";
  }
  return "/driving_log_replayer/time_step_based_trajectory/" + metric_variant_ + "/results";
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
    if (metrics.num_points <= 1u) {
      continue;
    }
    const double ade = metrics.ade.empty() ? 0.0 : metrics.ade.back();
    ade_values.push_back(ade);

    const double fde =
      metrics.displacement_errors.empty() ? 0.0 : metrics.displacement_errors.back();
    fde_values.push_back(fde);

    const double mean_lateral_dev =
      std::accumulate(metrics.lateral_deviations.begin(), metrics.lateral_deviations.end(), 0.0) /
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

  auto emit_metric = [&](
                       const std::string & horizon_key, const std::string & metric_name,
                       const std::string & desc, const std::vector<double> & vals) {
    const std::string prefix = horizon_key + "/" + metric_name;
    std::vector<double> sorted = vals;
    std::sort(sorted.begin(), sorted.end());
    j[prefix + "/description"] = desc;
    j[prefix + "/mean"] = compute_mean(vals);
    j[prefix + "/min"] = sorted.empty() ? 0.0 : sorted.front();
    j[prefix + "/max"] = sorted.empty() ? 0.0 : sorted.back();
    j[prefix + "/std"] = compute_std_dev(vals);
    j[prefix + "/percentile_95"] = compute_percentile(sorted, 0.95);
    j[prefix + "/percentile_99"] = compute_percentile(sorted, 0.99);
  };

  std::set<std::string> all_keys;
  for (const auto & m : metrics_list_) {
    for (const auto & p : m.horizon_results) all_keys.insert(p.first);
  }
  for (const auto & tag : all_keys) {
    std::vector<double> ade_vals, fde_vals, ahe_vals, fhe_vals;
    std::vector<double> avg_lat_vals, max_lat_vals, avg_lon_vals, max_lon_vals;
    std::vector<double> min_ttc_vals;
    for (const auto & m : metrics_list_) {
      const auto it = std::find_if(
        m.horizon_results.begin(), m.horizon_results.end(),
        [&tag](const auto & p) { return p.first == tag; });
      if (it == m.horizon_results.end()) continue;
      const auto & hm = it->second;
      ade_vals.push_back(hm.ade);
      fde_vals.push_back(hm.fde);
      ahe_vals.push_back(hm.ahe);
      fhe_vals.push_back(hm.fhe);
      avg_lat_vals.push_back(hm.average_lateral_deviation);
      max_lat_vals.push_back(hm.max_lateral_deviation);
      avg_lon_vals.push_back(hm.average_longitudinal_deviation);
      max_lon_vals.push_back(hm.max_longitudinal_deviation);
      min_ttc_vals.push_back(hm.min_ttc);
    }

    j[tag + "/count"] = ade_vals.size();

    emit_metric(tag, "ade", "Average Displacement Error within " + tag + " horizon [m]", ade_vals);
    emit_metric(tag, "fde", "Final Displacement Error at " + tag + " horizon [m]", fde_vals);
    emit_metric(tag, "ahe", "Average Heading Error within " + tag + " horizon [rad]", ahe_vals);
    emit_metric(tag, "fhe", "Final Heading Error at " + tag + " horizon [rad]", fhe_vals);
    emit_metric(
      tag, "average_lateral_deviation", "Average lateral deviation within " + tag + " horizon [m]",
      avg_lat_vals);
    emit_metric(
      tag, "max_lateral_deviation", "Max lateral deviation within " + tag + " horizon [m]",
      max_lat_vals);
    emit_metric(
      tag, "average_longitudinal_deviation",
      "Average longitudinal deviation within " + tag + " horizon [m]", avg_lon_vals);
    emit_metric(
      tag, "max_longitudinal_deviation",
      "Max longitudinal deviation within " + tag + " horizon [m]", max_lon_vals);
    emit_metric(
      tag, "min_ttc", "Minimum Time To Collision within " + tag + " horizon [s]", min_ttc_vals);
  }

  return j;
}

nlohmann::json OpenLoopEvaluator::get_detailed_results_as_json() const
{
  nlohmann::json results = nlohmann::json::array();

  for (const auto & metrics : metrics_list_) {
    const bool has_frame = !metrics.horizon_results.empty();
    nlohmann::json entry;
    entry["Result"]["Success"] = has_frame;
    entry["Result"]["Summary"] = result_summary(has_frame, metrics.num_points);
    entry["Stamp"]["System"] = metrics.bag_timestamp.seconds();
    entry["Stamp"]["ROS"] = metrics.trajectory_timestamp.seconds();
    entry["Frame"] = nlohmann::json::object();
    fill_horizon_json(entry["Frame"], metrics.horizon_results, metrics.num_points);
    results.push_back(entry);
  }

  nlohmann::json j;
  j["results"] = results;
  return j;
}

nlohmann::json OpenLoopEvaluator::get_full_results_as_json() const
{
  nlohmann::json j;
  j["summary"] = get_summary_as_json();

  nlohmann::json trajectories = nlohmann::json::array();
  for (size_t i = 0; i < metrics_list_.size(); ++i) {
    const auto & m = metrics_list_[i];
    nlohmann::json traj;

    traj["timestamp_sec"] = m.trajectory_timestamp.seconds();
    traj["bag_timestamp_sec"] = m.bag_timestamp.seconds();
    traj["num_points"] = m.num_points;
    traj["trajectory_duration_sec"] = m.trajectory_duration;

    traj["ade"] = m.ade;
    traj["ahe"] = m.ahe;
    traj["displacement_errors"] = m.displacement_errors;
    traj["heading_errors"] = m.heading_errors;
    traj["lateral_deviations"] = m.lateral_deviations;
    traj["longitudinal_deviations"] = m.longitudinal_deviations;
    traj["ttc"] = m.ttc;

    traj["horizon_results"] = nlohmann::json::object();
    fill_horizon_json(traj["horizon_results"], m.horizon_results, m.num_points);

    if (i < trajectory_point_metrics_list_.size()) {
      const auto & pm = trajectory_point_metrics_list_[i];
      traj["trajectory_point_metrics"]["lateral_accelerations"] = pm.lateral_accelerations;
      traj["trajectory_point_metrics"]["longitudinal_jerks"] = pm.longitudinal_jerks;
      traj["trajectory_point_metrics"]["ttc_values"] = pm.ttc_values;
      traj["trajectory_point_metrics"]["lateral_deviations"] = pm.lateral_deviations;
      traj["trajectory_point_metrics"]["travel_distances"] = pm.travel_distances;
    }

    trajectories.push_back(traj);
  }
  j["trajectories"] = trajectories;

  return j;
}

void OpenLoopEvaluator::save_dlr_style_result_to_bag(
  const OpenLoopTrajectoryMetrics & metrics, const EvaluationData & eval_data,
  rosbag2_cpp::Writer & bag_writer)
{
  const auto & trajectory_data = eval_data.synchronized_data;
  if (!trajectory_data || !trajectory_data->trajectory) {
    return;
  }

  const bool has_frame = !metrics.horizon_results.empty();
  nlohmann::json result_json;
  result_json["Result"]["Success"] = has_frame;
  result_json["Result"]["Summary"] = result_summary(has_frame, metrics.num_points);
  result_json["Stamp"]["System"] = trajectory_data->bag_timestamp.seconds();
  result_json["Stamp"]["ROS"] = trajectory_data->timestamp.seconds();
  result_json["Frame"] = nlohmann::json::object();
  fill_horizon_json(result_json["Frame"], metrics.horizon_results, metrics.num_points);

  std_msgs::msg::String result_msg;
  result_msg.data = result_json.dump();
  bag_writer.write(result_msg, dlr_result_topic(), trajectory_data->bag_timestamp);
}

std::vector<std::pair<std::string, std::string>> OpenLoopEvaluator::get_result_topics() const
{
  return {
    {dlr_result_topic(), "std_msgs/msg/String"},
    {metric_topic("ade"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("fde"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("ahe"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("fhe"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("ttc"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("lateral_deviation"), "std_msgs/msg/Float64MultiArray"},
    {metric_topic("longitudinal_deviation"), "std_msgs/msg/Float64MultiArray"},
    {trajectory_metric_topic("lateral_accelerations"), "std_msgs/msg/Float64MultiArray"},
    {trajectory_metric_topic("longitudinal_jerks"), "std_msgs/msg/Float64MultiArray"},
    {trajectory_metric_topic("ttc_values"), "std_msgs/msg/Float64MultiArray"},
    {trajectory_metric_topic("lateral_deviations"), "std_msgs/msg/Float64MultiArray"},
    {trajectory_metric_topic("travel_distances"), "std_msgs/msg/Float64MultiArray"},
    {"/planning/trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {compared_trajectory_topic(), "autoware_planning_msgs/msg/Trajectory"},
    {"/perception/object_recognition/objects", "autoware_perception_msgs/msg/PredictedObjects"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
    {"/tf_static", "tf2_msgs/msg/TFMessage"}};
}

std::pair<rclcpp::Time, rclcpp::Time> OpenLoopEvaluator::run_evaluation_from_bag(
  const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
  const TopicNames & topic_names)
{
  RCLCPP_INFO(logger_, "Running open-loop evaluation for trajectory analysis");

  // Use base class method to process bag and get synchronized data
  auto bag_result = process_bag_common(bag_path, evaluation_bag_writer, topic_names);

  if (gt_source_mode_ == GTSourceMode::GT_TRAJECTORY) {
    if (!bag_result.gt_trajectory_topic_seen || bag_result.gt_trajectory_message_count == 0) {
      throw std::runtime_error(
        "gt_trajectory mode requires GT topic '" + topic_names.gt_trajectory_topic +
        "' to exist in the bag and contain at least one message.");
    }
  }

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
    auto full_json = get_full_results_as_json();

    save_json_results(
      summary_json, bag_path, "open_loop", "time_step_based_trajectory_metric.json", false, false);
    save_jsonl_results(detailed_json["results"], "time_step_based_trajectory_result.jsonl");
    save_json_results(
      full_json, bag_path, "open_loop", "time_step_based_trajectory_detailed_result.json", false,
      true);
  }

  RCLCPP_INFO(logger_, "Open-loop evaluation complete");

  return {bag_result.evaluation_start_time, bag_result.evaluation_end_time};
}

}  // namespace autoware::planning_data_analyzer

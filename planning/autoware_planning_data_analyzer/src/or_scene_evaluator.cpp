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

#include "or_scene_evaluator.hpp"

#include "bag_handler.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

// Helper for statistics calculation
template <typename Container>
struct Statistics
{
  double mean = 0.0;
  double std_dev = 0.0;
  double min_val = 0.0;
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

  // Calculate min/max
  stats.min_val = *std::min_element(values.begin(), values.end());
  stats.max_val = *std::max_element(values.begin(), values.end());

  // Calculate standard deviation
  double variance = 0.0;
  for (const auto & val : values) {
    variance += (val - stats.mean) * (val - stats.mean);
  }
  stats.std_dev = std::sqrt(variance / values.size());

  return stats;
}

ORSceneEvaluator::ORSceneEvaluator(
  rclcpp::Logger logger, std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
  double time_window_sec, const ORSuccessCriteria & success_criteria,
  bool enable_debug_visualization, const std::string & debug_output_dir)
: BaseEvaluator(logger, route_handler),
  time_window_sec_(time_window_sec),
  success_criteria_(success_criteria),
  enable_debug_visualization_(enable_debug_visualization),
  debug_output_dir_(debug_output_dir)
{
  RCLCPP_INFO(
    logger_, "ORSceneEvaluator initialized: time_window=%.2fs, debug_viz=%s", time_window_sec_,
    enable_debug_visualization_ ? "enabled" : "disabled");

  if (enable_debug_visualization_) {
    RCLCPP_INFO(logger_, "  Debug output directory: %s", debug_output_dir_.c_str());

    // Create debug directory if it doesn't exist
    std::filesystem::path debug_path(debug_output_dir_);
    if (debug_path.string().substr(0, 2) == "~/") {
      const char * home = std::getenv("HOME");
      if (home) {
        debug_path = std::string(home) + debug_path.string().substr(1);
      }
    }

    if (!std::filesystem::exists(debug_path)) {
      std::filesystem::create_directories(debug_path);
      RCLCPP_INFO(logger_, "  Created debug directory: %s", debug_path.c_str());
    }
  }

  if (success_criteria_.enabled) {
    RCLCPP_INFO(logger_, "Success criteria enabled:");
    RCLCPP_INFO(logger_, "  max_ade=%.2fm", success_criteria_.max_ade);
    RCLCPP_INFO(logger_, "  max_fde=%.2fm", success_criteria_.max_fde);
    RCLCPP_INFO(logger_, "  max_lateral_deviation=%.2fm", success_criteria_.max_lateral_deviation);
    RCLCPP_INFO(logger_, "  min_ttc=%.2fs", success_criteria_.min_ttc);
  }

  // Create OpenLoopEvaluator for reusing GT generation and comparison logic
  open_loop_evaluator_ = std::make_unique<OpenLoopEvaluator>(logger_, route_handler_);
}

void ORSceneEvaluator::set_or_events_json_path(const std::string & json_path)
{
  or_events_json_input_path_ = json_path;
  RCLCPP_INFO(logger_, "Will load OR events from: %s", json_path.c_str());
}

void ORSceneEvaluator::set_or_events_output_path(const std::string & json_path)
{
  or_events_json_output_path_ = json_path;
  RCLCPP_INFO(logger_, "Will save OR events to: %s", json_path.c_str());
}

void ORSceneEvaluator::set_input_bag_path(const std::string & bag_path)
{
  input_bag_path_ = bag_path;
  RCLCPP_INFO(logger_, "Will extract OR events from input bag: %s", bag_path.c_str());
}

void ORSceneEvaluator::set_map_path(const std::string & map_path)
{
  map_path_ = map_path;
}

void ORSceneEvaluator::set_metric_topic_prefix(const std::string & prefix)
{
  metric_topic_prefix_ = prefix;
}

std::pair<rclcpp::Time, rclcpp::Time> ORSceneEvaluator::run_evaluation_from_bag(
  const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
  const TopicNames & topic_names)
{
  RCLCPP_INFO(logger_, "Running OR scene evaluation");
  RCLCPP_INFO(logger_, "  Bag path: %s", bag_path.c_str());
  RCLCPP_INFO(
    logger_, "  Time window: [OR - %.2fs, OR + %.2fs]", time_window_sec_, time_window_sec_);

  rclcpp::Time evaluation_start_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time evaluation_end_time(0, 0, RCL_ROS_TIME);

  // Stage 1: Extract OR events
  or_events_ = extract_or_events(bag_path, topic_names);

  if (or_events_.empty()) {
    RCLCPP_WARN(logger_, "No OR events found in bag");
    return {evaluation_start_time, evaluation_end_time};
  }

  RCLCPP_INFO(logger_, "Found %zu OR events, beginning evaluation...", or_events_.size());

  // Initialize bag writer topics if provided
  if (evaluation_bag_writer) {
    create_topics_in_bag(*evaluation_bag_writer);
  }

  // Stage 2: Evaluate each OR event
  event_metrics_list_.clear();
  for (size_t i = 0; i < or_events_.size(); ++i) {
    const auto & event = or_events_[i];
    RCLCPP_INFO(
      logger_, "Evaluating OR event %zu/%zu at t=%.3fs", i + 1, or_events_.size(),
      event.timestamp.seconds());

    auto event_metrics = evaluate_or_event(event, bag_path, evaluation_bag_writer, topic_names);
    event_metrics_list_.push_back(event_metrics);

    if (event_metrics_list_.size() == 1) {
      evaluation_start_time = event.window_start;
    }
    evaluation_end_time = event.window_end;
  }

  // Calculate summary
  calculate_summary();

  // Save results
  auto detailed_json = get_detailed_results_as_json();
  save_json_results(detailed_json, bag_path, "or_scene", "or_scene_evaluation_results");

  // Log summary
  RCLCPP_INFO(logger_, "OR scene evaluation complete:");
  RCLCPP_INFO(logger_, "  Total OR events: %zu", summary_.total_or_events);
  RCLCPP_INFO(logger_, "  Events with predictions: %zu", summary_.events_with_valid_predictions);
  RCLCPP_INFO(
    logger_, "  Mean ADE: %.3fm (±%.3fm)", summary_.mean_ade_all_events,
    summary_.std_ade_all_events);
  RCLCPP_INFO(
    logger_, "  Mean FDE: %.3fm (±%.3fm)", summary_.mean_fde_all_events,
    summary_.std_fde_all_events);

  if (success_criteria_.enabled) {
    RCLCPP_INFO(
      logger_, "  Events meeting criteria: %zu/%zu (%.1f%%)", summary_.events_meeting_criteria,
      summary_.total_or_events, summary_.criteria_success_rate * 100.0);
  }

  return {evaluation_start_time, evaluation_end_time};
}

void ORSceneEvaluator::evaluate(
  const std::vector<std::shared_ptr<SynchronizedData>> & /* synchronized_data_list */,
  rosbag2_cpp::Writer * /* bag_writer */)
{
  // This method is called by BaseEvaluator but not used in OR scene evaluation
  // OR scene evaluation uses run_evaluation_from_bag directly
  RCLCPP_WARN(logger_, "ORSceneEvaluator::evaluate() called but not implemented");
  RCLCPP_WARN(logger_, "Use run_evaluation_from_bag() instead");
}

std::vector<OREvent> ORSceneEvaluator::extract_or_events(
  const std::string & bag_path, const TopicNames & topic_names)
{
  // Check if we should load from existing JSON
  if (!or_events_json_input_path_.empty()) {
    if (std::filesystem::exists(or_events_json_input_path_)) {
      RCLCPP_INFO(logger_, "Loading pre-extracted OR events from JSON...");
      OREventExtractor extractor(logger_, time_window_sec_);
      return extractor.load_or_events_from_json(or_events_json_input_path_);
    } else {
      RCLCPP_WARN(
        logger_, "Specified OR events JSON not found: %s", or_events_json_input_path_.c_str());
      RCLCPP_INFO(logger_, "Falling back to extraction from bag");
    }
  }

  // Extract OR events from bag
  RCLCPP_INFO(logger_, "Extracting OR events from bag (Stage 1)...");
  OREventExtractor extractor(logger_, time_window_sec_);

  // Use default topic name for control mode
  std::string control_mode_topic = "/vehicle/status/control_mode";

  // Use input_bag_path if specified (for LIVE trajectory evaluation), otherwise use main bag_path
  std::string bag_for_or_extraction = input_bag_path_.empty() ? bag_path : input_bag_path_;
  RCLCPP_INFO(logger_, "Extracting OR events from: %s", bag_for_or_extraction.c_str());

  auto events = extractor.extract_or_events_from_bag(
    bag_for_or_extraction, control_mode_topic, topic_names.odometry_topic);

  // Save to JSON if output path specified
  if (!or_events_json_output_path_.empty()) {
    extractor.save_or_events_to_json(events, or_events_json_output_path_);
  }

  return events;
}

OREventMetrics ORSceneEvaluator::evaluate_or_event(
  const OREvent & event, const std::string & bag_path, rosbag2_cpp::Writer * bag_writer,
  const TopicNames & topic_names)
{
  OREventMetrics event_metrics;
  event_metrics.event_id = event_metrics_list_.size();
  event_metrics.or_timestamp = event.timestamp;
  event_metrics.window_start = event.window_start;
  event_metrics.window_end = event.window_end;
  event_metrics.vehicle_speed_at_or = event.vehicle_speed_mps;

  // Load ALL bag data (needed for GT interpolation over full trajectory horizon)
  RCLCPP_INFO(logger_, "Loading entire bag for GT generation...");
  auto bag_result = process_bag_common(bag_path, nullptr, topic_names);

  if (bag_result.synchronized_data_list.empty()) {
    RCLCPP_WARN(logger_, "OR event %zu: No synchronized data in bag", event_metrics.event_id);
    return event_metrics;
  }

  RCLCPP_INFO(
    logger_, "Loaded %zu synchronized data points from bag",
    bag_result.synchronized_data_list.size());

  // Filter to get trajectory predictions in OR window only
  std::vector<std::shared_ptr<SynchronizedData>> trajectories_in_window;
  const auto window_start_ns = event.window_start.nanoseconds();
  const auto window_end_ns = event.window_end.nanoseconds();

  for (const auto & data : bag_result.synchronized_data_list) {
    const auto data_time_ns = data->timestamp.nanoseconds();
    if (data_time_ns >= window_start_ns && data_time_ns <= window_end_ns) {
      trajectories_in_window.push_back(data);
    }
  }

  if (trajectories_in_window.empty()) {
    RCLCPP_WARN(
      logger_, "OR event %zu: No trajectories found in window [%.3f, %.3f]", event_metrics.event_id,
      event.window_start.seconds(), event.window_end.seconds());
    return event_metrics;
  }

  RCLCPP_INFO(
    logger_, "OR event %zu: Found %zu trajectory predictions in window", event_metrics.event_id,
    trajectories_in_window.size());

  // Evaluate each trajectory prediction in the window
  // Use ALL bag data for GT interpolation
  std::vector<ORTrajectoryMetrics> trajectory_metrics_list;

  for (const auto & trajectory_data : trajectories_in_window) {
    if (!trajectory_data->trajectory || trajectory_data->trajectory->points.empty()) {
      continue;
    }

    // Pass ALL bag data for GT interpolation (not just window)
    auto traj_metrics =
      evaluate_trajectory_prediction(trajectory_data, bag_result.synchronized_data_list, event);
    trajectory_metrics_list.push_back(traj_metrics);
  }

  // Generate debug visualizations if enabled
  if (enable_debug_visualization_) {
    for (size_t i = 0; i < trajectories_in_window.size() && i < trajectory_metrics_list.size();
         ++i) {
      const auto & traj_data = trajectories_in_window[i];
      const auto & traj_metrics = trajectory_metrics_list[i];

      if (traj_metrics.num_points > 0 && !traj_metrics.ground_truth_trajectory.points.empty()) {
        // Extract GT poses from stored trajectory
        std::vector<geometry_msgs::msg::Pose> gt_poses;
        for (const auto & pt : traj_metrics.ground_truth_trajectory.points) {
          gt_poses.push_back(pt.pose);
        }

        generate_debug_visualization(traj_data, gt_poses, event, traj_metrics, debug_output_dir_);
      }
    }
  }

  // Calculate aggregate metrics for this event
  event_metrics = calculate_event_metrics(trajectory_metrics_list, event);

  // Save to bag if writer provided
  if (bag_writer) {
    save_event_metrics_to_bag(event_metrics, event, *bag_writer);
  }

  return event_metrics;
}

std::vector<std::shared_ptr<SynchronizedData>> ORSceneEvaluator::load_data_for_or_window(
  const OREvent & event, const std::string & bag_path, const TopicNames & topic_names)
{
  // For now, we need to process the entire bag to get all data for GT interpolation
  // GT generation requires data beyond the evaluation window (for trajectory horizon)
  // TODO(go-sakayori): Optimize to only read specific time range from bag

  RCLCPP_INFO(logger_, "Loading bag data for GT generation (processing entire bag)...");
  auto bag_result = process_bag_common(bag_path, nullptr, topic_names);

  RCLCPP_INFO(
    logger_, "Total synchronized data points in bag: %zu",
    bag_result.synchronized_data_list.size());

  // Filter to get trajectory predictions in OR event window only
  // But keep ALL data for GT interpolation (trajectories need future kinematic states)
  std::vector<std::shared_ptr<SynchronizedData>> window_data;
  const auto window_start_ns = event.window_start.nanoseconds();
  const auto window_end_ns = event.window_end.nanoseconds();

  for (const auto & data : bag_result.synchronized_data_list) {
    const auto data_time_ns = data->timestamp.nanoseconds();
    if (data_time_ns >= window_start_ns && data_time_ns <= window_end_ns) {
      window_data.push_back(data);
    }
  }

  RCLCPP_INFO(
    logger_, "Filtered %zu trajectory predictions in window [%.3f, %.3f]", window_data.size(),
    event.window_start.seconds(), event.window_end.seconds());

  return window_data;
}

ORTrajectoryMetrics ORSceneEvaluator::evaluate_trajectory_prediction(
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  const std::vector<std::shared_ptr<SynchronizedData>> & all_data_for_gt, const OREvent & event)
{
  ORTrajectoryMetrics metrics;
  metrics.prediction_time = trajectory_data->timestamp;

  // Calculate time relative to OR (in seconds, preserving sign)
  const double time_diff_sec =
    (trajectory_data->timestamp.nanoseconds() - event.timestamp.nanoseconds()) / 1e9;
  metrics.time_relative_to_or_sec = time_diff_sec;  // Negative if before OR

  const auto & original_trajectory = *(trajectory_data->trajectory);
  metrics.num_points_original = original_trajectory.points.size();

  if (!original_trajectory.points.empty()) {
    metrics.trajectory_duration_original =
      rclcpp::Duration(original_trajectory.points.back().time_from_start).seconds();
  } else {
    metrics.trajectory_duration_original = 0.0;
  }

  // Get GT data time range
  if (all_data_for_gt.empty()) {
    RCLCPP_WARN(logger_, "No GT data available");
    metrics.num_points = 0;
    metrics.trajectory_duration = 0.0;
    metrics.gt_coverage_ratio = 0.0;
    return metrics;
  }

  const rclcpp::Time gt_start = all_data_for_gt.front()->timestamp;
  const rclcpp::Time gt_end = all_data_for_gt.back()->timestamp;

  // Truncate trajectory to available GT range
  auto [truncated_traj, coverage] = truncate_trajectory_to_gt_range(
    original_trajectory, trajectory_data->timestamp, gt_start, gt_end);

  metrics.gt_coverage_ratio = coverage;

  if (truncated_traj.points.empty()) {
    RCLCPP_WARN(
      logger_, "Trajectory at t=%.3f completely outside GT range [%.3f, %.3f]",
      metrics.prediction_time.seconds(), gt_start.seconds(), gt_end.seconds());
    metrics.num_points = 0;
    metrics.trajectory_duration = 0.0;
    return metrics;
  }

  RCLCPP_DEBUG(
    logger_, "Truncated trajectory from %zu to %zu points (%.1f%% coverage)",
    metrics.num_points_original, truncated_traj.points.size(), coverage * 100.0);

  // Create temporary SynchronizedData with truncated trajectory
  auto truncated_data = std::make_shared<SynchronizedData>(*trajectory_data);
  truncated_data->trajectory =
    std::make_shared<autoware_planning_msgs::msg::Trajectory>(truncated_traj);

  // Generate ground truth for truncated trajectory
  auto ground_truth_opt =
    open_loop_evaluator_->generate_ground_truth_trajectory(truncated_data, all_data_for_gt);

  if (!ground_truth_opt.has_value()) {
    RCLCPP_WARN(
      logger_, "Failed to generate GT for truncated trajectory at t=%.3f",
      metrics.prediction_time.seconds());
    metrics.num_points = 0;
    metrics.trajectory_duration = 0.0;
    return metrics;
  }

  // Create EvaluationData structure for OpenLoopEvaluator
  OpenLoopEvaluator::EvaluationData eval_data;
  eval_data.synchronized_data = truncated_data;
  eval_data.ground_truth_trajectory = ground_truth_opt.value();

  // Store for visualization
  metrics.predicted_trajectory = truncated_traj;
  metrics.ground_truth_trajectory = ground_truth_opt.value();

  // Evaluate trajectory using OpenLoopEvaluator
  auto open_loop_metrics = open_loop_evaluator_->evaluate_trajectory(eval_data);

  // Convert OpenLoopTrajectoryMetrics to ORTrajectoryMetrics
  metrics.num_points = open_loop_metrics.num_points;
  metrics.trajectory_duration = open_loop_metrics.trajectory_duration;

  // Extract ADE/FDE
  if (!open_loop_metrics.ade.empty()) {
    metrics.ade = open_loop_metrics.ade.back();  // Final ADE (average over all points)
  } else {
    metrics.ade = 0.0;
  }

  if (!open_loop_metrics.displacement_errors.empty()) {
    metrics.fde = open_loop_metrics.displacement_errors.back();  // Final point error
  } else {
    metrics.fde = 0.0;
  }

  // Lateral/longitudinal deviation
  if (!open_loop_metrics.lateral_deviations.empty()) {
    metrics.mean_lateral_deviation = std::accumulate(
                                       open_loop_metrics.lateral_deviations.begin(),
                                       open_loop_metrics.lateral_deviations.end(), 0.0) /
                                     open_loop_metrics.lateral_deviations.size();
    metrics.max_lateral_deviation = *std::max_element(
      open_loop_metrics.lateral_deviations.begin(), open_loop_metrics.lateral_deviations.end());
  } else {
    metrics.mean_lateral_deviation = 0.0;
    metrics.max_lateral_deviation = 0.0;
  }

  if (!open_loop_metrics.longitudinal_deviations.empty()) {
    metrics.mean_longitudinal_deviation = std::accumulate(
                                            open_loop_metrics.longitudinal_deviations.begin(),
                                            open_loop_metrics.longitudinal_deviations.end(), 0.0) /
                                          open_loop_metrics.longitudinal_deviations.size();
  } else {
    metrics.mean_longitudinal_deviation = 0.0;
  }

  // TTC
  if (!open_loop_metrics.ttc.empty()) {
    metrics.min_ttc = *std::min_element(open_loop_metrics.ttc.begin(), open_loop_metrics.ttc.end());
  } else {
    metrics.min_ttc = std::numeric_limits<double>::max();
  }

  // Evaluate success criteria if enabled
  if (success_criteria_.enabled) {
    metrics.meets_success_criteria = evaluate_success_criteria(metrics, metrics.failure_reasons);
  } else {
    metrics.meets_success_criteria = false;  // Not checked
  }

  return metrics;
}

bool ORSceneEvaluator::evaluate_success_criteria(
  const ORTrajectoryMetrics & metrics, std::vector<std::string> & failure_reasons) const
{
  failure_reasons.clear();

  if (!success_criteria_.enabled) {
    return false;
  }

  bool passes = true;

  if (metrics.ade > success_criteria_.max_ade) {
    failure_reasons.push_back(
      "ADE too high: " + std::to_string(metrics.ade) + " > " +
      std::to_string(success_criteria_.max_ade));
    passes = false;
  }

  if (metrics.fde > success_criteria_.max_fde) {
    failure_reasons.push_back(
      "FDE too high: " + std::to_string(metrics.fde) + " > " +
      std::to_string(success_criteria_.max_fde));
    passes = false;
  }

  if (std::abs(metrics.mean_lateral_deviation) > success_criteria_.max_lateral_deviation) {
    failure_reasons.push_back(
      "Lateral deviation too high: " + std::to_string(std::abs(metrics.mean_lateral_deviation)) +
      " > " + std::to_string(success_criteria_.max_lateral_deviation));
    passes = false;
  }

  if (metrics.min_ttc < success_criteria_.min_ttc) {
    failure_reasons.push_back(
      "TTC too low: " + std::to_string(metrics.min_ttc) + " < " +
      std::to_string(success_criteria_.min_ttc));
    passes = false;
  }

  return passes;
}

OREventMetrics ORSceneEvaluator::calculate_event_metrics(
  const std::vector<ORTrajectoryMetrics> & trajectory_metrics, const OREvent & event) const
{
  OREventMetrics event_metrics;
  event_metrics.event_id = event_metrics_list_.size();
  event_metrics.or_timestamp = event.timestamp;
  event_metrics.window_start = event.window_start;
  event_metrics.window_end = event.window_end;
  event_metrics.vehicle_speed_at_or = event.vehicle_speed_mps;
  event_metrics.trajectory_metrics = trajectory_metrics;
  event_metrics.total_predictions = trajectory_metrics.size();

  // Count predictions before/after OR
  event_metrics.predictions_before_or = std::count_if(
    trajectory_metrics.begin(), trajectory_metrics.end(),
    [](const auto & m) { return m.time_relative_to_or_sec < 0.0; });
  event_metrics.predictions_after_or =
    event_metrics.total_predictions - event_metrics.predictions_before_or;

  if (trajectory_metrics.empty()) {
    return event_metrics;
  }

  // Collect values for statistics
  std::vector<double> ade_values, fde_values, lateral_dev_values, ttc_values;
  for (const auto & m : trajectory_metrics) {
    ade_values.push_back(m.ade);
    fde_values.push_back(m.fde);
    lateral_dev_values.push_back(std::abs(m.mean_lateral_deviation));
    if (std::isfinite(m.min_ttc)) {
      ttc_values.push_back(m.min_ttc);
    }
  }

  // ADE statistics
  auto ade_stats = calculate_statistics(ade_values);
  event_metrics.mean_ade = ade_stats.mean;
  event_metrics.std_ade = ade_stats.std_dev;
  event_metrics.min_ade = ade_stats.min_val;
  event_metrics.max_ade = ade_stats.max_val;

  // FDE statistics
  auto fde_stats = calculate_statistics(fde_values);
  event_metrics.mean_fde = fde_stats.mean;
  event_metrics.std_fde = fde_stats.std_dev;
  event_metrics.min_fde = fde_stats.min_val;
  event_metrics.max_fde = fde_stats.max_val;

  // Lateral deviation statistics
  auto lateral_stats = calculate_statistics(lateral_dev_values);
  event_metrics.mean_lateral_deviation = lateral_stats.mean;
  event_metrics.max_lateral_deviation = lateral_stats.max_val;

  // TTC statistics
  if (!ttc_values.empty()) {
    auto ttc_stats = calculate_statistics(ttc_values);
    event_metrics.mean_min_ttc = ttc_stats.mean;
    event_metrics.worst_ttc = ttc_stats.min_val;
  } else {
    event_metrics.mean_min_ttc = std::numeric_limits<double>::max();
    event_metrics.worst_ttc = std::numeric_limits<double>::max();
  }

  // Success criteria results
  if (success_criteria_.enabled) {
    event_metrics.predictions_meeting_criteria = std::count_if(
      trajectory_metrics.begin(), trajectory_metrics.end(),
      [](const auto & m) { return m.meets_success_criteria; });
    event_metrics.criteria_success_rate =
      static_cast<double>(event_metrics.predictions_meeting_criteria) /
      event_metrics.total_predictions;
  } else {
    event_metrics.predictions_meeting_criteria = 0;
    event_metrics.criteria_success_rate = 0.0;
  }

  return event_metrics;
}

void ORSceneEvaluator::calculate_summary()
{
  summary_ = ORSceneSummary{};
  summary_.total_or_events = or_events_.size();
  summary_.events_with_valid_predictions = std::count_if(
    event_metrics_list_.begin(), event_metrics_list_.end(),
    [](const auto & e) { return e.total_predictions > 0; });

  if (event_metrics_list_.empty()) {
    return;
  }

  // Collect values across all events
  std::vector<double> ade_values, fde_values, lateral_dev_values, ttc_values;
  double total_predictions = 0.0;
  double total_duration = 0.0;

  for (const auto & event_metrics : event_metrics_list_) {
    if (event_metrics.total_predictions == 0) continue;

    ade_values.push_back(event_metrics.mean_ade);
    fde_values.push_back(event_metrics.mean_fde);
    lateral_dev_values.push_back(event_metrics.mean_lateral_deviation);
    if (std::isfinite(event_metrics.mean_min_ttc)) {
      ttc_values.push_back(event_metrics.mean_min_ttc);
    }

    total_predictions += event_metrics.total_predictions;
    total_duration += (event_metrics.window_end - event_metrics.window_start).seconds();
  }

  // ADE statistics
  if (!ade_values.empty()) {
    auto ade_stats = calculate_statistics(ade_values);
    summary_.mean_ade_all_events = ade_stats.mean;
    summary_.std_ade_all_events = ade_stats.std_dev;
    summary_.min_ade_all_events = ade_stats.min_val;
    summary_.max_ade_all_events = ade_stats.max_val;
  }

  // FDE statistics
  if (!fde_values.empty()) {
    auto fde_stats = calculate_statistics(fde_values);
    summary_.mean_fde_all_events = fde_stats.mean;
    summary_.std_fde_all_events = fde_stats.std_dev;
    summary_.min_fde_all_events = fde_stats.min_val;
    summary_.max_fde_all_events = fde_stats.max_val;
  }

  // Lateral deviation statistics
  if (!lateral_dev_values.empty()) {
    auto lateral_stats = calculate_statistics(lateral_dev_values);
    summary_.mean_lateral_deviation_all_events = lateral_stats.mean;
    summary_.std_lateral_deviation_all_events = lateral_stats.std_dev;
    summary_.max_lateral_deviation_all_events = lateral_stats.max_val;
  }

  // TTC statistics
  if (!ttc_values.empty()) {
    auto ttc_stats = calculate_statistics(ttc_values);
    summary_.mean_min_ttc_all_events = ttc_stats.mean;
    summary_.worst_ttc_all_events = ttc_stats.min_val;
  } else {
    summary_.mean_min_ttc_all_events = std::numeric_limits<double>::max();
    summary_.worst_ttc_all_events = std::numeric_limits<double>::max();
  }

  // Coverage statistics
  summary_.mean_predictions_per_event =
    summary_.events_with_valid_predictions > 0
      ? total_predictions / summary_.events_with_valid_predictions
      : 0.0;
  summary_.total_evaluation_duration = total_duration;

  // Success criteria results
  if (success_criteria_.enabled) {
    summary_.events_meeting_criteria = std::count_if(
      event_metrics_list_.begin(), event_metrics_list_.end(),
      [](const auto & e) { return e.predictions_meeting_criteria > 0; });
    summary_.criteria_success_rate =
      static_cast<double>(summary_.events_meeting_criteria) / summary_.total_or_events;
  } else {
    summary_.events_meeting_criteria = 0;
    summary_.criteria_success_rate = 0.0;
  }
}

nlohmann::json ORSceneEvaluator::get_summary_as_json() const
{
  nlohmann::json j;

  j["total_or_events"] = summary_.total_or_events;
  j["events_with_valid_predictions"] = summary_.events_with_valid_predictions;

  j["ade"]["mean"] = summary_.mean_ade_all_events;
  j["ade"]["std"] = summary_.std_ade_all_events;
  j["ade"]["min"] = summary_.min_ade_all_events;
  j["ade"]["max"] = summary_.max_ade_all_events;

  j["fde"]["mean"] = summary_.mean_fde_all_events;
  j["fde"]["std"] = summary_.std_fde_all_events;
  j["fde"]["min"] = summary_.min_fde_all_events;
  j["fde"]["max"] = summary_.max_fde_all_events;

  j["lateral_deviation"]["mean"] = summary_.mean_lateral_deviation_all_events;
  j["lateral_deviation"]["std"] = summary_.std_lateral_deviation_all_events;
  j["lateral_deviation"]["max"] = summary_.max_lateral_deviation_all_events;

  j["ttc"]["mean_min_ttc"] = summary_.mean_min_ttc_all_events;
  j["ttc"]["worst_ttc"] = summary_.worst_ttc_all_events;

  j["mean_predictions_per_event"] = summary_.mean_predictions_per_event;
  j["total_evaluation_duration_sec"] = summary_.total_evaluation_duration;

  if (success_criteria_.enabled) {
    j["success_criteria"]["enabled"] = true;
    j["success_criteria"]["events_meeting_criteria"] = summary_.events_meeting_criteria;
    j["success_criteria"]["criteria_success_rate"] = summary_.criteria_success_rate;
  } else {
    j["success_criteria"]["enabled"] = false;
  }

  return j;
}

nlohmann::json ORSceneEvaluator::get_detailed_results_as_json() const
{
  nlohmann::json j;

  j["evaluation_mode"] = "or_scene";
  j["time_window_sec"] = time_window_sec_;
  j["summary"] = get_summary_as_json();

  nlohmann::json events_array = nlohmann::json::array();
  for (const auto & event_metrics : event_metrics_list_) {
    nlohmann::json event_json;

    event_json["event_id"] = event_metrics.event_id;
    event_json["or_timestamp"] = event_metrics.or_timestamp.seconds();
    event_json["window_start"] = event_metrics.window_start.seconds();
    event_json["window_end"] = event_metrics.window_end.seconds();
    event_json["vehicle_speed_at_or"] = event_metrics.vehicle_speed_at_or;

    event_json["total_predictions"] = event_metrics.total_predictions;
    event_json["predictions_before_or"] = event_metrics.predictions_before_or;
    event_json["predictions_after_or"] = event_metrics.predictions_after_or;

    event_json["mean_ade"] = event_metrics.mean_ade;
    event_json["std_ade"] = event_metrics.std_ade;
    event_json["min_ade"] = event_metrics.min_ade;
    event_json["max_ade"] = event_metrics.max_ade;

    event_json["mean_fde"] = event_metrics.mean_fde;
    event_json["std_fde"] = event_metrics.std_fde;
    event_json["min_fde"] = event_metrics.min_fde;
    event_json["max_fde"] = event_metrics.max_fde;

    event_json["mean_lateral_deviation"] = event_metrics.mean_lateral_deviation;
    event_json["max_lateral_deviation"] = event_metrics.max_lateral_deviation;

    event_json["mean_min_ttc"] = event_metrics.mean_min_ttc;
    event_json["worst_ttc"] = event_metrics.worst_ttc;

    if (success_criteria_.enabled) {
      event_json["predictions_meeting_criteria"] = event_metrics.predictions_meeting_criteria;
      event_json["criteria_success_rate"] = event_metrics.criteria_success_rate;
    }

    // Per-trajectory details
    nlohmann::json predictions_array = nlohmann::json::array();
    for (const auto & traj_metrics : event_metrics.trajectory_metrics) {
      nlohmann::json traj_json;

      traj_json["prediction_time"] = traj_metrics.prediction_time.seconds();
      traj_json["time_relative_to_or"] = traj_metrics.time_relative_to_or_sec;

      traj_json["ade"] = traj_metrics.ade;
      traj_json["fde"] = traj_metrics.fde;
      traj_json["mean_lateral_deviation"] = traj_metrics.mean_lateral_deviation;
      traj_json["max_lateral_deviation"] = traj_metrics.max_lateral_deviation;
      traj_json["mean_longitudinal_deviation"] = traj_metrics.mean_longitudinal_deviation;
      traj_json["min_ttc"] = traj_metrics.min_ttc;

      traj_json["num_points"] = traj_metrics.num_points;
      traj_json["num_points_original"] = traj_metrics.num_points_original;
      traj_json["trajectory_duration"] = traj_metrics.trajectory_duration;
      traj_json["trajectory_duration_original"] = traj_metrics.trajectory_duration_original;
      traj_json["gt_coverage_ratio"] = traj_metrics.gt_coverage_ratio;

      if (success_criteria_.enabled) {
        traj_json["meets_criteria"] = traj_metrics.meets_success_criteria;
        if (!traj_metrics.failure_reasons.empty()) {
          traj_json["failure_reasons"] = traj_metrics.failure_reasons;
        }
      }

      predictions_array.push_back(traj_json);
    }
    event_json["predictions"] = predictions_array;

    events_array.push_back(event_json);
  }
  j["or_events"] = events_array;

  return j;
}

std::vector<std::pair<std::string, std::string>> ORSceneEvaluator::get_result_topics() const
{
  std::string prefix = metric_topic_prefix_.empty() ? "" : "/" + metric_topic_prefix_;
  return {
    {prefix + "/or_scene/event_markers", "visualization_msgs/msg/MarkerArray"},
    {prefix + "/or_scene/ade", "std_msgs/msg/Float64"},
    {prefix + "/or_scene/fde", "std_msgs/msg/Float64"},
    {prefix + "/or_scene/lateral_deviation", "std_msgs/msg/Float64"},
    {prefix + "/or_scene/ttc", "std_msgs/msg/Float64"},
    {prefix + "/or_scene/prediction_success", "std_msgs/msg/Bool"},
    {prefix + "/or_scene/predicted_trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {prefix + "/or_scene/ground_truth_trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
    {"/tf_static", "tf2_msgs/msg/TFMessage"}};
}

void ORSceneEvaluator::save_event_metrics_to_bag(
  const OREventMetrics & event_metrics, const OREvent & event, rosbag2_cpp::Writer & bag_writer)
{
  std::string prefix = metric_topic_prefix_.empty() ? "" : "/" + metric_topic_prefix_;

  // Create markers for OR event
  auto markers = create_or_event_markers(event, event_metrics);
  const rclcpp::Time marker_time = event.timestamp;
  bag_writer.write(markers, prefix + "/or_scene/event_markers", marker_time);

  // Write per-trajectory metrics
  for (const auto & traj_metrics : event_metrics.trajectory_metrics) {
    const rclcpp::Time traj_time = traj_metrics.prediction_time;

    // ADE
    std_msgs::msg::Float64 ade_msg;
    ade_msg.data = traj_metrics.ade;
    bag_writer.write(ade_msg, prefix + "/or_scene/ade", traj_time);

    // FDE
    std_msgs::msg::Float64 fde_msg;
    fde_msg.data = traj_metrics.fde;
    bag_writer.write(fde_msg, prefix + "/or_scene/fde", traj_time);

    // Lateral deviation
    std_msgs::msg::Float64 lateral_msg;
    lateral_msg.data = traj_metrics.mean_lateral_deviation;
    bag_writer.write(lateral_msg, prefix + "/or_scene/lateral_deviation", traj_time);

    // TTC
    std_msgs::msg::Float64 ttc_msg;
    ttc_msg.data = traj_metrics.min_ttc;
    bag_writer.write(ttc_msg, prefix + "/or_scene/ttc", traj_time);

    // Success flag (if criteria enabled)
    if (success_criteria_.enabled) {
      std_msgs::msg::Bool success_msg;
      success_msg.data = traj_metrics.meets_success_criteria;
      bag_writer.write(success_msg, prefix + "/or_scene/prediction_success", traj_time);
    }
  }
}

visualization_msgs::msg::MarkerArray ORSceneEvaluator::create_or_event_markers(
  const OREvent & /* event */, const OREventMetrics & /* event_metrics */) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // TODO(go-sakayori): Create visualization markers for OR event
  // - Marker at OR location
  // - Text showing event_id, metrics summary
  // - Timeline markers showing evaluation window

  return marker_array;
}

std::pair<autoware_planning_msgs::msg::Trajectory, double>
ORSceneEvaluator::truncate_trajectory_to_gt_range(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const rclcpp::Time & trajectory_start_time, const rclcpp::Time & gt_data_start,
  const rclcpp::Time & gt_data_end) const
{
  autoware_planning_msgs::msg::Trajectory truncated;
  truncated.header = trajectory.header;

  const size_t original_size = trajectory.points.size();

  if (original_size == 0) {
    return {truncated, 0.0};
  }

  // Find points within GT data range
  for (const auto & pt : trajectory.points) {
    const rclcpp::Time point_time = trajectory_start_time + rclcpp::Duration(pt.time_from_start);

    // Check if this point is within GT data range (use nanoseconds to avoid clock type mismatch)
    const int64_t point_ns = point_time.nanoseconds();
    const int64_t gt_start_ns = gt_data_start.nanoseconds();
    const int64_t gt_end_ns = gt_data_end.nanoseconds();

    if (point_ns >= gt_start_ns && point_ns <= gt_end_ns) {
      truncated.points.push_back(pt);
    }
  }

  const double coverage =
    truncated.points.empty() ? 0.0 : static_cast<double>(truncated.points.size()) / original_size;

  return {truncated, coverage};
}

void ORSceneEvaluator::generate_debug_visualization(
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  const std::vector<geometry_msgs::msg::Pose> & ground_truth_poses, const OREvent & event,
  const ORTrajectoryMetrics & metrics, const std::string & output_dir)
{
  RCLCPP_INFO(logger_, "Starting debug visualization generation...");

  // Prepare data for Python visualization script
  nlohmann::json viz_data;

  // Predicted trajectory (use truncated version from metrics)
  nlohmann::json pred_array = nlohmann::json::array();
  for (const auto & pt : metrics.predicted_trajectory.points) {
    nlohmann::json pose_json;
    pose_json["x"] = pt.pose.position.x;
    pose_json["y"] = pt.pose.position.y;
    pose_json["z"] = pt.pose.position.z;
    pred_array.push_back(pose_json);
  }
  viz_data["predicted_trajectory"] = pred_array;

  // Ground truth trajectory
  nlohmann::json gt_array = nlohmann::json::array();
  for (const auto & pose : ground_truth_poses) {
    nlohmann::json pose_json;
    pose_json["x"] = pose.position.x;
    pose_json["y"] = pose.position.y;
    pose_json["z"] = pose.position.z;
    gt_array.push_back(pose_json);
  }
  viz_data["ground_truth_trajectory"] = gt_array;

  // Metrics
  viz_data["metrics"]["ade"] = metrics.ade;
  viz_data["metrics"]["fde"] = metrics.fde;
  viz_data["metrics"]["mean_lateral_deviation"] = metrics.mean_lateral_deviation;
  viz_data["metrics"]["min_ttc"] = metrics.min_ttc;
  viz_data["metrics"]["num_points"] = metrics.num_points;
  viz_data["metrics"]["trajectory_duration"] = metrics.trajectory_duration;
  viz_data["metrics"]["time_relative_to_or"] = metrics.time_relative_to_or_sec;

  // Event info
  viz_data["event_info"]["event_id"] = event_metrics_list_.size();
  viz_data["event_info"]["or_timestamp"] = event.timestamp.seconds();
  viz_data["event_info"]["vehicle_speed_at_or"] = event.vehicle_speed_mps;

  // Vehicle position at OR (from event struct, not trajectory data)
  viz_data["event_info"]["vehicle_x_at_or"] = event.vehicle_x_at_or;
  viz_data["event_info"]["vehicle_y_at_or"] = event.vehicle_y_at_or;

  viz_data["event_info"]["bag_name"] = "rosbag";

  // Add map path for lanelet visualization (optional)
  if (!map_path_.empty()) {
    viz_data["map_path"] = map_path_;
  }

  // Add objects if available
  if (trajectory_data->objects && !trajectory_data->objects->objects.empty()) {
    nlohmann::json objects_array = nlohmann::json::array();

    for (const auto & obj : trajectory_data->objects->objects) {
      nlohmann::json obj_json;

      // Position and orientation
      const auto & pose = obj.kinematics.initial_pose_with_covariance.pose;
      obj_json["x"] = pose.position.x;
      obj_json["y"] = pose.position.y;
      obj_json["orientation"]["x"] = pose.orientation.x;
      obj_json["orientation"]["y"] = pose.orientation.y;
      obj_json["orientation"]["z"] = pose.orientation.z;
      obj_json["orientation"]["w"] = pose.orientation.w;

      // Dimensions
      obj_json["length"] = obj.shape.dimensions.x;
      obj_json["width"] = obj.shape.dimensions.y;
      obj_json["height"] = obj.shape.dimensions.z;

      // Shape type and footprint
      obj_json["shape_type"] = obj.shape.type;

      // If polygon footprint is available, use it
      if (!obj.shape.footprint.points.empty()) {
        nlohmann::json footprint_array = nlohmann::json::array();
        for (const auto & pt : obj.shape.footprint.points) {
          nlohmann::json pt_json;
          pt_json["x"] = pt.x;
          pt_json["y"] = pt.y;
          footprint_array.push_back(pt_json);
        }
        obj_json["footprint"] = footprint_array;
      }

      // Classification
      if (!obj.classification.empty()) {
        obj_json["class_label"] = obj.classification[0].label;
      }

      objects_array.push_back(obj_json);
    }

    viz_data["objects"] = objects_array;
  } else {
    viz_data["objects"] = nlohmann::json::array();
  }

  // Expand ~ in output_dir
  std::string expanded_dir = output_dir;
  if (expanded_dir.substr(0, 2) == "~/") {
    const char * home = std::getenv("HOME");
    if (home) {
      expanded_dir = std::string(home) + expanded_dir.substr(1);
    }
  }

  // Create temporary JSON file
  const std::string temp_json = expanded_dir + "/temp_viz_data.json";
  std::ofstream temp_file(temp_json);
  if (!temp_file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to create temp visualization data file: %s", temp_json.c_str());
    return;
  }
  temp_file << viz_data.dump(2);
  temp_file.close();

  // Generate output filename
  const int64_t pred_time_ms = static_cast<int64_t>(metrics.prediction_time.seconds() * 1000);
  const std::string output_image = expanded_dir + "/or_event_" +
                                   std::to_string(event_metrics_list_.size()) + "_pred_" +
                                   std::to_string(pred_time_ms) + ".png";

  // Call Python visualization script
  // Use ament_index to find the installed script
  std::string script_path;
  try {
    const std::string share_dir =
      ament_index_cpp::get_package_share_directory("autoware_planning_data_analyzer");
    script_path = share_dir + "/scripts/generate_or_visualization.py";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to find package share directory: %s", e.what());
    return;
  }

  if (!std::filesystem::exists(script_path)) {
    RCLCPP_ERROR(logger_, "Visualization script not found: %s", script_path.c_str());
    return;
  }

  const std::string cmd = "python3 " + script_path + " " + temp_json + " " + output_image + " 2>&1";

  const int result = std::system(cmd.c_str());

  if (result == 0) {
    RCLCPP_INFO(logger_, "Generated debug visualization: %s", output_image.c_str());
  } else {
    RCLCPP_WARN(logger_, "Failed to generate visualization (return code: %d)", result);
  }

  // Keep temp file for debugging (don't clean up)
  // std::filesystem::remove(temp_json);
}

}  // namespace autoware::planning_data_analyzer

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

#ifndef OR_SCENE_EVALUATOR_HPP_
#define OR_SCENE_EVALUATOR_HPP_

#include "base_evaluator.hpp"
#include "open_loop_evaluator.hpp"
#include "or_event_extractor.hpp"
#include "or_scene_structs.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

/**
 * @brief Evaluates trajectory predictions in Override (OR) scenarios
 *
 * This evaluator performs two-stage evaluation of planning performance
 * around detected OR events (when safety driver takes manual control):
 *
 * Stage 1: Extract OR events from bag (using OREventExtractor)
 * Stage 2: For each OR event, evaluate trajectory predictions in time window
 *
 * Time window: [OR - t, OR + t] where t is configurable (default: 0.5s)
 *
 * For each trajectory prediction in the window:
 * - Generate ground truth from actual vehicle motion (kinematic_state)
 * - Calculate ADE, FDE, lateral deviation, TTC
 * - Optionally check against success criteria thresholds
 *
 * Outputs three levels of detail:
 * 1. Per-trajectory metrics (for each prediction in each OR window)
 * 2. Per-OR-event summary (aggregated statistics for each OR)
 * 3. Overall summary (statistics across all OR events)
 */
class ORSceneEvaluator : public BaseEvaluator
{
public:
  /**
   * @brief Constructor
   * @param logger ROS logger
   * @param route_handler Route handler for map-based calculations
   * @param time_window_sec Time window on each side of OR (default: 0.5s)
   * @param success_criteria Success criteria thresholds (optional)
   * @param enable_debug_visualization Enable debug image generation (default: false)
   * @param debug_output_dir Directory for debug images (default: "~/or_scene_debug_images")
   */
  ORSceneEvaluator(
    rclcpp::Logger logger, std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
    double time_window_sec = 0.5, const ORSuccessCriteria & success_criteria = ORSuccessCriteria(),
    bool enable_debug_visualization = false,
    const std::string & debug_output_dir = "~/or_scene_debug_images");

  // BaseEvaluator interface implementation
  void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
    rosbag2_cpp::Writer * bag_writer) override;

  std::pair<rclcpp::Time, rclcpp::Time> run_evaluation_from_bag(
    const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
    const TopicNames & topic_names) override;

  nlohmann::json get_summary_as_json() const override;
  nlohmann::json get_detailed_results_as_json() const override;
  std::vector<std::pair<std::string, std::string>> get_result_topics() const override;

  /**
   * @brief Set path to pre-extracted OR events JSON (skips Stage 1)
   * @param json_path Path to or_events.json file
   */
  void set_or_events_json_path(const std::string & json_path);

  /**
   * @brief Set output path for OR events JSON
   * @param json_path Path where or_events.json should be saved
   */
  void set_or_events_output_path(const std::string & json_path);

  /**
   * @brief Set input bag path for OR extraction (when evaluate_live_trajectories=true)
   * @param bag_path Path to input bag containing OR events
   */
  void set_input_bag_path(const std::string & bag_path);

  /**
   * @brief Set map path for lanelet visualization
   * @param map_path Path to lanelet2_map.osm file
   */
  void set_map_path(const std::string & map_path);

  /**
   * @brief Set metric topic prefix for multi-run collection
   * @param prefix Prefix for metric topics (e.g., "model_v1" -> "/model_v1/or_scene/ade")
   */
  void set_metric_topic_prefix(const std::string & prefix);

private:
  double time_window_sec_;
  ORSuccessCriteria success_criteria_;
  bool enable_debug_visualization_;
  std::string debug_output_dir_;
  std::string map_path_;             // Optional: path to lanelet2_map.osm for visualization
  std::string metric_topic_prefix_;  // Optional: prefix for metric topics (multi-run support)

  std::string or_events_json_input_path_;   // Optional: pre-extracted OR events
  std::string or_events_json_output_path_;  // Where to save extracted OR events
  std::string input_bag_path_;              // Optional: input bag for OR extraction (LIVE mode)

  std::vector<OREvent> or_events_;
  std::vector<OREventMetrics> event_metrics_list_;
  ORSceneSummary summary_;

  // Reuse OpenLoopEvaluator for GT generation and comparison
  std::unique_ptr<OpenLoopEvaluator> open_loop_evaluator_;

  /**
   * @brief Stage 1: Extract OR events from bag
   * @param bag_path Path to input bag
   * @param topic_names Topic names configuration
   * @return Vector of OR events
   */
  std::vector<OREvent> extract_or_events(
    const std::string & bag_path, const TopicNames & topic_names);

  /**
   * @brief Stage 2: Evaluate single OR event
   * @param event OR event to evaluate
   * @param bag_path Path to input bag
   * @param bag_writer Output bag writer (optional)
   * @param topic_names Topic names configuration
   * @return Metrics for this OR event
   */
  OREventMetrics evaluate_or_event(
    const OREvent & event, const std::string & bag_path, rosbag2_cpp::Writer * bag_writer,
    const TopicNames & topic_names);

  /**
   * @brief Load synchronized data for OR event time window
   * @param event OR event
   * @param bag_path Path to input bag
   * @param topic_names Topic names configuration
   * @return Synchronized data in window [event.window_start, event.window_end]
   */
  std::vector<std::shared_ptr<SynchronizedData>> load_data_for_or_window(
    const OREvent & event, const std::string & bag_path, const TopicNames & topic_names);

  /**
   * @brief Evaluate single trajectory prediction against ground truth
   * @param trajectory_data Trajectory to evaluate
   * @param all_data_for_gt All synchronized data (for GT interpolation)
   * @param event OR event context
   * @return Metrics for this trajectory
   */
  ORTrajectoryMetrics evaluate_trajectory_prediction(
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    const std::vector<std::shared_ptr<SynchronizedData>> & all_data_for_gt, const OREvent & event);

  /**
   * @brief Truncate trajectory to available GT time range
   * @param trajectory Original trajectory
   * @param trajectory_start_time Timestamp when trajectory was published
   * @param gt_data_start First GT timestamp
   * @param gt_data_end Last GT timestamp
   * @return Truncated trajectory and coverage ratio
   */
  std::pair<autoware_planning_msgs::msg::Trajectory, double> truncate_trajectory_to_gt_range(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const rclcpp::Time & trajectory_start_time, const rclcpp::Time & gt_data_start,
    const rclcpp::Time & gt_data_end) const;

  /**
   * @brief Check if trajectory meets success criteria
   * @param metrics Trajectory metrics to check
   * @param failure_reasons Output vector of failure reasons (if any)
   * @return true if all criteria met
   */
  bool evaluate_success_criteria(
    const ORTrajectoryMetrics & metrics, std::vector<std::string> & failure_reasons) const;

  /**
   * @brief Calculate aggregate statistics for OR event
   * @param trajectory_metrics Vector of per-trajectory metrics
   * @return Aggregated event metrics
   */
  OREventMetrics calculate_event_metrics(
    const std::vector<ORTrajectoryMetrics> & trajectory_metrics, const OREvent & event) const;

  /**
   * @brief Calculate overall summary statistics
   */
  void calculate_summary();

  /**
   * @brief Save event metrics to output bag
   * @param event_metrics Metrics for one OR event
   * @param event OR event context
   * @param bag_writer Output bag writer
   */
  void save_event_metrics_to_bag(
    const OREventMetrics & event_metrics, const OREvent & event, rosbag2_cpp::Writer & bag_writer);

  /**
   * @brief Generate debug visualization image for trajectory comparison
   * @param trajectory_data Predicted trajectory
   * @param ground_truth_poses Ground truth poses
   * @param event OR event context
   * @param metrics Evaluation metrics
   * @param output_dir Directory to save image
   */
  void generate_debug_visualization(
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    const std::vector<geometry_msgs::msg::Pose> & ground_truth_poses, const OREvent & event,
    const ORTrajectoryMetrics & metrics, const std::string & output_dir);

  /**
   * @brief Create visualization marker array for OR event
   * @param event OR event
   * @param event_metrics Metrics for this event
   * @return MarkerArray message
   */
  visualization_msgs::msg::MarkerArray create_or_event_markers(
    const OREvent & event, const OREventMetrics & event_metrics) const;
};

}  // namespace autoware::planning_data_analyzer

#endif  // OR_SCENE_EVALUATOR_HPP_

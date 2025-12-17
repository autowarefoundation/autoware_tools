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

#ifndef OR_SCENE_STRUCTS_HPP_
#define OR_SCENE_STRUCTS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <vector>

namespace autoware::planning_data_analyzer
{

/**
 * @brief Represents a detected Override (OR) event from bag data
 *
 * An OR event occurs when operation mode transitions from AUTONOMOUS to LOCAL,
 * indicating the safety driver took manual control of the vehicle.
 */
struct OREvent
{
  rclcpp::Time timestamp;     // Timestamp when OR signal was detected
  rclcpp::Time window_start;  // OR_timestamp - time_window_sec
  rclcpp::Time window_end;    // OR_timestamp + time_window_sec
  rclcpp::Duration time_window =
    rclcpp::Duration(0, 0);  // Evaluation time window (symmetric around OR)

  double vehicle_speed_mps;   // Vehicle speed at OR event
  double vehicle_x_at_or;     // Vehicle X position at OR event
  double vehicle_y_at_or;     // Vehicle Y position at OR event
  uint8_t prev_mode;          // Previous operation mode (AUTONOMOUS = 2)
  uint8_t new_mode;           // New operation mode (LOCAL = 3)
  std::string prev_mode_str;  // Human-readable previous mode
  std::string new_mode_str;   // Human-readable new mode
};

/**
 * @brief Success criteria thresholds for trajectory evaluation (optional)
 *
 * These thresholds are disabled by default (like open_loop evaluation).
 * When enabled, they classify trajectories as meeting/not meeting criteria.
 */
struct ORSuccessCriteria
{
  bool enabled = false;  // Enforce thresholds?

  double max_ade = 1.0;                // Maximum allowed ADE (meters)
  double max_fde = 1.5;                // Maximum allowed FDE (meters)
  double max_lateral_deviation = 0.5;  // Maximum lateral deviation (meters)
  double min_ttc = 3.0;                // Minimum required TTC (seconds)
};

/**
 * @brief Metrics for a single trajectory prediction
 *
 * Contains evaluation metrics comparing predicted trajectory to ground truth
 * generated from actual vehicle motion (kinematic_state poses).
 */
struct ORTrajectoryMetrics
{
  rclcpp::Time prediction_time;  // When trajectory was published
  double
    time_relative_to_or_sec;  // prediction_time - OR_timestamp in seconds (negative = before OR)

  // Trajectory quality metrics (reuse from OpenLoopEvaluator)
  double ade;                          // Average Displacement Error (meters)
  double fde;                          // Final Displacement Error (meters)
  double mean_lateral_deviation;       // Average lateral deviation (meters)
  double max_lateral_deviation;        // Maximum lateral deviation (meters)
  double mean_longitudinal_deviation;  // Average longitudinal deviation (meters)
  double min_ttc;                      // Minimum Time To Collision (seconds)

  size_t num_points;                    // Number of trajectory points evaluated
  size_t num_points_original;           // Original trajectory length before truncation
  double trajectory_duration;           // Duration of evaluated trajectory (seconds)
  double trajectory_duration_original;  // Original trajectory duration before truncation
  double gt_coverage_ratio;             // Fraction of trajectory with GT data (0.0-1.0)

  // Optional: Success criteria evaluation
  bool meets_success_criteria;               // Passed all thresholds?
  std::vector<std::string> failure_reasons;  // Why it failed (if criteria enabled)

  // For debug visualization (not serialized to JSON)
  autoware_planning_msgs::msg::Trajectory predicted_trajectory;  // Truncated predicted trajectory
  autoware_planning_msgs::msg::Trajectory
    ground_truth_trajectory;  // GT trajectory for visualization
};

/**
 * @brief Aggregated metrics for one OR event
 *
 * Combines metrics from all trajectory predictions in the time window
 * around a single OR event.
 */
struct OREventMetrics
{
  size_t event_id;            // OR event index (0, 1, 2, ...)
  rclcpp::Time or_timestamp;  // When OR signal was detected
  rclcpp::Time window_start;  // Evaluation window start
  rclcpp::Time window_end;    // Evaluation window end

  double vehicle_speed_at_or;  // Speed when OR happened

  std::vector<ORTrajectoryMetrics> trajectory_metrics;  // All predictions in window

  // Aggregate statistics across all predictions
  size_t total_predictions;      // How many trajectories evaluated
  size_t predictions_before_or;  // How many were before OR timestamp
  size_t predictions_after_or;   // How many were after OR timestamp

  // ADE statistics
  double mean_ade;  // Average ADE across predictions
  double std_ade;   // Standard deviation of ADE
  double min_ade;   // Best (lowest) ADE
  double max_ade;   // Worst (highest) ADE

  // FDE statistics
  double mean_fde;  // Average FDE across predictions
  double std_fde;   // Standard deviation of FDE
  double min_fde;   // Best (lowest) FDE
  double max_fde;   // Worst (highest) FDE

  // Lateral deviation statistics
  double mean_lateral_deviation;  // Average lateral deviation
  double max_lateral_deviation;   // Maximum lateral deviation

  // TTC statistics
  double mean_min_ttc;  // Average of minimum TTCs
  double worst_ttc;     // Worst (lowest) TTC in window

  // Optional: Success criteria results
  size_t predictions_meeting_criteria;  // How many passed thresholds (if enabled)
  double criteria_success_rate;         // predictions_meeting / total (if enabled)
};

/**
 * @brief Overall summary statistics across all OR events in the bag
 */
struct ORSceneSummary
{
  // Event counts
  size_t total_or_events;                // How many OR events detected
  size_t events_with_valid_predictions;  // OR events with at least 1 trajectory in window

  // ADE statistics across all OR events
  double mean_ade_all_events;  // Mean ADE across all events
  double std_ade_all_events;   // Std dev of ADE
  double max_ade_all_events;   // Worst ADE across events
  double min_ade_all_events;   // Best ADE across events

  // FDE statistics across all OR events
  double mean_fde_all_events;  // Mean FDE across all events
  double std_fde_all_events;   // Std dev of FDE
  double max_fde_all_events;   // Worst FDE across events
  double min_fde_all_events;   // Best FDE across events

  // Lateral deviation statistics
  double mean_lateral_deviation_all_events;  // Mean lateral deviation
  double std_lateral_deviation_all_events;   // Std dev of lateral deviation
  double max_lateral_deviation_all_events;   // Maximum lateral deviation

  // TTC statistics
  double mean_min_ttc_all_events;  // Average of minimum TTCs
  double worst_ttc_all_events;     // Worst (lowest) TTC seen

  // Timing and coverage statistics
  double mean_predictions_per_event;  // Average number of trajectories per OR event
  double total_evaluation_duration;   // Total duration covered by all OR events

  // Optional: Success criteria results (only if enabled)
  size_t events_meeting_criteria;  // How many events had at least one good prediction
  double criteria_success_rate;    // events_meeting / total (if enabled)
};

}  // namespace autoware::planning_data_analyzer

#endif  // OR_SCENE_STRUCTS_HPP_

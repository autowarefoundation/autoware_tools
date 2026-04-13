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

#ifndef OPEN_LOOP_EVALUATOR_HPP_
#define OPEN_LOOP_EVALUATOR_HPP_

#include "bag_handler.hpp"
#include "base_evaluator.hpp"
#include "metrics/driving_direction_compliance.hpp"
#include "metrics/ego_progress.hpp"
#include "metrics/epdms_aggregation.hpp"
#include "metrics/extended_comfort.hpp"
#include "metrics/lane_keeping.hpp"
#include "metrics/trajectory_metrics.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

struct HorizonMetrics
{
  double ade;
  double fde;
  double ahe;
  double fhe;
  double average_lateral_deviation;
  double max_lateral_deviation;
  double average_longitudinal_deviation;
  double max_longitudinal_deviation;
  double min_ttc;
};

struct OpenLoopTrajectoryMetrics
{
  // Point-wise metrics
  std::vector<double>
    lateral_deviations;  // Lateral deviation at each trajectory point (in vehicle frame)
  std::vector<double>
    longitudinal_deviations;  // Longitudinal deviation at each trajectory point (in vehicle frame)
  std::vector<double> displacement_errors;  // Euclidean distance at each trajectory point
  std::vector<double> ade;                  // Average Displacement Error at each trajectory point
  std::vector<double> ahe;                  // Average heading error at each trajectory point [rad]
  std::vector<double> heading_errors;       // Absolute heading error at each trajectory point [rad]
  std::vector<double> ttc;                  // Time To Collision at each trajectory point
  double history_comfort{0.0};              // Binary comfort subscore for the trajectory
  double extended_comfort{0.0};             // Binary extended comfort subscore
  bool extended_comfort_available{false};
  std::string extended_comfort_reason{"unavailable"};
  double time_to_collision_within_bound{0.0};
  bool time_to_collision_within_bound_available{false};
  std::string time_to_collision_within_bound_reason{"unavailable"};
  double time_to_collision_infraction_time_s{std::numeric_limits<double>::infinity()};
  double lane_keeping{0.0};  // Binary lane keeping subscore for the trajectory
  bool lane_keeping_available{false};
  std::string lane_keeping_reason{"unavailable"};
  double ego_progress{0.0};  // Proposal-relative ego progress subscore
  bool ego_progress_available{false};
  std::string ego_progress_reason{"unavailable"};
  double ego_progress_raw_m{0.0};
  double ego_progress_best_raw_m{0.0};
  double drivable_area_compliance{0.0};  // Binary drivable area compliance subscore
  bool drivable_area_compliance_available{false};
  std::string drivable_area_compliance_reason{"unavailable"};
  double no_at_fault_collision{0.0};
  bool no_at_fault_collision_available{false};
  std::string no_at_fault_collision_reason{"unavailable"};
  double time_to_at_fault_collision_s{std::numeric_limits<double>::infinity()};
  double driving_direction_compliance{0.0};
  bool driving_direction_compliance_available{false};
  std::string driving_direction_compliance_reason{"unavailable"};
  double max_oncoming_progress_m{0.0};
  double traffic_light_compliance{0.0};
  bool traffic_light_compliance_available{false};
  std::string traffic_light_compliance_reason{"unavailable"};

  // Per-horizon metrics in insertion order: "full" first, then "1s", "2s", ...
  std::vector<std::pair<std::string, HorizonMetrics>> horizon_results;

  // Ground truth trajectory for this evaluation
  std::vector<geometry_msgs::msg::Pose> ground_truth_poses;  // Interpolated ground truth poses

  // Trajectory info
  size_t num_points;           // Number of trajectory points
  double trajectory_duration;  // Total trajectory duration in seconds

  rclcpp::Time trajectory_timestamp;   // When the trajectory was published (ROS time)
  rclcpp::Time bag_timestamp;          // Corresponding bag/system timestamp
  rclcpp::Time evaluation_start_time;  // Start time of evaluation window
  rclcpp::Time evaluation_end_time;    // End time of evaluation window
};

struct OpenLoopEvaluationSummary
{
  // Overall statistics
  double mean_ade;
  double std_ade;
  double max_ade;
  double mean_fde;
  double std_fde;
  double max_fde;

  double mean_lateral_deviation;
  double std_lateral_deviation;
  double max_lateral_deviation;

  // Evaluation coverage
  size_t total_trajectories;
  size_t valid_trajectories;        // Trajectories with at least one valid comparison
  size_t fully_valid_trajectories;  // Trajectories with all points having ground truth
  double mean_coverage_ratio;       // Average ratio of valid comparisons per trajectory

  double total_evaluation_duration;
};

class OpenLoopEvaluator : public BaseEvaluator
{
public:
  enum class GTSourceMode { KINEMATIC_STATE, GT_TRAJECTORY };
  explicit OpenLoopEvaluator(
    rclcpp::Logger logger,
    std::shared_ptr<autoware::route_handler::RouteHandler> route_handler = nullptr,
    GTSourceMode gt_source_mode = GTSourceMode::KINEMATIC_STATE,
    double gt_sync_tolerance_ms = 200.0,
    metrics::HistoryComfortParameters history_comfort_params = {},
    metrics::LaneKeepingParameters lane_keeping_params = {},
    metrics::DrivingDirectionComplianceParameters driving_direction_params = {},
    autoware::vehicle_info_utils::VehicleInfo vehicle_info = {})
  : BaseEvaluator(logger, route_handler),
    history_comfort_params_(std::move(history_comfort_params)),
    lane_keeping_params_(std::move(lane_keeping_params)),
    driving_direction_params_(std::move(driving_direction_params)),
    vehicle_info_(std::move(vehicle_info)),
    gt_source_mode_(gt_source_mode),
    gt_sync_tolerance_ms_(gt_sync_tolerance_ms)
  {
  }

  /**
   * @brief Evaluate synchronized trajectories against ground-truth data.
   *
   * Computes per-trajectory open-loop metrics and optionally writes the resulting
   * metric topics and result messages to an evaluation bag.
   * @param synchronized_data_list List of synchronized bag data containing trajectories and
   * localization
   * @param bag_writer Optional writer to save evaluation results to a new bag
   */
  void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
    rosbag2_cpp::Writer * bag_writer = nullptr) override;

  OpenLoopEvaluationSummary get_summary() const { return summary_; }

  std::vector<OpenLoopTrajectoryMetrics> get_metrics() const { return metrics_list_; }

  void set_metric_variant(const std::string & metric_variant) { metric_variant_ = metric_variant; }

  void set_evaluation_horizons(const std::vector<double> & horizons)
  {
    evaluation_horizons_ = horizons;
  }

  void set_epdms_horizons(const std::vector<double> & horizons) { epdms_horizons_ = horizons; }

  void set_extended_comfort_parameters(const metrics::ExtendedComfortParameters & parameters)
  {
    extended_comfort_parameters_ = parameters;
  }

  nlohmann::json get_summary_as_json() const override;

  nlohmann::json get_detailed_results_as_json() const override;

  /**
   * @brief Full results JSON: summary + per-trajectory per-point data + evaluation_info when saved.
   * For machine consumption / debugging; not human-readable at scale.
   */
  nlohmann::json get_full_results_as_json() const;

  /**
   * @brief Return the result topics produced by the open-loop evaluator.
   *
   * Includes both raw metric topics and the DLR-format ADE/FDE result topic
   * written to the evaluation bag.
   * @return Vector of topic name and type pairs
   */
  std::vector<std::pair<std::string, std::string>> get_result_topics() const override;

  /**
   * @brief Run open-loop evaluation directly from a rosbag.
   *
   * Reads the configured input topics from the bag, performs evaluation, and
   * optionally writes the generated outputs to an evaluation bag.
   * @param bag_path Path to the bag file
   * @param evaluation_bag_writer Optional bag writer for results
   * @param topic_names Topic names configuration
   * @return Pair of start and end times
   */
  std::pair<rclcpp::Time, rclcpp::Time> run_evaluation_from_bag(
    const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
    const TopicNames & topic_names) override;

  /**
   * @brief Input bundle for evaluating one synchronized trajectory sample.
   *
   * Stores the synchronized input data together with the precomputed ground-truth
   * trajectory used for metric calculation.
   */
  struct EvaluationData
  {
    std::shared_ptr<SynchronizedData> synchronized_data;
    autoware_planning_msgs::msg::Trajectory ground_truth_trajectory;
  };

  /**
   * @brief Generate the ground-truth trajectory for one synchronized trajectory sample.
   *
   * Uses the configured ground-truth source to create the reference trajectory
   * aligned with the evaluated prediction trajectory.
   * @param trajectory_data Data containing the trajectory
   * @param all_data All synchronized data for interpolation
   * @return Ground truth trajectory or nullopt if generation failed
   */
  std::optional<autoware_planning_msgs::msg::Trajectory> generate_ground_truth_trajectory(
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    const std::vector<std::shared_ptr<SynchronizedData>> & all_data);

  std::optional<autoware_planning_msgs::msg::Trajectory>
  generate_ground_truth_trajectory_from_topic(
    const std::shared_ptr<SynchronizedData> & trajectory_data) const;

  bool can_directly_pair_gt_trajectory(
    const autoware_planning_msgs::msg::Trajectory & predicted,
    const autoware_planning_msgs::msg::Trajectory & gt_trajectory) const;

  /**
   * @brief Evaluate one trajectory against its ground-truth trajectory.
   *
   * Computes point-wise and aggregate metrics for a single synchronized sample.
   * @param eval_data Evaluation data containing trajectory and ground truth
   * @return Metrics for this trajectory
   */
  OpenLoopTrajectoryMetrics evaluate_trajectory(const EvaluationData & eval_data);

private:
  /**
   * @brief Build the list of valid evaluation inputs for open-loop analysis.
   *
   * Generates ground-truth trajectories and excludes samples that cannot be
   * evaluated with the configured ground-truth source.
   * @param synchronized_data_list List of synchronized data
   * @return Vector of evaluation data with ground truth (invalid data excluded)
   */
  std::vector<EvaluationData> prepare_evaluation_data(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list);

  /**
   * @brief Compute planar Euclidean distance between two positions.
   */
  double calculate_distance_2d(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

  /**
   * @brief Compute longitudinal and lateral position errors in the vehicle frame.
   * @param trajectory_pose Trajectory pose (position and orientation)
   * @param ground_truth_pose Ground truth pose (position and orientation)
   * @return Pair of (longitudinal_error, lateral_error) in vehicle frame
   */
  std::pair<double, double> calculate_errors_in_vehicle_frame(
    const geometry_msgs::msg::Pose & trajectory_pose,
    const geometry_msgs::msg::Pose & ground_truth_pose);

  /**
   * @brief Interpolate ground-truth pose at a requested timestamp.
   *
   * Searches neighboring synchronized ground-truth samples and interpolates a
   * pose when the requested time lies within the available range.
   * @param target_time Time to interpolate ground truth
   * @param ground_truth_data List of ground truth data points
   * @return Interpolated pose or nullptr if interpolation not possible
   */
  std::optional<geometry_msgs::msg::Pose> interpolate_ground_truth(
    const rclcpp::Time & target_time,
    const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data);

  std::optional<geometry_msgs::msg::Pose> interpolate_ground_truth_from_trajectory(
    const rclcpp::Time & target_time,
    const autoware_planning_msgs::msg::Trajectory & gt_trajectory) const;

  /**
   * @brief Write per-trajectory evaluation outputs to the result bag.
   *
   * Saves raw metric arrays, writes the DLR-format ADE/FDE result message, and
   * stores the compared ground-truth trajectory for the evaluated sample.
   */
  void save_metrics_to_bag(
    const OpenLoopTrajectoryMetrics & metrics,
    const metrics::SyntheticEpdmsMetrics & synthetic_epdms,
    const std::map<std::string, metrics::SyntheticEpdmsMetrics> & synthetic_epdms_by_horizon,
    const EvaluationData & eval_data, rosbag2_cpp::Writer & bag_writer);

  /**
   * @brief Write ADE/FDE results in the driving_log_replayer result format.
   *
   * Serializes the metrics into a JSON payload with top-level Result, Stamp,
   * and Frame fields and writes it to
   * /driving_log_replayer/time_step_based_trajectory/results as std_msgs/msg/String.
   */
  void save_dlr_style_result_to_bag(
    const OpenLoopTrajectoryMetrics & metrics, const EvaluationData & eval_data,
    rosbag2_cpp::Writer & bag_writer);

  void save_trajectory_point_metrics_to_bag_with_variant(
    const metrics::TrajectoryPointMetrics & metrics, rosbag2_cpp::Writer & bag_writer,
    const rclcpp::Time & normalized_timestamp) const;

  std::string metric_topic(const std::string & metric_name) const;
  std::string trajectory_metric_topic(const std::string & metric_name) const;
  std::string compared_trajectory_topic() const;
  std::string dlr_result_topic() const;

  /**
   * @brief Aggregate summary statistics across all evaluated trajectories.
   *
   * Computes dataset-level ADE, FDE, lateral-deviation, coverage, and duration
   * statistics from the collected per-trajectory metrics.
   */
  void calculate_summary();

  std::vector<OpenLoopTrajectoryMetrics> metrics_list_;
  std::vector<metrics::TrajectoryPointMetrics> trajectory_point_metrics_list_;
  std::vector<metrics::HumanFilterMetrics> human_filter_metrics_list_;
  std::vector<metrics::SyntheticEpdmsMetrics> synthetic_epdms_metrics_list_;
  std::vector<std::map<std::string, metrics::SyntheticEpdmsMetrics>>
    synthetic_epdms_horizon_metrics_list_;
  metrics::HistoryComfortParameters history_comfort_params_;
  metrics::ExtendedComfortParameters extended_comfort_parameters_{};
  metrics::LaneKeepingParameters lane_keeping_params_;
  metrics::DrivingDirectionComplianceParameters driving_direction_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  OpenLoopEvaluationSummary summary_;
  std::string metric_variant_;
  GTSourceMode gt_source_mode_;
  double gt_sync_tolerance_ms_;
  std::vector<double> evaluation_horizons_;
  std::vector<double> epdms_horizons_;
};

}  // namespace autoware::planning_data_analyzer

#endif  // OPEN_LOOP_EVALUATOR_HPP_

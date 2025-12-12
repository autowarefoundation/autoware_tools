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

#include <autoware/route_handler/route_handler.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer
{

struct OpenLoopTrajectoryMetrics
{
  // Point-wise metrics
  std::vector<double> lateral_deviations;      // Lateral deviation at each trajectory point (in vehicle frame)
  std::vector<double> longitudinal_deviations; // Longitudinal deviation at each trajectory point (in vehicle frame)
  std::vector<double> displacement_errors;     // Euclidean distance at each trajectory point
  std::vector<double> ade;                    // Average Displacement Error at each trajectory point
  std::vector<double> ttc;                    // Time To Collision at each trajectory point
  
  // Ground truth trajectory for this evaluation
  std::vector<geometry_msgs::msg::Pose> ground_truth_poses;  // Interpolated ground truth poses
  
  // Trajectory info
  size_t num_points;                  // Number of trajectory points
  double trajectory_duration;         // Total trajectory duration in seconds
  
  rclcpp::Time trajectory_timestamp;  // When the trajectory was published
  rclcpp::Time evaluation_start_time; // Start time of evaluation window
  rclcpp::Time evaluation_end_time;   // End time of evaluation window
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
  size_t valid_trajectories;      // Trajectories with at least one valid comparison
  size_t fully_valid_trajectories; // Trajectories with all points having ground truth
  double mean_coverage_ratio;      // Average ratio of valid comparisons per trajectory
  
  double total_evaluation_duration;
};

class OpenLoopEvaluator : public BaseEvaluator
{
public:
  explicit OpenLoopEvaluator(
    rclcpp::Logger logger,
    std::shared_ptr<autoware::route_handler::RouteHandler> route_handler = nullptr)
  : BaseEvaluator(logger, route_handler) {}

  /**
   * @brief Evaluate trajectories against ground truth localization data
   * @param synchronized_data_list List of synchronized bag data containing trajectories and localization
   * @param bag_writer Optional writer to save evaluation results to a new bag
   */
  void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
    rosbag2_cpp::Writer * bag_writer = nullptr) override;

  OpenLoopEvaluationSummary get_summary() const { return summary_; }

  std::vector<OpenLoopTrajectoryMetrics> get_metrics() const { return metrics_list_; }

  nlohmann::json get_summary_as_json() const override;
  
  nlohmann::json get_detailed_results_as_json() const override;
  
  /**
   * @brief Get topic definitions for open-loop evaluation results
   * @return Vector of topic name and type pairs
   */
  std::vector<std::pair<std::string, std::string>> get_result_topics() const override;
  
  /**
   * @brief Run open-loop evaluation from bag file
   * @param bag_path Path to the bag file
   * @param evaluation_bag_writer Optional bag writer for results
   * @param topic_names Topic names configuration
   * @return Pair of start and end times
   */
  std::pair<rclcpp::Time, rclcpp::Time> run_evaluation_from_bag(
    const std::string & bag_path,
    rosbag2_cpp::Writer * evaluation_bag_writer,
    const TopicNames & topic_names) override;

  /**
   * @brief Structure to hold synchronized data with its precomputed ground truth trajectory
   * (Public for use by ORSceneEvaluator)
   */
  struct EvaluationData
  {
    std::shared_ptr<SynchronizedData> synchronized_data;
    autoware_planning_msgs::msg::Trajectory ground_truth_trajectory;
  };

  /**
   * @brief Generate ground truth trajectory for a single synchronized data point
   * (Public for use by ORSceneEvaluator)
   * @param trajectory_data Data containing the trajectory
   * @param all_data All synchronized data for interpolation
   * @return Ground truth trajectory or nullopt if generation failed
   */
  std::optional<autoware_planning_msgs::msg::Trajectory> generate_ground_truth_trajectory(
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    const std::vector<std::shared_ptr<SynchronizedData>> & all_data);

  /**
   * @brief Evaluate a single trajectory against ground truth
   * (Public for use by ORSceneEvaluator)
   * @param eval_data Evaluation data containing trajectory and ground truth
   * @return Metrics for this trajectory
   */
  OpenLoopTrajectoryMetrics evaluate_trajectory(
    const EvaluationData & eval_data);

private:
  /**
   * @brief Prepare evaluation data with ground truth trajectories
   * @param synchronized_data_list List of synchronized data
   * @return Vector of evaluation data with ground truth (invalid data excluded)
   */
  std::vector<EvaluationData> prepare_evaluation_data(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list);


  /**
   * @brief Calculate 2D Euclidean distance between two points
   */
  double calculate_distance_2d(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  /**
   * @brief Calculate errors in vehicle coordinate frame
   * @param trajectory_pose Trajectory pose (position and orientation)
   * @param ground_truth_pose Ground truth pose (position and orientation)
   * @return Pair of (longitudinal_error, lateral_error) in vehicle frame
   */
  std::pair<double, double> calculate_errors_in_vehicle_frame(
    const geometry_msgs::msg::Pose & trajectory_pose,
    const geometry_msgs::msg::Pose & ground_truth_pose);

  /**
   * @brief Find ground truth data at a specific time using interpolation
   * @param target_time Time to interpolate ground truth
   * @param ground_truth_data List of ground truth data points
   * @return Interpolated pose or nullptr if interpolation not possible
   */
  std::optional<geometry_msgs::msg::Pose> interpolate_ground_truth(
    const rclcpp::Time & target_time,
    const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data);

  /**
   * @brief Save evaluation metrics to bag
   */
  void save_metrics_to_bag(
    const OpenLoopTrajectoryMetrics & metrics,
    const EvaluationData & eval_data,
    rosbag2_cpp::Writer & bag_writer);


  /**
   * @brief Calculate summary statistics from all evaluations
   */
  void calculate_summary();

  std::vector<OpenLoopTrajectoryMetrics> metrics_list_;
  std::vector<metrics::TrajectoryPointMetrics> trajectory_point_metrics_list_;
  OpenLoopEvaluationSummary summary_;
};

}  // namespace autoware::planning_data_analyzer

#endif  // OPEN_LOOP_EVALUATOR_HPP_
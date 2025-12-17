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

#ifndef BASE_EVALUATOR_HPP_
#define BASE_EVALUATOR_HPP_

#include "bag_handler.hpp"
#include "metrics/trajectory_metrics.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer
{

// Forward declarations
struct SynchronizedData;
struct TopicNames;

/**
 * @brief Base class for trajectory evaluators
 *
 * This abstract base class provides common functionality for both
 * open-loop and closed-loop evaluation modes.
 */
class BaseEvaluator
{
public:
  explicit BaseEvaluator(
    rclcpp::Logger logger,
    std::shared_ptr<autoware::route_handler::RouteHandler> route_handler = nullptr)
  : logger_(logger), route_handler_(std::move(route_handler))
  {
  }

  virtual ~BaseEvaluator() = default;

  /**
   * @brief Evaluate trajectories with synchronized data
   * @param synchronized_data_list List of synchronized bag data
   * @param bag_writer Optional writer to save evaluation results
   */
  virtual void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
    rosbag2_cpp::Writer * bag_writer = nullptr) = 0;

  /**
   * @brief Get evaluation results as JSON
   * @return JSON object containing evaluation summary
   */
  virtual nlohmann::json get_summary_as_json() const = 0;

  /**
   * @brief Get detailed evaluation results as JSON
   * @return JSON object containing detailed results
   */
  virtual nlohmann::json get_detailed_results_as_json() const = 0;

  /**
   * @brief Get topic definitions for evaluation results
   * @return Vector of topic name and type pairs
   */
  virtual std::vector<std::pair<std::string, std::string>> get_result_topics() const = 0;

  /**
   * @brief Run evaluation from bag file
   * @param bag_path Path to the bag file
   * @param evaluation_bag_writer Optional bag writer for results
   * @param topic_names Topic names configuration
   * @return Pair of start and end times
   */
  virtual std::pair<rclcpp::Time, rclcpp::Time> run_evaluation_from_bag(
    const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
    const TopicNames & topic_names) = 0;

protected:
  /**
   * @brief Create topics in bag writer
   * @param bag_writer Bag writer to create topics in
   */
  void create_topics_in_bag(rosbag2_cpp::Writer & bag_writer) const
  {
    const auto topics = get_result_topics();
    for (const auto & [topic_name, topic_type] : topics) {
      const auto topic_info =
        rosbag2_storage::TopicMetadata{topic_name, topic_type, rmw_get_serialization_format(), ""};
      bag_writer.create_topic(topic_info);
    }
  }

  /**
   * @brief Write map and route markers to bag
   * @param bag_writer Bag writer
   * @param timestamp Timestamp for the markers
   */
  void write_static_markers_to_bag(
    rosbag2_cpp::Writer & /*bag_writer*/, const rclcpp::Time & /*timestamp*/) const
  {
    // This would be implemented to write map/route visualization
    // Currently a placeholder for common marker writing functionality
  }

  /**
   * @brief Common implementation for reading and processing bag data
   * @param bag_path Path to the bag file
   * @param evaluation_bag_writer Optional bag writer for results
   * @param topic_names Topic names configuration
   * @return Synchronized data list and evaluation time range
   */
  struct BagProcessingResult
  {
    std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
    rclcpp::Time evaluation_start_time;
    rclcpp::Time evaluation_end_time;
    tf2_msgs::msg::TFMessage tf_static_msgs;
  };

  BagProcessingResult process_bag_common(
    const std::string & bag_path, rosbag2_cpp::Writer * evaluation_bag_writer,
    const TopicNames & topic_names);

  /**
   * @brief Save evaluation results to JSON file
   * @param json_output JSON object containing results
   * @param bag_path Original bag file path
   * @param evaluation_mode Mode string (e.g., "closed_loop" or "open_loop")
   * @param output_filename Base filename for output (without timestamp)
   */
  void save_json_results(
    const nlohmann::json & json_output, const std::string & bag_path,
    const std::string & evaluation_mode,
    const std::string & output_filename = "evaluation_result") const;

  /**
   * @brief Write tf_static messages to evaluation bag
   * @param evaluation_bag_writer Bag writer
   * @param tf_static_msgs Static transform messages
   * @param normalized_timestamp Timestamp to use for writing
   */
  void write_tf_static_to_bag(
    rosbag2_cpp::Writer * evaluation_bag_writer, const tf2_msgs::msg::TFMessage & tf_static_msgs,
    const rclcpp::Time & normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME)) const;

  /**
   * @brief Write trajectory to evaluation bag with normalized timestamp
   * @param sync_data Synchronized data containing trajectory
   * @param bag_writer Bag writer
   * @param normalized_timestamp Normalized timestamp to use
   */
  void write_trajectory_to_bag(
    const std::shared_ptr<SynchronizedData> & sync_data, rosbag2_cpp::Writer & bag_writer,
    const rclcpp::Time & normalized_timestamp) const;

  /**
   * @brief Save trajectory point metrics to bag
   * @param metrics Trajectory point metrics to save
   * @param bag_writer Bag writer
   * @param normalized_timestamp Normalized timestamp to use
   */
  void save_trajectory_point_metrics_to_bag(
    const metrics::TrajectoryPointMetrics & metrics, rosbag2_cpp::Writer & bag_writer,
    const rclcpp::Time & normalized_timestamp) const;

  rclcpp::Logger logger_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  // For normalized timestamp calculation
  rclcpp::Time first_bag_timestamp_;
  bool first_bag_timestamp_set_ = false;
};

}  // namespace autoware::planning_data_analyzer

#endif  // BASE_EVALUATOR_HPP_

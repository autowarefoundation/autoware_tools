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

#include "base_evaluator.hpp"

#include "metrics/trajectory_metrics.hpp"
#include "serialized_bag_message.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <limits>
#include <memory>
#include <string>

namespace autoware::planning_data_analyzer
{

BaseEvaluator::BagProcessingResult BaseEvaluator::process_bag_common(
  const std::string & bag_path, rosbag2_cpp::Writer * /*evaluation_bag_writer*/,
  const TopicNames & topic_names)
{
  // Open bag reader
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(bag_path);

  // Create bag data handler with common parameters
  const double buffer_duration_sec = 20.0;    // TODO(go-sakayori): make configurable
  const size_t max_buffer_messages = 100000;  // TODO(go-sakayori): make configurable
  auto bag_data =
    std::make_shared<BagData>(0, topic_names, buffer_duration_sec, max_buffer_messages);

  // Result to return
  BagProcessingResult result;

  // Find the time range of the bag
  rclcpp::Time bag_start_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
  rclcpp::Time bag_end_time = rclcpp::Time(0);

  const bool use_bag_timestamp = true;  // TODO(go-sakayori): make configurable

  // Process all messages in the bag
  while (bag_reader.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader.read_next();
    const auto & topic_name = serialized_message->topic_name;
    rclcpp::Time msg_time(get_timestamp_ns(*serialized_message));

    // Update time range
    if (msg_time < bag_start_time) bag_start_time = msg_time;
    if (msg_time > bag_end_time) bag_end_time = msg_time;

    // Process messages using template helper
    if (topic_name == topic_names.odometry_topic) {
      process_and_append_message<Odometry>(
        serialized_message, bag_data, topic_names.odometry_topic, use_bag_timestamp, logger_);
    } else if (topic_name == topic_names.trajectory_topic) {
      process_and_append_message<Trajectory>(
        serialized_message, bag_data, topic_names.trajectory_topic, use_bag_timestamp, logger_);
    } else if (
      !topic_names.candidate_trajectories_topic.empty() &&
      topic_name == topic_names.candidate_trajectories_topic) {
      process_and_append_message<CandidateTrajectories>(
        serialized_message, bag_data, topic_names.candidate_trajectories_topic, false, logger_);
    } else if (
      !topic_names.gt_trajectory_topic.empty() && topic_name == topic_names.gt_trajectory_topic) {
      result.gt_trajectory_topic_seen = true;
      result.gt_trajectory_message_count++;
      process_and_append_message<Trajectory>(
        serialized_message, bag_data, topic_names.gt_trajectory_topic, use_bag_timestamp, logger_);
    } else if (
      topic_name == topic_names.acceleration_topic && !topic_names.acceleration_topic.empty()) {
      process_and_append_message<AccelWithCovarianceStamped>(
        serialized_message, bag_data, topic_names.acceleration_topic, use_bag_timestamp, logger_);
    } else if (topic_name == topic_names.steering_topic && !topic_names.steering_topic.empty()) {
      // SteeringReport doesn't have header, so we don't override timestamp
      process_and_append_message<SteeringReport>(
        serialized_message, bag_data, topic_names.steering_topic, false, logger_);
    } else if (topic_name == topic_names.objects_topic) {
      process_and_append_message<PredictedObjects>(
        serialized_message, bag_data, topic_names.objects_topic, use_bag_timestamp, logger_);
    } else if (
      topic_name == topic_names.traffic_signals_topic && !topic_names.traffic_signals_topic.empty())
    {
      process_and_append_message<TrafficLightGroupArray>(
        serialized_message, bag_data, topic_names.traffic_signals_topic, use_bag_timestamp,
        logger_);
    } else if (topic_name == topic_names.tf_topic) {
      process_and_append_message<TFMessage>(
        serialized_message, bag_data, topic_names.tf_topic, false, logger_);
    }
  }

  // Get kinematic states at regular intervals
  auto kinematic_states =
    bag_data->get_kinematic_states_at_interval(topic_names.evaluation_interval_ms);

  if (kinematic_states.empty()) {
    RCLCPP_ERROR(logger_, "No kinematic states found in the rosbag");
    result.evaluation_start_time = rclcpp::Clock{RCL_ROS_TIME}.now();
    result.evaluation_end_time = rclcpp::Clock{RCL_ROS_TIME}.now();
    return result;
  }

  // Collect synchronized data
  for (const auto & kinematic_state : kinematic_states) {
    const auto timestamp = rclcpp::Time(kinematic_state->header.stamp).nanoseconds();
    auto sync_data =
      bag_data->get_synchronized_data_at_time(timestamp, topic_names.sync_tolerance_ms);

    if (sync_data && sync_data->trajectory) {
      result.synchronized_data_list.push_back(sync_data);
    }
  }

  // Sort by timestamp
  std::sort(
    result.synchronized_data_list.begin(), result.synchronized_data_list.end(),
    [](const auto & a, const auto & b) { return a->timestamp < b->timestamp; });

  // Set evaluation time range
  if (!kinematic_states.empty()) {
    result.evaluation_start_time = rclcpp::Time(kinematic_states.front()->header.stamp);
    result.evaluation_end_time = rclcpp::Time(kinematic_states.back()->header.stamp);
  } else {
    result.evaluation_start_time = bag_start_time;
    result.evaluation_end_time = bag_end_time;
  }

  return result;
}

void BaseEvaluator::save_json_results(
  const nlohmann::json & json_output, const std::string & bag_path,
  const std::string & evaluation_mode, const std::string & output_filename, bool add_timestamp,
  bool include_evaluation_info) const
{
  std::filesystem::path output_path;
  if (!json_output_dir_.empty()) {
    output_path = std::filesystem::path(json_output_dir_) / output_filename;
  } else {
    const std::string json_output_path = "~/" + output_filename;
    std::string expanded_path = json_output_path;

    // Expand home directory if needed
    if (!expanded_path.empty() && expanded_path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        expanded_path = std::string(home) + expanded_path.substr(1);
      }
    }
    output_path = expanded_path;
  }

  std::filesystem::path json_path(output_path);
  std::string timestamp_string;
  if (add_timestamp) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    timestamp_string = timestamp_ss.str();

    const std::string extension =
      json_path.has_extension() ? json_path.extension().string() : ".json";
    std::string filename = json_path.stem().string() + "_" + timestamp_string + extension;
    json_path = json_path.parent_path() / filename;
  }

  nlohmann::json json_to_write = json_output;
  if (include_evaluation_info) {
    json_to_write["evaluation_info"]["timestamp"] = timestamp_string;
    json_to_write["evaluation_info"]["bag_path"] = bag_path;
    json_to_write["evaluation_info"]["evaluation_mode"] = evaluation_mode;
  }

  std::ofstream json_file(json_path);
  if (json_file.is_open()) {
    json_file << json_to_write.dump(2);  // Pretty print with 2 spaces
    json_file.close();
    RCLCPP_INFO(logger_, "JSON results saved to: %s", json_path.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Failed to save JSON results to: %s", json_path.c_str());
  }
}

void BaseEvaluator::save_jsonl_results(
  const nlohmann::json & results_array, const std::string & output_filename) const
{
  std::filesystem::path output_path;
  if (!json_output_dir_.empty()) {
    output_path = std::filesystem::path(json_output_dir_) / output_filename;
  } else {
    std::string expanded_path = "~/" + output_filename;
    if (!expanded_path.empty() && expanded_path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        expanded_path = std::string(home) + expanded_path.substr(1);
      }
    }
    output_path = expanded_path;
  }

  std::ofstream out(output_path);
  if (!out.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to save JSONL results to: %s", output_path.c_str());
    return;
  }
  for (const auto & obj : results_array) {
    out << obj.dump() << '\n';
  }
  out.close();
  RCLCPP_INFO(logger_, "JSONL results saved to: %s", output_path.c_str());
}

void BaseEvaluator::write_tf_static_to_bag(
  rosbag2_cpp::Writer * evaluation_bag_writer, const tf2_msgs::msg::TFMessage & tf_static_msgs,
  const rclcpp::Time & normalized_timestamp) const
{
  if (!evaluation_bag_writer || tf_static_msgs.transforms.empty()) {
    return;
  }

  // Create normalized tf_static message
  tf2_msgs::msg::TFMessage normalized_tf_static = tf_static_msgs;
  for (auto & transform : normalized_tf_static.transforms) {
    transform.header.stamp = normalized_timestamp;
  }

  evaluation_bag_writer->write(normalized_tf_static, "/tf_static", normalized_timestamp);
}

void BaseEvaluator::write_trajectory_to_bag(
  const std::shared_ptr<SynchronizedData> & sync_data, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & normalized_timestamp) const
{
  if (sync_data && sync_data->trajectory) {
    Trajectory corrected_trajectory = *sync_data->trajectory;
    corrected_trajectory.header.stamp = normalized_timestamp;
    bag_writer.write(corrected_trajectory, "/planning/trajectory", normalized_timestamp);
  }
}

void BaseEvaluator::save_trajectory_point_metrics_to_bag(
  const metrics::TrajectoryPointMetrics & metrics, rosbag2_cpp::Writer & bag_writer,
  const rclcpp::Time & normalized_timestamp) const
{
  // Save lateral accelerations
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.lateral_accelerations;
    bag_writer.write(msg, "/trajectory/lateral_accelerations", normalized_timestamp);
  }

  // Save longitudinal jerks
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.longitudinal_jerks;
    bag_writer.write(msg, "/trajectory/longitudinal_jerks", normalized_timestamp);
  }

  // Save TTC values
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.ttc_values;
    bag_writer.write(msg, "/trajectory/ttc_values", normalized_timestamp);
  }

  // Save lateral deviations
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.lateral_deviations;
    bag_writer.write(msg, "/trajectory/lateral_deviations", normalized_timestamp);
  }

  // Save travel distances
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = metrics.travel_distances;
    bag_writer.write(msg, "/trajectory/travel_distances", normalized_timestamp);
  }
}

}  // namespace autoware::planning_data_analyzer

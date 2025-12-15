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

#ifndef OR_EVENT_EXTRACTOR_HPP_
#define OR_EVENT_EXTRACTOR_HPP_

#include "data_types.hpp"
#include "or_scene_structs.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer
{

/**
 * @brief Extracts Override (OR) events from rosbag data (Stage 1)
 *
 * This class performs the first stage of OR scene evaluation:
 * - Opens rosbag and reads operation_mode_state topic
 * - Detects transitions from AUTONOMOUS to LOCAL mode
 * - Records OR event timestamps and vehicle state
 * - Saves/loads OR events to/from JSON for reuse
 *
 * Usage:
 *   OREventExtractor extractor(logger, 0.5);  // 0.5s window on each side
 *   auto events = extractor.extract_or_events_from_bag(bag_path);
 *   extractor.save_or_events_to_json(events, "or_events.json");
 */
class OREventExtractor
{
public:
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

  /**
   * @brief Constructor
   * @param logger ROS logger for output
   * @param time_window_sec Time window on each side of OR event (default: 0.5s)
   *                        Creates evaluation window: [OR - time_window, OR + time_window]
   */
  explicit OREventExtractor(rclcpp::Logger logger, double time_window_sec = 0.5);

  /**
   * @brief Extract OR events from rosbag
   * @param bag_path Path to input rosbag file
   * @param control_mode_topic Topic name for vehicle control mode
   * @param kinematic_state_topic Topic name for vehicle odometry (to get speed at OR)
   * @return Vector of detected OR events
   */
  std::vector<OREvent> extract_or_events_from_bag(
    const std::string & bag_path,
    const std::string & control_mode_topic = "/vehicle/status/control_mode",
    const std::string & kinematic_state_topic = "/localization/kinematic_state");

  /**
   * @brief Save OR events to JSON file
   * @param events Vector of OR events
   * @param output_path Path to output JSON file
   */
  void save_or_events_to_json(const std::vector<OREvent> & events, const std::string & output_path);

  /**
   * @brief Load OR events from JSON file
   * @param json_path Path to JSON file
   * @return Vector of OR events
   */
  std::vector<OREvent> load_or_events_from_json(const std::string & json_path);

private:
  rclcpp::Logger logger_;
  double time_window_sec_;

  /**
   * @brief Check if mode transition is an OR event (AUTONOMOUS → LOCAL)
   * @param prev_mode Previous operation mode
   * @param current_mode Current operation mode
   * @return true if this is an OR transition
   */
  bool is_or_transition(uint8_t prev_mode, uint8_t current_mode) const;

  /**
   * @brief Get human-readable mode name
   * @param mode Operation mode value
   * @return Mode name string (e.g., "AUTONOMOUS", "LOCAL")
   */
  std::string get_mode_name(uint8_t mode) const;

  /**
   * @brief Find closest odometry message to given timestamp
   * @param target_time Timestamp to find odometry for
   * @param odometry_buffer Buffer of odometry messages
   * @return Closest odometry message, or nullptr if not found
   */
  std::shared_ptr<Odometry> find_closest_odometry(
    const rclcpp::Time & target_time,
    const std::vector<std::shared_ptr<Odometry>> & odometry_buffer,
    const rclcpp::Duration & tolerance = rclcpp::Duration(1, 0)) const;
};

}  // namespace autoware::planning_data_analyzer

#endif  // OR_EVENT_EXTRACTOR_HPP_

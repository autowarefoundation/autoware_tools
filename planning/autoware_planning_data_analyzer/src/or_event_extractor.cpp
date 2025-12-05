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

#include "or_event_extractor.hpp"
#include "utils/bag_utils.hpp"
#include "utils/json_utils.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

namespace autoware::planning_data_analyzer
{

OREventExtractor::OREventExtractor(rclcpp::Logger logger, double time_window_sec)
: logger_(logger), time_window_sec_(time_window_sec)
{
  RCLCPP_INFO(logger_, "OREventExtractor initialized with time_window=%.2fs", time_window_sec_);
}

std::vector<OREvent> OREventExtractor::extract_or_events_from_bag(
  const std::string & bag_path, const std::string & control_mode_topic,
  const std::string & kinematic_state_topic)
{
  RCLCPP_INFO(
    logger_, "Extracting OR events from bag: %s", bag_path.c_str());
  RCLCPP_INFO(
    logger_, "  Control mode topic: %s", control_mode_topic.c_str());
  RCLCPP_INFO(
    logger_, "  Kinematic state topic: %s", kinematic_state_topic.c_str());

  std::vector<OREvent> or_events;

  // Open bag
  rosbag2_cpp::Reader bag_reader;
  if (!utils::open_bag(bag_reader, bag_path, logger_)) {
    return or_events;
  }

  // Filter topics
  utils::set_topic_filter(bag_reader, {control_mode_topic, kinematic_state_topic});

  // State tracking
  uint8_t prev_mode = 255;  // Invalid initial value
  bool first_message = true;

  // Buffer for odometry messages (for speed at OR)
  std::vector<std::shared_ptr<Odometry>> odometry_buffer;
  constexpr size_t MAX_ODOMETRY_BUFFER_SIZE = 1000;

  // Read bag
  while (bag_reader.has_next()) {
    auto bag_message = bag_reader.read_next();

    // Process control mode messages
    if (bag_message->topic_name == control_mode_topic) {
      auto control_mode_msg = utils::deserialize_message<ControlModeReport>(bag_message);

      const uint8_t current_mode = control_mode_msg->mode;

      // Check for OR transition (skip first message)
      if (!first_message && is_or_transition(prev_mode, current_mode)) {
        OREvent event;
        event.timestamp = rclcpp::Time(control_mode_msg->stamp);
        event.prev_mode = prev_mode;
        event.new_mode = current_mode;
        event.prev_mode_str = get_mode_name(prev_mode);
        event.new_mode_str = get_mode_name(current_mode);

        // Calculate time window (symmetric around OR)
        const rclcpp::Duration window_duration(
          static_cast<int32_t>(time_window_sec_),
          static_cast<uint32_t>((time_window_sec_ - static_cast<int32_t>(time_window_sec_)) * 1e9));
        event.time_window = window_duration;
        event.window_start = event.timestamp - window_duration;
        event.window_end = event.timestamp + window_duration;

        // Find vehicle speed and position at OR time
        auto closest_odom = find_closest_odometry(event.timestamp, odometry_buffer);
        if (closest_odom) {
          event.vehicle_speed_mps = closest_odom->twist.twist.linear.x;
          event.vehicle_x_at_or = closest_odom->pose.pose.position.x;
          event.vehicle_y_at_or = closest_odom->pose.pose.position.y;
        } else {
          event.vehicle_speed_mps = 0.0;
          event.vehicle_x_at_or = 0.0;
          event.vehicle_y_at_or = 0.0;
          RCLCPP_WARN(
            logger_, "Could not find odometry for OR event at time %.3f",
            event.timestamp.seconds());
        }

        or_events.push_back(event);

        RCLCPP_INFO(
          logger_, "OR Event #%zu detected at t=%.3fs: %s → %s (speed: %.2f m/s)", or_events.size(),
          event.timestamp.seconds(), event.prev_mode_str.c_str(), event.new_mode_str.c_str(),
          event.vehicle_speed_mps);
      }

      prev_mode = current_mode;
      first_message = false;
    }
    // Process odometry messages (buffer for speed lookup)
    else if (bag_message->topic_name == kinematic_state_topic) {
      auto odometry_msg = utils::deserialize_message<Odometry>(bag_message);
      odometry_buffer.push_back(odometry_msg);

      // Keep buffer size reasonable
      if (odometry_buffer.size() > MAX_ODOMETRY_BUFFER_SIZE) {
        odometry_buffer.erase(odometry_buffer.begin(), odometry_buffer.begin() + 100);
      }
    }
  }

  RCLCPP_INFO(logger_, "Extraction complete: %zu OR events found", or_events.size());

  return or_events;
}

void OREventExtractor::save_or_events_to_json(
  const std::vector<OREvent> & events, const std::string & output_path)
{
  nlohmann::json j;
  j["time_window_sec"] = time_window_sec_;
  j["total_or_events"] = events.size();

  nlohmann::json events_array = nlohmann::json::array();
  for (size_t i = 0; i < events.size(); ++i) {
    const auto & event = events[i];
    nlohmann::json event_json;

    event_json["event_id"] = i;
    event_json["timestamp"] = event.timestamp.seconds();
    event_json["window_start"] = event.window_start.seconds();
    event_json["window_end"] = event.window_end.seconds();
    event_json["time_window_sec"] = event.time_window.seconds();

    event_json["vehicle_speed_mps"] = event.vehicle_speed_mps;
    event_json["vehicle_x_at_or"] = event.vehicle_x_at_or;
    event_json["vehicle_y_at_or"] = event.vehicle_y_at_or;
    event_json["prev_mode"] = event.prev_mode;
    event_json["new_mode"] = event.new_mode;
    event_json["prev_mode_str"] = event.prev_mode_str;
    event_json["new_mode_str"] = event.new_mode_str;

    events_array.push_back(event_json);
  }
  j["or_events"] = events_array;

  // Write to file
  if (utils::save_json_to_file(j, output_path, logger_)) {
    RCLCPP_INFO(logger_, "Saved %zu OR events", events.size());
  }
}

std::vector<OREvent> OREventExtractor::load_or_events_from_json(const std::string & json_path)
{
  std::vector<OREvent> events;

  auto j = utils::load_json_from_file(json_path, logger_);
  if (j.empty() || !j.contains("or_events")) {
    RCLCPP_ERROR(logger_, "Invalid JSON file or missing 'or_events' field");
    return events;
  }

  for (const auto & event_json : j["or_events"]) {
    OREvent event;

    const double timestamp_sec = event_json["timestamp"];
    const double window_start_sec = event_json["window_start"];
    const double window_end_sec = event_json["window_end"];
    const double time_window_sec = event_json["time_window_sec"];

    event.timestamp = rclcpp::Time(static_cast<int64_t>(timestamp_sec * 1e9), RCL_ROS_TIME);
    event.window_start = rclcpp::Time(static_cast<int64_t>(window_start_sec * 1e9), RCL_ROS_TIME);
    event.window_end = rclcpp::Time(static_cast<int64_t>(window_end_sec * 1e9), RCL_ROS_TIME);
    event.time_window = rclcpp::Duration(
      static_cast<int32_t>(time_window_sec),
      static_cast<uint32_t>((time_window_sec - static_cast<int32_t>(time_window_sec)) * 1e9));

    event.vehicle_speed_mps = event_json["vehicle_speed_mps"];
    event.vehicle_x_at_or = event_json.value("vehicle_x_at_or", 0.0);
    event.vehicle_y_at_or = event_json.value("vehicle_y_at_or", 0.0);
    event.prev_mode = event_json["prev_mode"];
    event.new_mode = event_json["new_mode"];
    event.prev_mode_str = event_json["prev_mode_str"];
    event.new_mode_str = event_json["new_mode_str"];

    events.push_back(event);
  }

  RCLCPP_INFO(logger_, "Loaded %zu OR events from: %s", events.size(), json_path.c_str());

  return events;
}

bool OREventExtractor::is_or_transition(uint8_t prev_mode, uint8_t current_mode) const
{
  // OR event: AUTONOMOUS (1) → MANUAL (4)
  return (prev_mode == ControlModeReport::AUTONOMOUS) &&
         (current_mode == ControlModeReport::MANUAL);
}

std::string OREventExtractor::get_mode_name(uint8_t mode) const
{
  switch (mode) {
    case ControlModeReport::NO_COMMAND:
      return "NO_COMMAND";
    case ControlModeReport::AUTONOMOUS:
      return "AUTONOMOUS";
    case ControlModeReport::AUTONOMOUS_STEER_ONLY:
      return "AUTONOMOUS_STEER_ONLY";
    case ControlModeReport::AUTONOMOUS_VELOCITY_ONLY:
      return "AUTONOMOUS_VELOCITY_ONLY";
    case ControlModeReport::MANUAL:
      return "MANUAL";
    case ControlModeReport::DISENGAGED:
      return "DISENGAGED";
    case ControlModeReport::NOT_READY:
      return "NOT_READY";
    default:
      return "INVALID";
  }
}

std::shared_ptr<Odometry> OREventExtractor::find_closest_odometry(
  const rclcpp::Time & target_time, const std::vector<std::shared_ptr<Odometry>> & odometry_buffer,
  const rclcpp::Duration & tolerance) const
{
  if (odometry_buffer.empty()) {
    return nullptr;
  }

  std::shared_ptr<Odometry> closest = nullptr;
  double min_time_diff = std::numeric_limits<double>::max();

  for (const auto & odom : odometry_buffer) {
    const rclcpp::Time odom_time(odom->header.stamp);
    const double time_diff = std::abs((target_time - odom_time).seconds());

    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      closest = odom;
    }
  }

  // Check if within tolerance
  if (min_time_diff > tolerance.seconds()) {
    return nullptr;
  }

  return closest;
}

}  // namespace autoware::planning_data_analyzer

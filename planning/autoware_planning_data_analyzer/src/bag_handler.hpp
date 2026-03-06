// Copyright 2024 TIER IV, Inc.
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

#ifndef BAG_HANDLER_HPP_
#define BAG_HANDLER_HPP_

#include "buffer.hpp"
#include "data_types.hpp"
#include "serialized_bag_message.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::planning_data_analyzer
{

struct BagData
{
  // Template helper to create and configure buffer
  template <typename MessageType>
  void create_buffer(
    const std::string & topic_name, const double buffer_duration_sec, const size_t max_buffer_msgs)
  {
    auto buffer = std::make_shared<Buffer<MessageType>>();
    buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(topic_name, buffer);
  }

  // Constructor for backward compatibility (without topic names)
  explicit BagData(
    const rcutils_time_point_value_t timestamp, const double buffer_duration_sec = 20.0,
    const size_t max_buffer_msgs = 10000)
  : timestamp{timestamp}
  {
    // Default topic names for backward compatibility
    create_buffer<TFMessage>("/tf", buffer_duration_sec, max_buffer_msgs);
    create_buffer<Odometry>("/localization/kinematic_state", buffer_duration_sec, max_buffer_msgs);
    create_buffer<AccelWithCovarianceStamped>(
      "/localization/acceleration", buffer_duration_sec, max_buffer_msgs);
    create_buffer<Trajectory>(
      "/planning/scenario_planning/trajectory", buffer_duration_sec, max_buffer_msgs);
    create_buffer<PredictedObjects>(
      "/perception/object_recognition/objects", buffer_duration_sec, max_buffer_msgs);
    create_buffer<SteeringReport>(
      "/vehicle/status/steering_status", buffer_duration_sec, max_buffer_msgs);
  }

  // Constructor with topic names
  explicit BagData(
    const rcutils_time_point_value_t timestamp, const TopicNames & topic_names,
    const double buffer_duration_sec = 20.0, const size_t max_buffer_msgs = 10000)
  : timestamp{timestamp}
  {
    // Create buffers using provided topic names
    create_buffer<TFMessage>(topic_names.tf_topic, buffer_duration_sec, max_buffer_msgs);
    create_buffer<Odometry>(topic_names.odometry_topic, buffer_duration_sec, max_buffer_msgs);
    create_buffer<AccelWithCovarianceStamped>(
      topic_names.acceleration_topic, buffer_duration_sec, max_buffer_msgs);
    create_buffer<Trajectory>(topic_names.trajectory_topic, buffer_duration_sec, max_buffer_msgs);
    create_buffer<PredictedObjects>(
      topic_names.objects_topic, buffer_duration_sec, max_buffer_msgs);
    create_buffer<SteeringReport>(topic_names.steering_topic, buffer_duration_sec, max_buffer_msgs);
  }

  rcutils_time_point_value_t timestamp;

  std::map<std::string, std::shared_ptr<BufferBase>> buffers{};

  auto get_synchronized_data_at_time(
    const rcutils_time_point_value_t target_time, const double tolerance_ms = 50.0) const
    -> std::shared_ptr<SynchronizedData>
  {
    auto synchronized_data = std::make_shared<SynchronizedData>();
    synchronized_data->timestamp = rclcpp::Time(target_time);
    synchronized_data->bag_timestamp =
      rclcpp::Time(target_time);  // Set bag_timestamp to the requested time

    // Get odometry buffer
    // Find odometry buffer by checking all buffers
    std::shared_ptr<Buffer<Odometry>> odom_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto ob = std::dynamic_pointer_cast<Buffer<Odometry>>(buffer)) {
        odom_buffer = ob;
        break;
      }
    }
    if (!odom_buffer) return nullptr;

    synchronized_data->kinematic_state = odom_buffer->get_closest(target_time, tolerance_ms);
    if (!synchronized_data->kinematic_state) return nullptr;

    // Get trajectory
    // Find trajectory buffer
    std::shared_ptr<Buffer<Trajectory>> traj_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto tb = std::dynamic_pointer_cast<Buffer<Trajectory>>(buffer)) {
        traj_buffer = tb;
        break;
      }
    }
    if (traj_buffer) {
      synchronized_data->trajectory = traj_buffer->get_closest(target_time, tolerance_ms);

      // CRITICAL: Resync all data to trajectory's actual timestamp for accurate GT
      if (synchronized_data->trajectory) {
        const auto traj_stamp_ns =
          rclcpp::Time(synchronized_data->trajectory->header.stamp).nanoseconds();
        synchronized_data->timestamp = rclcpp::Time(synchronized_data->trajectory->header.stamp);

        // Resync kinematic_state to trajectory timestamp (not sampling time!)
        if (odom_buffer) {
          synchronized_data->kinematic_state =
            odom_buffer->get_closest(traj_stamp_ns, tolerance_ms);
        }
      }
    }

    // Get acceleration
    // Find acceleration buffer
    std::shared_ptr<Buffer<AccelWithCovarianceStamped>> accel_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto ab = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(buffer)) {
        accel_buffer = ab;
        break;
      }
    }
    if (accel_buffer && synchronized_data->trajectory) {
      const auto traj_stamp_ns =
        rclcpp::Time(synchronized_data->trajectory->header.stamp).nanoseconds();
      synchronized_data->acceleration = accel_buffer->get_closest(traj_stamp_ns, tolerance_ms);
    } else if (accel_buffer) {
      synchronized_data->acceleration = accel_buffer->get_closest(target_time, tolerance_ms);
    }

    // Get steering status
    // Find steering buffer
    std::shared_ptr<Buffer<SteeringReport>> steer_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto sb = std::dynamic_pointer_cast<Buffer<SteeringReport>>(buffer)) {
        steer_buffer = sb;
        break;
      }
    }
    if (steer_buffer && synchronized_data->trajectory) {
      const auto traj_stamp_ns =
        rclcpp::Time(synchronized_data->trajectory->header.stamp).nanoseconds();
      synchronized_data->steering_status = steer_buffer->get_closest(traj_stamp_ns, tolerance_ms);
    } else if (steer_buffer) {
      synchronized_data->steering_status = steer_buffer->get_closest(target_time, tolerance_ms);
    }

    // Get objects
    // Find objects buffer
    std::shared_ptr<Buffer<PredictedObjects>> obj_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto ob = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(buffer)) {
        obj_buffer = ob;
        break;
      }
    }
    if (obj_buffer && synchronized_data->trajectory) {
      const auto traj_stamp_ns =
        rclcpp::Time(synchronized_data->trajectory->header.stamp).nanoseconds();
      synchronized_data->objects = obj_buffer->get_closest(traj_stamp_ns, tolerance_ms);
    } else if (obj_buffer) {
      synchronized_data->objects = obj_buffer->get_closest(target_time, tolerance_ms);
    }

    return synchronized_data;
  }

  // Template helper to append message to appropriate buffer
  template <typename MessageType>
  bool append_message(const std::string & topic_name, const MessageType & msg)
  {
    auto buffer = std::dynamic_pointer_cast<Buffer<MessageType>>(buffers[topic_name]);
    if (buffer) {
      buffer->append(msg);
      return true;
    }
    return false;
  }

  // Template helper to get buffer for a specific message type
  template <typename MessageType>
  std::shared_ptr<Buffer<MessageType>> get_buffer(const std::string & topic_name)
  {
    return std::dynamic_pointer_cast<Buffer<MessageType>>(buffers[topic_name]);
  }

  auto get_kinematic_states_at_interval(const double interval_ms = 100.0) const
    -> std::vector<std::shared_ptr<Odometry>>
  {
    std::vector<std::shared_ptr<Odometry>> result;

    // Find odometry buffer by checking all buffers
    std::shared_ptr<Buffer<Odometry>> odom_buffer;
    for (const auto & [topic, buffer] : buffers) {
      if (auto ob = std::dynamic_pointer_cast<Buffer<Odometry>>(buffer)) {
        odom_buffer = ob;
        break;
      }
    }
    if (!odom_buffer || odom_buffer->msgs.empty()) {
      return result;
    }

    // Get first and last timestamp from odometry messages
    const auto first_time = rclcpp::Time(odom_buffer->msgs.front().header.stamp).nanoseconds();
    const auto last_time = rclcpp::Time(odom_buffer->msgs.back().header.stamp).nanoseconds();
    const auto interval_ns = static_cast<rcutils_time_point_value_t>(interval_ms * 1e6);

    // Sample at regular intervals
    for (auto current_time = first_time; current_time <= last_time; current_time += interval_ns) {
      auto odom = odom_buffer->get_closest(current_time, interval_ms / 2.0);
      if (odom) {
        result.push_back(odom);
      }
    }

    return result;
  }
};

// Helper trait to detect if a type has header.stamp (C++17 compatible)
template <typename T, typename = void>
struct has_header_stamp : std::false_type
{
};

template <typename T>
struct has_header_stamp<T, std::void_t<decltype(std::declval<T>().header.stamp)>> : std::true_type
{
};

// Template helper to set timestamp for messages with header
template <typename MessageType>
typename std::enable_if<has_header_stamp<MessageType>::value, void>::type
set_header_timestamp_if_needed(
  MessageType & msg, bool use_bag_timestamp, const rclcpp::Time & bag_time)
{
  if (use_bag_timestamp && msg.header.stamp != rclcpp::Time(0)) {
    msg.header.stamp = bag_time;
  }
}

// Template helper for messages without header - does nothing
template <typename MessageType>
typename std::enable_if<!has_header_stamp<MessageType>::value, void>::type
set_header_timestamp_if_needed(MessageType &, bool, const rclcpp::Time &)
{
  // No-op for messages without header.stamp
}

// Template helper to process and append message to bag data
template <typename MessageType>
void process_and_append_message(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & serialized_message,
  std::shared_ptr<BagData> bag_data, const std::string & topic_key, bool use_bag_timestamp,
  rclcpp::Logger logger)
{
  try {
    MessageType msg;
    rclcpp::Serialization<MessageType> serializer;
    rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
    serializer.deserialize_message(&serialized_msg, &msg);

    // Override header timestamp with bag timestamp if option is enabled
    set_header_timestamp_if_needed(
      msg, use_bag_timestamp, rclcpp::Time(get_timestamp_ns(*serialized_message)));

    bag_data->append_message<MessageType>(topic_key, msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger, "Failed to deserialize message on topic %s: %s",
      serialized_message->topic_name.c_str(), e.what());
  }
}

}  // namespace autoware::planning_data_analyzer

#endif  // BAG_HANDLER_HPP_

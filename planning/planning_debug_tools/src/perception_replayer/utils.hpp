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

#ifndef PERCEPTION_REPLAYER__UTILS_HPP_
#define PERCEPTION_REPLAYER__UTILS_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::planning_debug_tools::utils
{

template <class MessageT>
using DataStamped = std::pair<rclcpp::Time, MessageT>;

// binary search to find the index of the nearest timestamp
template <class MessageT>
size_t get_nearest_index(
  const std::vector<DataStamped<MessageT>> & stamped_messages, const rclcpp::Time & timestamp)
{
  if (stamped_messages.empty()) {
    throw std::runtime_error("No messages available to find nearest index.");
  }

  if (stamped_messages.front().first > timestamp) {
    return 0;
  }
  if (stamped_messages.back().first < timestamp) {
    return stamped_messages.size() - 1;
  }

  size_t low = 0;
  size_t high = stamped_messages.size() - 1;

  while (low <= high) {
    const size_t mid = low + (high - low) / 2;
    if (stamped_messages[mid].first < timestamp) {
      low = mid + 1;
    } else if (stamped_messages[mid].first > timestamp) {
      high = mid - 1;
    } else {
      return mid;
    }
  }

  // Return the next timestamp's data if available
  if (low < stamped_messages.size()) {
    return low;
  }
  throw std::runtime_error("No suitable timestamp found.");
}

// find message by timestamp with optional return
template <class MessageT>
std::optional<MessageT> find_message_by_timestamp(
  const std::vector<DataStamped<MessageT>> & stamped_messages, const rclcpp::Time & timestamp)
{
  if (stamped_messages.empty()) {
    return std::nullopt;
  }

  try {
    const size_t idx = get_nearest_index(stamped_messages, timestamp);
    return stamped_messages.at(idx).second;
  } catch (const std::exception & e) {
    return std::nullopt;
  }
}

template <class MessageT>
std::shared_ptr<MessageT> deserialize_message(
  const std::shared_ptr<rcutils_uint8_array_t> & serialized_data)
{
  rclcpp::SerializedMessage serialized_msg(*serialized_data);
  auto msg = std::make_shared<MessageT>();
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized_msg, msg.get());

  return msg;
}

// get yaw from quaternion
inline double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// get quaternion from yaw
inline geometry_msgs::msg::Quaternion get_quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(tf_quat);
}

// calculate 2D speed from twist
inline double calculate_speed_2d(const geometry_msgs::msg::Twist & twist)
{
  return std::sqrt(std::pow(twist.linear.x, 2.0) + std::pow(twist.linear.y, 2.0));
}

// calculate 2D distance between two points
inline double calculate_distance_2d(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0));
}

// translate objects coordinate from log ego pose to current ego pose
template <typename ObjectsT>
void translate_objects_coordinate(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & log_ego_pose,
  ObjectsT & objects_msg)
{
  const double log_ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation);
  const double ego_yaw = get_yaw_from_quaternion(ego_pose.orientation);

  // create transformation matrices
  const double cos_log = std::cos(log_ego_yaw);
  const double sin_log = std::sin(log_ego_yaw);
  const double cos_ego = std::cos(ego_yaw);
  const double sin_ego = std::sin(ego_yaw);

  // log_ego_pose transformation matrix (3x3 for 2D homogeneous coordinates)
  // [cos -sin x]
  // [sin  cos y]
  // [0    0   1]

  for (auto & object : objects_msg.objects) {
    auto & log_object_pose = object.kinematics.pose_with_covariance.pose;
    const double log_object_yaw = get_yaw_from_quaternion(log_object_pose.orientation);

    // transform object position from log ego frame to map frame
    const double log_obj_x = log_object_pose.position.x;
    const double log_obj_y = log_object_pose.position.y;

    // apply log_ego_pose transformation: map_pos = log_ego_pose_trans_mat * log_obj_pos
    const double map_obj_x = cos_log * log_obj_x - sin_log * log_obj_y + log_ego_pose.position.x;
    const double map_obj_y = sin_log * log_obj_x + cos_log * log_obj_y + log_ego_pose.position.y;

    // apply inverse ego_pose transformation: obj_pos = inv(ego_pose_trans_mat) * map_obj_pos
    const double rel_x = map_obj_x - ego_pose.position.x;
    const double rel_y = map_obj_y - ego_pose.position.y;

    log_object_pose.position.x = cos_ego * rel_x + sin_ego * rel_y;
    log_object_pose.position.y = -sin_ego * rel_x + cos_ego * rel_y;

    // transform orientation
    const double new_yaw = log_object_yaw + log_ego_yaw - ego_yaw;
    log_object_pose.orientation = get_quaternion_from_yaw(new_yaw);
  }
}

}  // namespace autoware::planning_debug_tools::utils

#endif  // PERCEPTION_REPLAYER__UTILS_HPP_

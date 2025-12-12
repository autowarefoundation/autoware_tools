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

#ifndef UTILS__BAG_UTILS_HPP_
#define UTILS__BAG_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::utils
{

/**
 * @brief Open a rosbag file with error handling
 * @param reader Reader object to open
 * @param bag_path Path to the bag file
 * @param logger ROS logger for error messages
 * @return true if successful, false otherwise
 */
bool open_bag(
  rosbag2_cpp::Reader & reader, const std::string & bag_path, const rclcpp::Logger & logger);

/**
 * @brief Set topic filter on bag reader
 * @param reader Bag reader to configure
 * @param topics List of topic names to read
 */
void set_topic_filter(rosbag2_cpp::Reader & reader, const std::vector<std::string> & topics);

/**
 * @brief Deserialize message from bag
 * @tparam T Message type
 * @param serialized_msg Serialized message from bag
 * @return Shared pointer to deserialized message
 */
template <typename T>
std::shared_ptr<T> deserialize_message(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  auto msg = std::make_shared<T>();
  rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
  rclcpp::Serialization<T> serialization;
  serialization.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

}  // namespace autoware::planning_data_analyzer::utils

#endif  // UTILS__BAG_UTILS_HPP_

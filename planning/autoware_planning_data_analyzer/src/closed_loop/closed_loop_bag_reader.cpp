// Copyright 2026 TIER IV, Inc.
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

#include "closed_loop/closed_loop_bag_reader.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

std::string resolve_bag_uri(const std::string & bag_path)
{
  const std::filesystem::path path(bag_path);
  if (
    std::filesystem::is_regular_file(path) &&
    std::filesystem::exists(path.parent_path() / "metadata.yaml")) {
    return path.parent_path().string();
  }
  return path.string();
}

template <typename MessageType>
MessageType deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  MessageType message;
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  rclcpp::Serialization<MessageType>().deserialize_message(&serialized, &message);
  return message;
}

builtin_interfaces::msg::Time shifted(
  const builtin_interfaces::msg::Time & stamp, const std::int64_t offset_ns)
{
  return rclcpp::Time(rclcpp::Time(stamp, RCL_ROS_TIME).nanoseconds() + offset_ns, RCL_ROS_TIME);
}

}  // namespace

ClosedLoopBagContents read_closed_loop_bag(
  const std::string & bag_path, const ClosedLoopBagTopics & topics, const rclcpp::Time & from,
  const rclcpp::Time & to, const std::int64_t time_offset_ns)
{
  rosbag2_cpp::Reader reader;
  try {
    reader.open(resolve_bag_uri(bag_path));
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot open bag " + bag_path + ": " + error.what());
  }
  rosbag2_storage::StorageFilter filter;
  for (const auto & topic : {topics.odometry, topics.tracked_objects, topics.route, topics.map}) {
    if (!topic.empty()) {
      filter.topics.push_back(topic);
    }
  }
  reader.set_filter(filter);

  const auto in_range = [&](const builtin_interfaces::msg::Time & stamp) {
    const rclcpp::Time time(stamp, RCL_ROS_TIME);
    return time >= from && time <= to;
  };

  ClosedLoopBagContents contents;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const auto & topic = bag_message->topic_name;
    if (!topics.odometry.empty() && topic == topics.odometry) {
      auto odometry = deserialize<Odometry>(*bag_message);
      odometry.header.stamp = shifted(odometry.header.stamp, time_offset_ns);
      if (in_range(odometry.header.stamp)) {
        contents.odometry.push_back(std::move(odometry));
      }
    } else if (!topics.tracked_objects.empty() && topic == topics.tracked_objects) {
      auto objects = std::make_shared<TrackedObjects>(deserialize<TrackedObjects>(*bag_message));
      objects->header.stamp = shifted(objects->header.stamp, time_offset_ns);
      if (in_range(objects->header.stamp)) {
        const rclcpp::Time stamp(objects->header.stamp, RCL_ROS_TIME);
        contents.tracked_objects.push_back(TimedTrackedObjects{stamp, std::move(objects)});
      }
    } else if (!topics.route.empty() && topic == topics.route) {
      contents.routes.push_back(
        deserialize<autoware_planning_msgs::msg::LaneletRoute>(*bag_message));
    } else if (!topics.map.empty() && topic == topics.map) {
      contents.map = deserialize<autoware_map_msgs::msg::LaneletMapBin>(*bag_message);
    }
  }

  std::stable_sort(
    contents.odometry.begin(), contents.odometry.end(), [](const auto & lhs, const auto & rhs) {
      return rclcpp::Time(lhs.header.stamp, RCL_ROS_TIME) <
             rclcpp::Time(rhs.header.stamp, RCL_ROS_TIME);
    });
  std::stable_sort(
    contents.tracked_objects.begin(), contents.tracked_objects.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.stamp < rhs.stamp; });
  return contents;
}

std::optional<std::int64_t> estimate_time_offset(
  const std::vector<Odometry> & executed, const std::vector<Odometry> & recorded,
  const rclcpp::Time & match_start, const rclcpp::Time & match_end, const double min_speed_mps)
{
  if (executed.empty()) {
    return std::nullopt;
  }
  std::vector<std::int64_t> differences;
  for (const auto & sample : recorded) {
    const rclcpp::Time time(sample.header.stamp, RCL_ROS_TIME);
    const double speed = std::hypot(sample.twist.twist.linear.x, sample.twist.twist.linear.y);
    if (time < match_start || time >= match_end || speed < min_speed_mps) {
      continue;
    }
    const auto & position = sample.pose.pose.position;
    const auto nearest = std::min_element(
      executed.begin(), executed.end(), [&](const Odometry & lhs, const Odometry & rhs) {
        const auto & a = lhs.pose.pose.position;
        const auto & b = rhs.pose.pose.position;
        return std::hypot(a.x - position.x, a.y - position.y) <
               std::hypot(b.x - position.x, b.y - position.y);
      });
    differences.push_back(
      rclcpp::Time(nearest->header.stamp, RCL_ROS_TIME).nanoseconds() - time.nanoseconds());
  }
  if (differences.empty()) {
    return std::nullopt;
  }
  std::nth_element(
    differences.begin(), differences.begin() + static_cast<std::ptrdiff_t>(differences.size() / 2),
    differences.end());
  return differences.at(differences.size() / 2);
}

std::optional<autoware_planning_msgs::msg::LaneletRoute> select_route(
  const std::vector<autoware_planning_msgs::msg::LaneletRoute> & routes,
  const rclcpp::Time & window_start)
{
  std::optional<autoware_planning_msgs::msg::LaneletRoute> selected;
  for (const auto & route : routes) {
    if (rclcpp::Time(route.header.stamp, RCL_ROS_TIME) <= window_start) {
      selected = route;
    }
  }
  if (!selected.has_value() && !routes.empty()) {
    selected = routes.front();
  }
  return selected;
}

}  // namespace autoware::planning_data_analyzer::closed_loop

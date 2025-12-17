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

#include "buffer.hpp"

#include <limits>
#include <memory>

namespace autoware::planning_data_analyzer
{

// Template specializations for SteeringReport (uses stamp instead of header.stamp)
template <>
bool Buffer<SteeringReport>::ready() const
{
  if (msgs.empty()) {
    return false;
  }

  return rclcpp::Time(msgs.back().stamp).nanoseconds() -
           rclcpp::Time(msgs.front().stamp).nanoseconds() >
         buffer_time_ns;
}

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now)
{
  if (msgs.empty()) {
    return;
  }

  const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
    return rclcpp::Time(msg.stamp).nanoseconds() < now;
  });
  msgs.erase(itr, msgs.end());
}

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr
{
  const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
    return rclcpp::Time(msg.stamp).nanoseconds() > now;
  });

  if (itr == msgs.end()) {
    return nullptr;
  }

  return std::make_shared<SteeringReport>(*itr);
}

template <>
auto Buffer<SteeringReport>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const
  -> SteeringReport::SharedPtr
{
  if (msgs.empty()) {
    return nullptr;
  }

  const double tolerance_ns = tolerance_ms * 1e6;

  auto closest_itr = msgs.begin();
  double min_diff =
    std::abs(static_cast<double>(rclcpp::Time(closest_itr->stamp).nanoseconds() - target_time));

  for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
    const double diff =
      std::abs(static_cast<double>(rclcpp::Time(itr->stamp).nanoseconds() - target_time));
    if (diff < min_diff) {
      min_diff = diff;
      closest_itr = itr;
    }
  }

  if (min_diff > tolerance_ns) {
    return nullptr;
  }

  return std::make_shared<SteeringReport>(*closest_itr);
}

// Template specializations for TFMessage (uses transforms.front().header.stamp)
template <>
bool Buffer<TFMessage>::ready() const
{
  if (msgs.empty()) {
    return false;
  }

  if (msgs.front().transforms.empty()) {
    return false;
  }

  if (msgs.back().transforms.empty()) {
    return false;
  }

  return rclcpp::Time(msgs.back().transforms.front().header.stamp).nanoseconds() -
           rclcpp::Time(msgs.front().transforms.front().header.stamp).nanoseconds() >
         buffer_time_ns;
}

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now)
{
  if (msgs.empty()) {
    return;
  }

  const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
    return rclcpp::Time(msg.transforms.front().header.stamp).nanoseconds() < now;
  });
  msgs.erase(itr, msgs.end());
}

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr
{
  const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
    return rclcpp::Time(msg.transforms.front().header.stamp).nanoseconds() > now;
  });

  if (itr == msgs.end()) {
    return nullptr;
  }

  return std::make_shared<TFMessage>(*itr);
}

template <>
auto Buffer<TFMessage>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const
  -> TFMessage::SharedPtr
{
  if (msgs.empty()) {
    return nullptr;
  }

  const double tolerance_ns = tolerance_ms * 1e6;

  auto closest_itr = msgs.begin();
  double min_diff = std::numeric_limits<double>::max();

  for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
    if (!itr->transforms.empty()) {
      const double diff = std::abs(
        static_cast<double>(
          rclcpp::Time(itr->transforms.front().header.stamp).nanoseconds() - target_time));
      if (diff < min_diff) {
        min_diff = diff;
        closest_itr = itr;
      }
    }
  }

  if (min_diff > tolerance_ns) {
    return nullptr;
  }

  return std::make_shared<TFMessage>(*closest_itr);
}

}  // namespace autoware::planning_data_analyzer

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

#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include "data_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::planning_data_analyzer
{

template <typename T, typename = void>
struct has_header_stamp : std::false_type
{
};

template <typename T>
struct has_header_stamp<T, std::void_t<decltype(std::declval<T>().header.stamp)>> : std::true_type
{
};

template <typename T, typename = void>
struct has_stamp : std::false_type
{
};

template <typename T>
struct has_stamp<T, std::void_t<decltype(std::declval<T>().stamp)>> : std::true_type
{
};

template <typename MessageType>
typename std::enable_if<has_header_stamp<MessageType>::value, rclcpp::Time>::type message_stamp(
  const MessageType & msg)
{
  return rclcpp::Time(msg.header.stamp);
}

template <typename MessageType>
typename std::enable_if<
  !has_header_stamp<MessageType>::value && has_stamp<MessageType>::value, rclcpp::Time>::type
message_stamp(const MessageType & msg)
{
  return rclcpp::Time(msg.stamp);
}

// Base interface for message buffers
struct BufferBase
{
  virtual bool ready() const = 0;
  virtual void remove_old_data(const rcutils_time_point_value_t now) = 0;
  virtual ~BufferBase() = default;
};

// Template buffer for messages with header.stamp
template <typename T>
struct Buffer : BufferBase
{
  std::vector<T> msgs;

  double buffer_time_ns = 20.0 * 1e9;
  size_t max_buffer_size = 10000;

  bool ready() const override
  {
    if (msgs.empty()) {
      return false;
    }

    return message_stamp(msgs.back()).nanoseconds() - message_stamp(msgs.front()).nanoseconds() >
           buffer_time_ns;
  }

  void remove_old_data(const rcutils_time_point_value_t now) override
  {
    const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
      return message_stamp(msg).nanoseconds() < now;
    });
    msgs.erase(itr, msgs.end());
  }

  void append(const T & msg)
  {
    msgs.push_back(msg);

    if (msgs.size() > max_buffer_size) {
      const size_t remove_count = max_buffer_size / 10;
      msgs.erase(msgs.begin(), msgs.begin() + remove_count);
    }
  }

  auto get(const rcutils_time_point_value_t now) const -> typename T::SharedPtr
  {
    const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
      return message_stamp(msg).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return nullptr;
    }

    return std::make_shared<T>(*itr);
  }

  auto get_closest(const rcutils_time_point_value_t target_time, const double tolerance_ms = 50.0)
    const -> typename T::SharedPtr
  {
    if (msgs.empty()) {
      return nullptr;
    }

    const double tolerance_ns = tolerance_ms * 1e6;

    auto closest_itr = msgs.begin();
    double min_diff =
      std::abs(static_cast<double>(message_stamp(*closest_itr).nanoseconds() - target_time));

    for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
      const double diff =
        std::abs(static_cast<double>(message_stamp(*itr).nanoseconds() - target_time));
      if (diff < min_diff) {
        min_diff = diff;
        closest_itr = itr;
      }
    }

    if (min_diff > tolerance_ns) {
      return nullptr;
    }

    return std::make_shared<T>(*closest_itr);
  }

  auto get_latest_before_or_equal(const rcutils_time_point_value_t target_time) const ->
    typename T::SharedPtr
  {
    if (msgs.empty()) {
      return nullptr;
    }

    typename std::vector<T>::const_iterator latest_itr = msgs.end();
    for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
      const auto stamp_ns = message_stamp(*itr).nanoseconds();
      if (stamp_ns <= target_time) {
        latest_itr = itr;
      } else {
        break;
      }
    }

    if (latest_itr == msgs.end()) {
      return nullptr;
    }
    return std::make_shared<T>(*latest_itr);
  }
};

// Template specializations for messages without standard header.stamp
template <>
bool Buffer<SteeringReport>::ready() const;

template <>
bool Buffer<TFMessage>::ready() const;

template <>
bool Buffer<CandidateTrajectories>::ready() const;

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<CandidateTrajectories>::remove_old_data(const rcutils_time_point_value_t now);

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr;

template <>
auto Buffer<CandidateTrajectories>::get(const rcutils_time_point_value_t now) const
  -> CandidateTrajectories::SharedPtr;

template <>
auto Buffer<SteeringReport>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const
  -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const
  -> TFMessage::SharedPtr;

template <>
auto Buffer<CandidateTrajectories>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const
  -> CandidateTrajectories::SharedPtr;

template <>
auto Buffer<CandidateTrajectories>::get_latest_before_or_equal(
  const rcutils_time_point_value_t target_time) const -> CandidateTrajectories::SharedPtr;

}  // namespace autoware::planning_data_analyzer

#endif  // BUFFER_HPP_

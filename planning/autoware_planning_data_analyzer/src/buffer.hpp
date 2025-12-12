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

    return rclcpp::Time(msgs.back().header.stamp).nanoseconds() -
             rclcpp::Time(msgs.front().header.stamp).nanoseconds() >
           buffer_time_ns;
  }

  void remove_old_data(const rcutils_time_point_value_t now) override
  {
    const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() < now;
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
      return rclcpp::Time(msg.header.stamp).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return nullptr;
    }

    return std::make_shared<T>(*itr);
  }

  auto get_closest(
    const rcutils_time_point_value_t target_time, const double tolerance_ms = 50.0) const ->
    typename T::SharedPtr
  {
    if (msgs.empty()) {
      return nullptr;
    }

    const double tolerance_ns = tolerance_ms * 1e6;

    auto closest_itr = msgs.begin();
    double min_diff = std::abs(
      static_cast<double>(rclcpp::Time(closest_itr->header.stamp).nanoseconds() - target_time));

    for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
      const double diff =
        std::abs(static_cast<double>(rclcpp::Time(itr->header.stamp).nanoseconds() - target_time));
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
};

// Template specializations for messages without standard header.stamp
template <>
bool Buffer<SteeringReport>::ready() const;

template <>
bool Buffer<TFMessage>::ready() const;

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now);

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const ->
  SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr;

template <>
auto Buffer<SteeringReport>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const ->
  SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get_closest(
  const rcutils_time_point_value_t target_time, const double tolerance_ms) const ->
  TFMessage::SharedPtr;

}  // namespace autoware::planning_data_analyzer

#endif  // BUFFER_HPP_

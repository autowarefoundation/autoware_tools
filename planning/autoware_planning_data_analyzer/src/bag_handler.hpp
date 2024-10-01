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

#include "data_structs.hpp"
#include "type_alias.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_analyzer
{

struct BufferBase
{
  virtual bool ready() const = 0;
  virtual void remove_old_data(const rcutils_time_point_value_t now) = 0;
};

template <typename T>
struct Buffer : BufferBase
{
  std::vector<T> msgs;

  const double BUFFER_TIME = 20.0 * 1e9;

  bool ready() const override
  {
    if (msgs.empty()) {
      return false;
    }

    return rclcpp::Time(msgs.back().header.stamp).nanoseconds() -
             rclcpp::Time(msgs.front().header.stamp).nanoseconds() >
           BUFFER_TIME;
  }

  void remove_old_data(const rcutils_time_point_value_t now) override
  {
    const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() < now;
    });
    msgs.erase(itr, msgs.end());
  }

  void append(const T & msg) { msgs.push_back(msg); }

  auto get(const rcutils_time_point_value_t now) const -> typename T::SharedPtr
  {
    const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return nullptr;
    }

    return std::make_shared<T>(*itr);
  }
};

template <>
bool Buffer<SteeringReport>::ready() const;

template <>
bool Buffer<TFMessage>::ready() const;

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now);

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr;

struct BagData
{
  explicit BagData(const rcutils_time_point_value_t timestamp) : timestamp{timestamp}
  {
    buffers.emplace(TOPIC::TF, std::make_shared<Buffer<TFMessage>>());
    buffers.emplace(TOPIC::ODOMETRY, std::make_shared<Buffer<Odometry>>());
    buffers.emplace(TOPIC::ACCELERATION, std::make_shared<Buffer<AccelWithCovarianceStamped>>());
    buffers.emplace(TOPIC::TRAJECTORY, std::make_shared<Buffer<Trajectory>>());
    buffers.emplace(TOPIC::OBJECTS, std::make_shared<Buffer<PredictedObjects>>());
    buffers.emplace(TOPIC::STEERING, std::make_shared<Buffer<SteeringReport>>());
  }

  rcutils_time_point_value_t timestamp;

  std::map<std::string, std::shared_ptr<BufferBase>> buffers{};

  void update(const rcutils_time_point_value_t dt)
  {
    timestamp += dt;
    remove_old_data();
  }

  void remove_old_data()
  {
    std::for_each(buffers.begin(), buffers.end(), [this](const auto & buffer) {
      buffer.second->remove_old_data(timestamp);
    });
  }

  bool ready() const
  {
    return std::all_of(
      buffers.begin(), buffers.end(), [](const auto & buffer) { return buffer.second->ready(); });
  }
};

}  // namespace autoware::behavior_analyzer

#endif  // BAG_HANDLER_HPP_

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

#ifndef METRICS__GEOMETRY__OBJECT_TRACKS_HPP_
#define METRICS__GEOMETRY__OBJECT_TRACKS_HPP_

#include "data_types.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct LoggedObjectState
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  autoware_perception_msgs::msg::Shape shape;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
};

struct LoggedObjectTrack
{
  std::array<uint8_t, 16> object_id{};
  bool has_valid_object_id{false};
  std::vector<LoggedObjectState> states;
};

struct InterpolatedLoggedObject
{
  std::array<uint8_t, 16> object_id{};
  bool has_valid_object_id{false};
  geometry_msgs::msg::Pose pose;
  double speed_mps{0.0};
  autoware_perception_msgs::msg::Shape shape;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
  autoware_utils_geometry::Polygon2d polygon;
};

bool has_valid_object_id(const unique_identifier_msgs::msg::UUID & object_id);

std::array<uint8_t, 16> object_id_key(const unique_identifier_msgs::msg::UUID & object_id);

bool is_agent_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);

bool is_unknown_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedTrackedObjects> & future_objects);

std::optional<InterpolatedLoggedObject> interpolate_logged_object_state(
  const LoggedObjectTrack & track, const rclcpp::Time & query_time);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__OBJECT_TRACKS_HPP_

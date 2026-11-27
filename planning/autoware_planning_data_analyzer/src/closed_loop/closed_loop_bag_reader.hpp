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

#ifndef CLOSED_LOOP__CLOSED_LOOP_BAG_READER_HPP_
#define CLOSED_LOOP__CLOSED_LOOP_BAG_READER_HPP_

#include "data_types.hpp"

#include <rclcpp/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// The topics read from a bag; an empty name is not read.
struct ClosedLoopBagTopics
{
  std::string odometry;
  std::string tracked_objects;
  std::string route;
  std::string map;
};

struct ClosedLoopBagContents
{
  // Messages whose header stamp lies in the requested range, sorted by stamp.
  std::vector<Odometry> odometry;
  std::vector<TimedTrackedObjects> tracked_objects;
  // Every route message, in the order of the bag.
  std::vector<autoware_planning_msgs::msg::LaneletRoute> routes;
  // The bag's last map message.
  std::optional<autoware_map_msgs::msg::LaneletMapBin> map;
};

// Reads a bag (a directory with metadata.yaml, or one of its files). Odometry and tracked objects
// are kept when their header stamp lies in [from, to]; time_offset_ns is added to those stamps
// first, so a recorded drive comes out in another bag's clock. Throws std::runtime_error when the
// bag cannot be opened.
ClosedLoopBagContents read_closed_loop_bag(
  const std::string & bag_path, const ClosedLoopBagTopics & topics, const rclcpp::Time & from,
  const rclcpp::Time & to, std::int64_t time_offset_ns = 0);

// The offset (ns) that lines the recorded drive up with the executed one, where a simulation
// replays the recorded drive: over the recorded odometry in [match_start, match_end) while the
// recorded ego moves (min_speed_mps), each sample is paired with the executed sample nearest to it
// in position; the offset is the median of their time differences (executed - recorded). None
// without moving samples (a recorded ego that stood still there) or executed odometry.
std::optional<std::int64_t> estimate_time_offset(
  const std::vector<Odometry> & executed, const std::vector<Odometry> & recorded,
  const rclcpp::Time & match_start, const rclcpp::Time & match_end, double min_speed_mps = 0.5);

// The route the run drove: the last route stamped at or before window_start, else the first one.
std::optional<autoware_planning_msgs::msg::LaneletRoute> select_route(
  const std::vector<autoware_planning_msgs::msg::LaneletRoute> & routes,
  const rclcpp::Time & window_start);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CLOSED_LOOP_BAG_READER_HPP_

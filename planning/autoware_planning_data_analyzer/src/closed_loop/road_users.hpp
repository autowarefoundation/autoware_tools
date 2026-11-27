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

#ifndef CLOSED_LOOP__ROAD_USERS_HPP_
#define CLOSED_LOOP__ROAD_USERS_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/lane_layout.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

enum class RoadUserType { VEHICLE, PEDESTRIAN, BICYCLE };

std::string to_string(RoadUserType type);

// The road users of an episode as columns over its executed rows (one column per object ID, its
// stretches joined), as the reviewed closed-loop metrics lay them out.
struct RoadUserColumns
{
  std::vector<std::string> track_ids;
  std::vector<RoadUserType> types;
  // [row][column]; a column is zero where it is not valid.
  std::vector<std::vector<char>> valid;
  std::vector<std::vector<BoxCorners>> corners;
  std::vector<std::vector<Point2d>> xy;
  std::vector<std::vector<double>> yaw;
  // The fitted speed (m/s), 0 on a row without a fit; fitted: the row has one.
  std::vector<std::vector<double>> speed;
  std::vector<std::vector<char>> fitted;

  std::size_t rows() const { return valid.size(); }
  std::size_t columns() const { return track_ids.size(); }
  // The column of a track ID; columns() when there is none.
  std::size_t column(const std::string & track_id) const;
};

// half_window_rows: the speed fit's half window, which decides on which rows a speed is fitted
// (at least two valid rows within it).
RoadUserColumns road_user_columns(const ClosedLoopEpisode & episode, std::size_t half_window_rows);

// The executed ego's box on each executed row.
std::vector<BoxCorners> ego_boxes(
  const ClosedLoopEpisode & episode,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

// The box of an ego pose.
BoxCorners ego_box(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

// The signed clearance between two boxes: the closest-point distance when apart, minus the
// separating-axis penetration depth when they overlap.
double box_clearance(const BoxCorners & first, const BoxCorners & second);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__ROAD_USERS_HPP_

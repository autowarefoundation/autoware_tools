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

#ifndef CLOSED_LOOP__CONFLICT_HPP_
#define CLOSED_LOOP__CONFLICT_HPP_

#include "closed_loop/lane_layout.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// The raster and crossing parameters of the reviewed yielding metric.
struct CrossingGrid
{
  // Side of the square ground cells.
  double cell_m{0.1};
  // Largest spacing of the points sampled inside a box (at most cell_m).
  double sample_m{0.05};
  // Smallest heading difference of a crossing cell.
  double crossing_angle_deg{30.0};
  // Smallest number of cells that makes a conflict zone.
  int min_cells{10};
};

// Who passed the shared ground first.
enum class CrossingOrder { NONE = 0, NO_CROSSING = 1, ROAD_USER_FIRST = 2, EGO_FIRST = 3 };

std::string to_string(CrossingOrder order);

// How one road user shared ground with the ego (rows are the episode's).
struct Crossing
{
  CrossingOrder order{CrossingOrder::NONE};
  std::int64_t shared_cells{0};
  std::int64_t zone_cells{0};
  // The earlier party's last and the later party's first row in the cell that decides the PET,
  // centred at pet_cell; -1 unless ordered.
  std::int64_t exit_row{-1};
  std::int64_t entry_row{-1};
  Point2d pet_cell{0.0, 0.0};
  std::array<std::int64_t, 2> ego_zone_rows{-1, -1};
  std::array<std::int64_t, 2> road_user_zone_rows{-1, -1};

  bool ordered() const
  {
    return order == CrossingOrder::ROAD_USER_FIRST || order == CrossingOrder::EGO_FIRST;
  }
  // The PET in rows; -1 unless ordered.
  std::int64_t pet_rows() const { return ordered() ? entry_row - exit_row : -1; }
  // Rows from the ego's to the road user's first row in the zone; -1 unless the ego went first.
  std::int64_t entry_gap_rows() const
  {
    return order == CrossingOrder::EGO_FIRST ? road_user_zone_rows[0] - ego_zone_rows[0] : -1;
  }
};

// Measures each road user (a column of road_corners / road_valid / road_yaw, [row][column])
// against the ego (one box and yaw per row, every row valid) on square cells around origin; see
// conflict.cpp for the rules.
std::vector<Crossing> measure_crossings(
  const std::vector<BoxCorners> & ego_corners, const std::vector<double> & ego_yaw,
  const std::vector<std::vector<BoxCorners>> & road_corners,
  const std::vector<std::vector<char>> & road_valid,
  const std::vector<std::vector<double>> & road_yaw, const Point2d & origin,
  const CrossingGrid & grid = {});

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CONFLICT_HPP_

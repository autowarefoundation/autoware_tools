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

// Who passes a shared spot first, the post-encroachment time (PET) and the entry gap, computed as
// the reviewed yielding metric does, so that the reviewed PETs stay as they were. Two boxes moving
// over time cross where their footprints cover the same ground at a clear angle; the ground is cut
// into square cells, every box is sampled on a grid of points and marks the cells they fall in,
// and each party keeps its first and last row in each cell.

#include "closed_loop/conflict.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

using Cell = std::pair<std::int64_t, std::int64_t>;

struct Steps
{
  std::int64_t along{1};
  std::int64_t across{1};
};

struct Span
{
  std::int64_t first{std::numeric_limits<std::int64_t>::max()};
  std::int64_t last{-1};
};

// Samples per side of a party's boxes, from its longest sides.
Steps steps_for(const std::vector<const BoxCorners *> & boxes, const double sample_m)
{
  const auto length = [](const Point2d & from, const Point2d & to) {
    const double x = to.x() - from.x();
    const double y = to.y() - from.y();
    return std::sqrt(x * x + y * y);
  };
  double along = 0.0;
  double across = 0.0;
  for (const auto * box : boxes) {
    along = std::max(along, length((*box)[2], (*box)[3]));
    across = std::max(across, length((*box)[2], (*box)[1]));
  }
  return {
    static_cast<std::int64_t>(std::ceil(along / sample_m)) + 1,
    static_cast<std::int64_t>(std::ceil(across / sample_m)) + 1};
}

// numpy.linspace(0, 1, n)[k]: k times the step, the last sample exactly 1.
double fraction(const std::int64_t k, const std::int64_t n)
{
  if (k == n - 1 && n > 1) {
    return 1.0;
  }
  const double step = 1.0 / static_cast<double>(std::max<std::int64_t>(n - 1, 1));
  return static_cast<double>(k) * step;
}

// Every sample of a box, from its rear-right corner, as the cell it falls in.
template <typename Visit>
void sample_cells(
  const BoxCorners & box, const Point2d & origin, const Steps & steps, const double cell_m,
  Visit && visit)
{
  const Point2d & rear_right = box[2];
  const double along_x = box[3].x() - rear_right.x();
  const double along_y = box[3].y() - rear_right.y();
  const double across_x = box[1].x() - rear_right.x();
  const double across_y = box[1].y() - rear_right.y();
  for (std::int64_t i = 0; i < steps.along; ++i) {
    const double u = fraction(i, steps.along);
    for (std::int64_t j = 0; j < steps.across; ++j) {
      const double w = fraction(j, steps.across);
      const double x = rear_right.x() + u * along_x + w * across_x;
      const double y = rear_right.y() + u * along_y + w * across_y;
      visit(
        Cell{
          static_cast<std::int64_t>(std::floor((x - origin.x()) / cell_m)),
          static_cast<std::int64_t>(std::floor((y - origin.y()) / cell_m))});
    }
  }
}

// torch.remainder(angle + pi, 2 pi) - pi: [-pi, pi).
double wrap_angle(const double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

}  // namespace

std::string to_string(const CrossingOrder order)
{
  switch (order) {
    case CrossingOrder::NO_CROSSING:
      return "no_crossing";
    case CrossingOrder::ROAD_USER_FIRST:
      return "road_user_first";
    case CrossingOrder::EGO_FIRST:
      return "ego_first";
    case CrossingOrder::NONE:
    default:
      return "none";
  }
}

std::vector<Crossing> measure_crossings(
  const std::vector<BoxCorners> & ego_corners, const std::vector<double> & ego_yaw,
  const std::vector<std::vector<BoxCorners>> & road_corners,
  const std::vector<std::vector<char>> & road_valid,
  const std::vector<std::vector<double>> & road_yaw, const Point2d & origin,
  const CrossingGrid & grid)
{
  const auto rows = ego_corners.size();
  const auto columns = road_valid.empty() ? 0U : road_valid.front().size();
  std::vector<Crossing> crossings(columns);
  if (rows == 0U) {
    return crossings;
  }
  const double cell_m = grid.cell_m;

  // The ego, rasterized once: every cell it covers, its first and last row there.
  std::vector<const BoxCorners *> ego_boxes;
  for (const auto & box : ego_corners) {
    ego_boxes.push_back(&box);
  }
  const auto ego_steps = steps_for(ego_boxes, grid.sample_m);
  std::map<Cell, Span> ego_cells;
  for (std::size_t row = 0; row < rows; ++row) {
    sample_cells(ego_corners.at(row), origin, ego_steps, cell_m, [&](const Cell & cell) {
      auto & span = ego_cells[cell];
      span.first = std::min(span.first, static_cast<std::int64_t>(row));
      span.last = std::max(span.last, static_cast<std::int64_t>(row));
    });
  }
  if (ego_cells.empty()) {
    return crossings;
  }
  std::int64_t x_lo = std::numeric_limits<std::int64_t>::max();
  std::int64_t x_hi = std::numeric_limits<std::int64_t>::min();
  std::int64_t y_lo = std::numeric_limits<std::int64_t>::max();
  std::int64_t y_hi = std::numeric_limits<std::int64_t>::min();
  for (const auto & [cell, _] : ego_cells) {
    x_lo = std::min(x_lo, cell.first);
    x_hi = std::max(x_hi, cell.first);
    y_lo = std::min(y_lo, cell.second);
    y_hi = std::max(y_hi, cell.second);
  }

  for (std::size_t column = 0; column < columns; ++column) {
    std::vector<const BoxCorners *> boxes;
    for (std::size_t row = 0; row < rows && row < road_valid.size(); ++row) {
      if (road_valid.at(row).at(column) != 0) {
        boxes.push_back(&road_corners.at(row).at(column));
      }
    }
    if (boxes.empty()) {
      continue;
    }
    const auto steps = steps_for(boxes, grid.sample_m);
    // The road user's first and last row in each cell the ego covered.
    std::map<Cell, Span> shared;
    for (std::size_t row = 0; row < rows && row < road_valid.size(); ++row) {
      if (road_valid.at(row).at(column) == 0) {
        continue;
      }
      const auto & box = road_corners.at(row).at(column);
      double min_x = box[0].x();
      double max_x = box[0].x();
      double min_y = box[0].y();
      double max_y = box[0].y();
      for (const auto & corner : box) {
        min_x = std::min(min_x, corner.x());
        max_x = std::max(max_x, corner.x());
        min_y = std::min(min_y, corner.y());
        max_y = std::max(max_y, corner.y());
      }
      const std::int64_t lo_x =
        static_cast<std::int64_t>(std::floor((min_x - origin.x()) / cell_m)) - 1;
      const std::int64_t hi_x =
        static_cast<std::int64_t>(std::floor((max_x - origin.x()) / cell_m)) + 1;
      const std::int64_t lo_y =
        static_cast<std::int64_t>(std::floor((min_y - origin.y()) / cell_m)) - 1;
      const std::int64_t hi_y =
        static_cast<std::int64_t>(std::floor((max_y - origin.y()) / cell_m)) + 1;
      if (lo_x > x_hi || hi_x < x_lo || lo_y > y_hi || hi_y < y_lo) {
        continue;
      }
      sample_cells(box, origin, steps, cell_m, [&](const Cell & cell) {
        if (
          cell.first < x_lo || cell.first > x_hi || cell.second < y_lo || cell.second > y_hi ||
          ego_cells.count(cell) == 0U) {
          return;
        }
        auto & span = shared[cell];
        span.first = std::min(span.first, static_cast<std::int64_t>(row));
        span.last = std::max(span.last, static_cast<std::int64_t>(row));
      });
    }

    auto & crossing = crossings.at(column);
    crossing.shared_cells = static_cast<std::int64_t>(shared.size());
    if (shared.empty()) {
      continue;
    }
    // Per shared cell: does it cross (the road user's heading when it last left the cell against
    // the ego's when it first entered it), and who left it before the other came.
    struct Judged
    {
      Cell cell;
      Span road;
      Span ego;
      bool road_first;
      bool ego_first;
    };
    std::vector<Judged> cells;
    std::int64_t road_count = 0;
    std::int64_t ego_count = 0;
    for (const auto & [cell, road] : shared) {
      const auto & ego = ego_cells.at(cell);
      const double difference = std::abs(wrap_angle(
        road_yaw.at(static_cast<std::size_t>(road.last)).at(column) -
        ego_yaw.at(static_cast<std::size_t>(ego.first))));
      const bool crosses = difference * 180.0 / M_PI > grid.crossing_angle_deg;
      Judged judged{
        cell, road, ego, crosses && road.last < ego.first, crosses && ego.last < road.first};
      road_count += judged.road_first ? 1 : 0;
      ego_count += judged.ego_first ? 1 : 0;
      cells.push_back(judged);
    }
    crossing.order = CrossingOrder::NO_CROSSING;
    if (ego_count >= grid.min_cells) {
      crossing.order = CrossingOrder::EGO_FIRST;
    }
    if (road_count >= grid.min_cells) {
      crossing.order = CrossingOrder::ROAD_USER_FIRST;
    }
    if (!crossing.ordered()) {
      continue;
    }
    const bool road_led = crossing.order == CrossingOrder::ROAD_USER_FIRST;
    // The PET cell: the smallest gap, then the smallest cell (x first; cells are in that order).
    std::int64_t best_gap = std::numeric_limits<std::int64_t>::max();
    const Judged * decides = nullptr;
    Span ego_zone;
    Span road_zone;
    for (const auto & judged : cells) {
      const bool zone = road_led ? judged.road_first : judged.ego_first;
      if (!zone) {
        continue;
      }
      ++crossing.zone_cells;
      const std::int64_t earlier = road_led ? judged.road.last : judged.ego.last;
      const std::int64_t later = road_led ? judged.ego.first : judged.road.first;
      if (later - earlier < best_gap) {
        best_gap = later - earlier;
        decides = &judged;
      }
      ego_zone.first = std::min(ego_zone.first, judged.ego.first);
      ego_zone.last = std::max(ego_zone.last, judged.ego.last);
      road_zone.first = std::min(road_zone.first, judged.road.first);
      road_zone.last = std::max(road_zone.last, judged.road.last);
    }
    crossing.exit_row = road_led ? decides->road.last : decides->ego.last;
    crossing.entry_row = road_led ? decides->ego.first : decides->road.first;
    crossing.pet_cell = Point2d{
      origin.x() + (static_cast<double>(decides->cell.first) + 0.5) * cell_m,
      origin.y() + (static_cast<double>(decides->cell.second) + 0.5) * cell_m};
    crossing.ego_zone_rows = {ego_zone.first, ego_zone.last};
    crossing.road_user_zone_rows = {road_zone.first, road_zone.last};
  }
  return crossings;
}

}  // namespace autoware::planning_data_analyzer::closed_loop

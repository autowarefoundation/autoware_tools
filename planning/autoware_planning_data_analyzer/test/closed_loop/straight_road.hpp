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

#ifndef CLOSED_LOOP__STRAIGHT_ROAD_HPP_
#define CLOSED_LOOP__STRAIGHT_ROAD_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop::test
{

// A straight road along +x: two lanes the same way (right: y -1.75..1.75, left: 1.75..5.25), each
// as two lanelets split at x = 50, and an oncoming lane (5.25..8.75) running -x.
class StraightRoad
{
public:
  StraightRoad()
  {
    const auto y0 = row(-1.75);
    const auto y1 = row(1.75);
    const auto y2 = row(5.25);
    const auto y3 = row(8.75);
    // A lanelet's left bound is on its left looking its way.
    add(1, line(y1, 0, 1), line(y0, 0, 1));
    add(2, line(y1, 1, 2), line(y0, 1, 2));
    add(3, line(y2, 0, 1), line(y1, 0, 1));
    add(4, line(y2, 1, 2), line(y1, 1, 2));
    add(5, line(y2, 2, 0), line(y3, 2, 0));
  }

  lanelet::LaneletMapConstPtr map() const { return map_; }

private:
  // Points at x = 0, 50 and 100 on the line y.
  std::vector<lanelet::Point3d> row(const double y)
  {
    std::vector<lanelet::Point3d> points;
    for (const double x : {0.0, 50.0, 100.0}) {
      points.emplace_back(next_id_++, x, y, 0.0);
    }
    return points;
  }

  // The line from point `from` to point `to` of a row (both ways along it).
  lanelet::LineString3d line(const std::vector<lanelet::Point3d> & points, int from, const int to)
  {
    std::vector<lanelet::Point3d> part;
    const int step = to > from ? 1 : -1;
    for (; from != to + step; from += step) {
      part.push_back(points.at(static_cast<std::size_t>(from)));
    }
    return lanelet::LineString3d(next_id_++, part);
  }

  void add(
    const lanelet::Id id, const lanelet::LineString3d & left, const lanelet::LineString3d & right)
  {
    lanelet::Lanelet lanelet(id, left, right);
    lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    map_->add(lanelet);
  }

  lanelet::Id next_id_{100};
  lanelet::LaneletMapPtr map_{std::make_shared<lanelet::LaneletMap>()};
};

}  // namespace autoware::planning_data_analyzer::closed_loop::test

#endif  // CLOSED_LOOP__STRAIGHT_ROAD_HPP_

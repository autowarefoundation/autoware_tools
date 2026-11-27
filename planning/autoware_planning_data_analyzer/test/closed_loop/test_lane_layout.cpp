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

#include "closed_loop/lane_layout.hpp"
#include "straight_road.hpp"

#include <boost/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

using test::StraightRoad;

BoxCorners car(const double x, const double y, const double yaw = 0.0)
{
  return box_corners(x, y, yaw, 3.7, 1.0, 0.9, 0.9);
}

LineString2d straight_path(const double y, const double from_x = 0.0, const double to_x = 100.0)
{
  LineString2d path;
  for (double x = from_x; x <= to_x + 1.0e-9; x += 5.0) {
    path.emplace_back(x, y);
  }
  return path;
}

}  // namespace

TEST(LaneLayout, KeepsTheRoadLanesAndLinksConsecutiveLanelets)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());

  EXPECT_EQ(layout.lane_count(), 5U);
  // Lanelet 1 with its successor 2 is one chain; 5 has neither.
  ASSERT_EQ(layout.chains(1).size(), 1U);
  EXPECT_NEAR(boost::geometry::area(layout.chains(1).front()), 100.0 * 3.5, 1.0e-6);
  EXPECT_NEAR(boost::geometry::area(layout.chains(5).front()), 100.0 * 3.5, 1.0e-6);
}

TEST(LaneLayout, ASeamBetweenConsecutiveLaneletsIsNotStraddling)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());

  EXPECT_FALSE(layout.flags(car(10.0, 0.0)).straddles);
  EXPECT_FALSE(layout.flags(car(10.0, 0.0)).off_road);
  // Across x = 50, in one lane.
  EXPECT_FALSE(layout.flags(car(49.0, 0.0)).straddles);
  // Across the line between the two lanes.
  EXPECT_TRUE(layout.flags(car(30.0, 1.75)).straddles);
  // Over the right edge of the road.
  EXPECT_TRUE(layout.flags(car(30.0, -1.5)).off_road);
}

TEST(LaneLayout, OnRoadAndSameWay)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());

  EXPECT_TRUE(layout.on_road(car(30.0, 0.0)));
  EXPECT_FALSE(layout.on_road(car(30.0, 20.0)));
  EXPECT_TRUE(layout.same_way(car(30.0, 0.0), 0.0, 0.1));
  EXPECT_FALSE(layout.same_way(car(30.0, 0.0), M_PI, 0.1));
  // The oncoming lane runs -x.
  EXPECT_TRUE(layout.same_way(car(30.0, 7.0, M_PI), M_PI, 0.1));
  EXPECT_FALSE(layout.same_way(car(30.0, 7.0), 0.0, 0.1));
  // Across two lanes.
  EXPECT_FALSE(layout.same_way(car(30.0, 1.75), 0.0, 0.1));
}

TEST(LaneLayout, TheStartLaneGrowsAlongThePath)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());
  StartLaneOptions options;

  const auto lane = layout.start_lane(car(10.0, 0.0), straight_path(0.0), options);

  ASSERT_TRUE(lane.has_value());
  EXPECT_TRUE(LaneLayout::inside(car(80.0, 0.0), *lane));
  EXPECT_FALSE(LaneLayout::inside(car(80.0, 3.5), *lane));
  EXPECT_NEAR(LaneLayout::share_inside(car(80.0, 1.75), *lane), 0.5 + 0.1 / 1.8, 1.0e-2);
}

TEST(LaneLayout, NoStartLaneHoldsABoxAcrossTwoLanes)
{
  const StraightRoad road;
  const LaneLayout layout(road.map());

  EXPECT_FALSE(
    layout.start_lane(car(10.0, 1.75), straight_path(1.75), StartLaneOptions{}).has_value());
  StartLaneOptions loose;
  loose.hold = false;
  loose.one_branch = true;
  EXPECT_TRUE(layout.start_lane(car(10.0, 1.6), straight_path(1.6), loose).has_value());
}

TEST(LaneLayout, DistanceToAPath)
{
  const auto path = straight_path(0.0, 0.0, 10.0);

  EXPECT_DOUBLE_EQ(distance_to_path(Point2d{5.0, 2.0}, path), 2.0);
  EXPECT_DOUBLE_EQ(distance_to_path(Point2d{13.0, 4.0}, path), 5.0);
}

}  // namespace autoware::planning_data_analyzer::closed_loop

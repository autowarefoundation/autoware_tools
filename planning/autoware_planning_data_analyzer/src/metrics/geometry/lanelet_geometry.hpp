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

#ifndef METRICS__GEOMETRY__LANELET_GEOMETRY_HPP_
#define METRICS__GEOMETRY__LANELET_GEOMETRY_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids);

void append_unique_polygon(
  const lanelet::ConstPolygon3d & polygon, std::vector<lanelet::ConstPolygon3d> & polygons,
  std::unordered_set<lanelet::Id> & seen_ids);

void append_unique_line_string(
  const lanelet::ConstLineString3d & line_string,
  std::vector<lanelet::ConstLineString3d> & line_strings,
  std::unordered_set<lanelet::Id> & seen_ids);

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon);

bool point_in_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet);

bool point_in_polygon(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & polygon);

lanelet::BoundingBox2d make_bounding_box(
  double min_x, double min_y, double max_x, double max_y, double margin_m);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__GEOMETRY__LANELET_GEOMETRY_HPP_

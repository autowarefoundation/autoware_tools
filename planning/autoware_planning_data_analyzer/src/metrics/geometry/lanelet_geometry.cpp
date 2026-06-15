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

#include "lanelet_geometry.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/utility/Utilities.h>

#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
  }
}

void append_unique_polygon(
  const lanelet::ConstPolygon3d & polygon, std::vector<lanelet::ConstPolygon3d> & polygons,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(polygon.id()).second) {
    polygons.push_back(polygon);
  }
}

void append_unique_line_string(
  const lanelet::ConstLineString3d & line_string,
  std::vector<lanelet::ConstLineString3d> & line_strings,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(line_string.id()).second) {
    line_strings.push_back(line_string);
  }
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  namespace bg = boost::geometry;

  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back({point.x(), point.y()});
  }
  bg::correct(converted);
  return converted;
}

bool point_in_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

bool point_in_polygon(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & polygon)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
}

lanelet::BoundingBox2d make_bounding_box(
  const double min_x, const double min_y, const double max_x, const double max_y,
  const double margin_m)
{
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{min_x - margin_m, min_y - margin_m},
    lanelet::BasicPoint2d{max_x + margin_m, max_y + margin_m}};
}

}  // namespace autoware::planning_data_analyzer::metrics

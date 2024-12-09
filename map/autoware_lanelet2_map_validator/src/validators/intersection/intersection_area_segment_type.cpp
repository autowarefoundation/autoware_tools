// Copyright 2024 Autoware Foundation
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

#include "lanelet2_map_validator/validators/intersection/intersection_area_segment_type.hpp"

#include "lanelet2_map_validator/utils.hpp"

#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <limits>

namespace lanelet::autoware::validation
{
namespace
{
lanelet::validation::RegisterMapValidator<IntersectionAreaSegmentTypeValidator> reg;
}

lanelet::validation::Issues IntersectionAreaSegmentTypeValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(issues, check_intersection_area_segment_type(map));

  return issues;
}

lanelet::validation::Issues
IntersectionAreaSegmentTypeValidator::check_intersection_area_segment_type(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  for (const lanelet::ConstPolygon3d & polygon3d : map.polygonLayer) {
    const auto [nearby_lanelets, nearby_road_borders] = extract_nearby_elements(map, polygon3d);
  }

  return issues;
}

std::pair<lanelet::ConstLanelets, lanelet::ConstLineStrings3d>
IntersectionAreaSegmentTypeValidator::extract_nearby_elements(
  const lanelet::LaneletMap & map, const lanelet::ConstPolygon3d & intersection_area)
{
  lanelet::BoundingBox2d bbox2d = get_circumscribed_box_from_polygon(intersection_area);

  lanelet::ConstLanelets nearby_lanelets = map.laneletLayer.search(bbox2d);
  lanelet::ConstLineStrings3d nearby_linestrings = map.lineStringLayer.search(bbox2d);

  lanelet::ConstLineStrings3d nearby_road_borders;
  for (const auto linestring : nearby_linestrings) {
    if (
      linestring.hasAttribute(lanelet::AttributeName::Type) &&
      linestring.attribute(lanelet::AttributeName::Type).value() ==
        lanelet::AttributeValueString::RoadBorder) {
      nearby_road_borders.push_back(linestring);
    }
  }

  return std::make_pair(nearby_lanelets, nearby_road_borders);
}

lanelet::BoundingBox2d IntersectionAreaSegmentTypeValidator::get_circumscribed_box_from_polygon(
  const lanelet::ConstPolygon3d & polygon)
{
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();

  for (const lanelet::ConstPoint3d & point : polygon) {
    const lanelet::BasicPoint2d & coord = point.basicPoint2d();
    minX = std::min(minX, coord.x());
    minY = std::min(minY, coord.y());
    maxX = std::max(maxX, coord.x());
    maxY = std::max(maxY, coord.y());
  }

  return lanelet::BoundingBox2d(
    lanelet::BasicPoint2d(minX, minY), lanelet::BasicPoint2d(maxX, maxY));
}

}  // namespace lanelet::autoware::validation

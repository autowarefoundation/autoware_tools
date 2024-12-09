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
    if (
      !polygon3d.hasAttribute(lanelet::AttributeName::Type) ||
      polygon3d.attribute(lanelet::AttributeName::Type).value() != "intersection_area") {
      continue;
    }

    const auto borders_submap = create_nearby_borders_submap(map, polygon3d);
    lanelet::Ids invalid_point_ids = {};
    for (const lanelet::ConstPoint3d & point : polygon3d) {
      lanelet::LineStrings3d search_results = borders_submap->lineStringLayer.findUsages(point);
      if (search_results.empty()) {
        invalid_point_ids.push_back(point.id());
      }
    }
    if (!invalid_point_ids.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Polygon,
        polygon3d.id(),
        append_issue_code_prefix(
          this->name(), 1,
          "This intersection area is not made by points from road_border linestrings or lanelet "
          "edges. (Point ID: " +
            ids_to_string(invalid_point_ids) + ")"));
    }
  }

  return issues;
}

lanelet::LaneletSubmapUPtr IntersectionAreaSegmentTypeValidator::create_nearby_borders_submap(
  const lanelet::LaneletMap & map, const lanelet::ConstPolygon3d & intersection_area)
{
  lanelet::BoundingBox2d bbox2d = get_circumscribed_box_from_polygon(intersection_area);

  lanelet::ConstLanelets nearby_lanelets = map.laneletLayer.search(bbox2d);
  lanelet::ConstLineStrings3d nearby_linestrings = map.lineStringLayer.search(bbox2d);

  lanelet::LineStrings3d nearby_borders;

  // Collect lanelet edges intersecting the intersection area
  for (const auto & lanelet : nearby_lanelets) {
    if (
      !lanelet.hasAttribute(lanelet::AttributeName::Subtype) ||
      lanelet.attribute(lanelet::AttributeName::Subtype).value() !=
        lanelet::AttributeValueString::Road) {
      continue;
    }

    if (bbox2d.contains(lanelet.leftBound2d().front().basicPoint2d())) {
      lanelet::Point3d left_point(lanelet.leftBound().front());
      lanelet::Point3d right_point(lanelet.rightBound().front());

      lanelet::AttributeMap attribute;
      attribute["type"] = "lanelet_edge";  // An instant linestring type for this code

      lanelet::LineString3d lanelet_front_edge(
        lanelet::utils::getId(), {left_point, right_point}, attribute);
      nearby_borders.push_back(lanelet_front_edge);
    }

    if (bbox2d.contains(lanelet.leftBound2d().back().basicPoint2d())) {
      lanelet::Point3d left_point(lanelet.leftBound().back());
      lanelet::Point3d right_point(lanelet.rightBound().back());

      lanelet::AttributeMap attribute;
      attribute["type"] = "lanelet_edge";  // An instant linestring type for this code

      lanelet::LineString3d lanelet_back_edge(
        lanelet::utils::getId(), {left_point, right_point}, attribute);
      nearby_borders.push_back(lanelet_back_edge);
    }
  }

  // Collect road_border subtype linestrings
  for (const auto & linestring : nearby_linestrings) {
    if (
      linestring.hasAttribute(lanelet::AttributeName::Type) &&
      linestring.attribute(lanelet::AttributeName::Type).value() ==
        lanelet::AttributeValueString::RoadBorder) {
      auto data = std::const_pointer_cast<LineStringData>(linestring.constData());
      nearby_borders.push_back(lanelet::LineString3d(data, linestring.inverted()));
    }
  }

  return lanelet::utils::createSubmap(nearby_borders);
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

std::string IntersectionAreaSegmentTypeValidator::ids_to_string(const lanelet::Ids ids)
{
  std::string result = "(";
  for (size_t i = 0; i < ids.size(); i++) {
    result += std::to_string(ids[i]);
    if (i < ids.size() - 1) {
      result += ", ";
    }
  }
  result += ")";
  return result;
}

}  // namespace lanelet::autoware::validation

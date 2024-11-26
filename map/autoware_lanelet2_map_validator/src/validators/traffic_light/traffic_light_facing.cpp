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

#include "lanelet2_map_validator/validators/traffic_light/traffic_light_facing.hpp"

#include "lanelet2_map_validator/utils.hpp"

#include <Eigen/Core>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace lanelet::autoware::validation
{
namespace
{
lanelet::validation::RegisterMapValidator<TrafficLightFacingValidator> reg;
}  // namespace

lanelet::validation::Issues TrafficLightFacingValidator::operator()(const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Remove this line and write down how to append issues
  lanelet::autoware::validation::appendIssues(issues, check_traffic_light_facing(map));

  return issues;
}

lanelet::validation::Issues TrafficLightFacingValidator::check_traffic_light_facing(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Get all red_yellow_green traffic lights with a std::map of flags
  std::map<lanelet::Id, bool> tl_has_been_judged_as_correct;
  std::map<lanelet::Id, bool> tl_has_been_judged_as_wrong;

  for (const lanelet::ConstLineString3d & linestring : map.lineStringLayer) {
    if (is_red_yellow_green_traffic_light(linestring)) {
      tl_has_been_judged_as_correct.insert({linestring.id(), false});
      tl_has_been_judged_as_wrong.insert({linestring.id(), false});
    }
  }

  // Main validation procedure
  for (const lanelet::RegulatoryElementConstPtr & reg_elem : map.regulatoryElementLayer) {
    // Skip non traffic light regulatory elements
    if (
      reg_elem->attribute(lanelet::AttributeName::Subtype).value() !=
      lanelet::AttributeValueString::TrafficLight) {
      continue;
    }

    lanelet::ConstLineString3d stop_line = get_stop_line_from_reg_elem(reg_elem);

    for (const lanelet::ConstLineString3d & refers_linestring :
         reg_elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers)) {
      if (!is_red_yellow_green_traffic_light(refers_linestring)) {
        continue;
      }

      const lanelet::ConstLanelets referring_lanelets =
        collect_referring_lanelets(map, reg_elem->id());

      if (referring_lanelets.empty()) {
        continue;
      }

      // Assume the psuedo stop line from the first lanelet and check it is similar to the ones
      // of other lanelets
      lanelet::ConstLineString3d temp_pseudo_stop_line =
        get_starting_edge_from_lanelet(referring_lanelets[0], stop_line);

      Eigen::Vector3d pseudo_stop_line = linestring_to_vector3d(temp_pseudo_stop_line);

      for (size_t i = 1; i < referring_lanelets.size(); i++) {
        Eigen::Vector3d comparing_line =
          linestring_to_vector3d(get_starting_edge_from_lanelet(referring_lanelets[i], stop_line));
        double cosine_angle =
          pseudo_stop_line.dot(comparing_line) / (pseudo_stop_line.norm() * comparing_line.norm());
        if (cosine_angle < 0) {
          issues.emplace_back(
            lanelet::validation::Severity::Info, lanelet::validation::Primitive::LineString,
            refers_linestring.id(),
            issue_code_prefix(this->name(), 1) +
              "Lanelets referring this traffic_light have several divergent starting lines");
        }
      }

      // A traffic light is facing correct if the inner product is positve to the psuedo stop line
      Eigen::Vector3d traffic_light = linestring_to_vector3d(refers_linestring);
      double cosine_pseudo_stop_line_and_traffic_light =
        pseudo_stop_line.dot(traffic_light) / (pseudo_stop_line.norm() * traffic_light.norm());
      if (cosine_pseudo_stop_line_and_traffic_light > 0) {
        tl_has_been_judged_as_correct[refers_linestring.id()] = true;
      } else {
        tl_has_been_judged_as_wrong[refers_linestring.id()] = true;
      }
    }
  }

  // Digest the stop line non-existance and the traffic light facing error to issues
  for (const auto & entry : tl_has_been_judged_as_correct) {
    lanelet::Id id = entry.first;

    if (!tl_has_been_judged_as_correct[id] && tl_has_been_judged_as_wrong[id]) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString, id,
        issue_code_prefix(this->name(), 2) + "The linestring direction seems to be wrong.");
    } else if (tl_has_been_judged_as_correct[id] && tl_has_been_judged_as_wrong[id]) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::LineString, id,
        issue_code_prefix(this->name(), 3) +
          "The linestring direction has been judged as both correct and wrong.");
    }
  }

  return issues;
}

lanelet::ConstLineString3d TrafficLightFacingValidator::get_stop_line_from_reg_elem(
  const lanelet::RegulatoryElementConstPtr & reg_elem)
{
  // Assume that there is only one stop_line
  for (const lanelet::ConstLineString3d & ref_line :
       reg_elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine)) {
    if (
      ref_line.hasAttribute(lanelet::AttributeName::Type) &&
      ref_line.attribute(lanelet::AttributeName::Type).value() ==
        lanelet::AttributeValueString::StopLine) {
      return ref_line;
    }
  }

  // If there is not stop_line return an empty one.
  return lanelet::ConstLineString3d();
}

bool TrafficLightFacingValidator::is_red_yellow_green_traffic_light(
  const lanelet::ConstLineString3d & linestring)
{
  return linestring.hasAttribute(lanelet::AttributeName::Type) &&
         linestring.hasAttribute(lanelet::AttributeName::Subtype) &&
         linestring.attribute(lanelet::AttributeName::Type).value() ==
           lanelet::AttributeValueString::TrafficLight &&
         linestring.attribute(lanelet::AttributeName::Subtype).value() ==
           lanelet::AttributeValueString::RedYellowGreen;
}

lanelet::LineString3d TrafficLightFacingValidator::get_starting_edge_from_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::ConstLineString3d & reference)
{
  Eigen::Vector3d front_l = lanelet.leftBound().front().basicPoint();
  Eigen::Vector3d back_l = lanelet.leftBound().back().basicPoint();

  Eigen::Vector3d front_r = lanelet.rightBound().front().basicPoint();
  Eigen::Vector3d back_r = lanelet.rightBound().back().basicPoint();

  Eigen::Vector3d ref_1 = reference.front().basicPoint();
  Eigen::Vector3d ref_2 = reference.back().basicPoint();

  auto norm_sum = [ref_1, ref_2](const Eigen::Vector3d & vec1, const Eigen::Vector3d & vec2) {
    return (vec1 - ref_1).norm() + (vec2 - ref_2).norm();
  };

  double front_min = std::min(norm_sum(front_l, front_r), norm_sum(front_r, front_l));
  double back_min = std::min(norm_sum(back_l, back_r), norm_sum(back_r, back_l));

  Points3d points;
  if (front_min <= back_min) {
    points.push_back(Point3d(lanelet::InvalId, front_l.x(), front_l.y(), front_l.z()));
    points.push_back(Point3d(lanelet::InvalId, front_r.x(), front_r.y(), front_r.z()));
  } else {
    points.push_back(Point3d(lanelet::InvalId, back_l.x(), back_l.y(), back_l.z()));
    points.push_back(Point3d(lanelet::InvalId, back_r.x(), back_r.y(), back_r.z()));
  }

  return LineString3d(reference.id(), points);  // same id for debug reasons
}

lanelet::ConstLanelets TrafficLightFacingValidator::collect_referring_lanelets(
  const lanelet::LaneletMap & map, const lanelet::Id target_reg_elem_id)
{
  lanelet::ConstLanelets lanelet_group;
  for (const lanelet::ConstLanelet & lanelet : map.laneletLayer) {
    for (lanelet::RegulatoryElementConstPtr reg_elem : lanelet.regulatoryElements()) {
      if (reg_elem->id() == target_reg_elem_id) {
        lanelet_group.push_back(lanelet);
      }
    }
  }
  return lanelet_group;
}

Eigen::Vector3d TrafficLightFacingValidator::linestring_to_vector3d(
  const lanelet::ConstLineString3d linestring)
{
  Eigen::Vector3d front_point = linestring.front().basicPoint();
  Eigen::Vector3d back_point = linestring.back().basicPoint();

  return back_point - front_point;
}

}  // namespace lanelet::autoware::validation

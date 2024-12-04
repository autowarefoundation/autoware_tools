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

#include "autoware_lanelet2_extension/utility/query.hpp"
#include "lanelet2_map_validator/utils.hpp"

#include <Eigen/Core>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#define NOT_EXAMINED 0
#define FOUND_CORRECT 1
#define FOUND_WRONG 2
#define FOUND_AMBIGUOUS 3

struct CompareByRegElemId
{
  bool operator()(
    const lanelet::TrafficLightConstPtr & lhs, const lanelet::TrafficLightConstPtr & rhs) const
  {
    return lhs->id() < rhs->id();
  }
};

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

  // Collect lanelets that refers traffic_light subtype regulatory elements
  // and those regulatory elements
  lanelet::ConstLanelets traffic_light_referrers;
  std::set<lanelet::TrafficLightConstPtr, CompareByRegElemId> traffic_light_reg_elems;
  for (const lanelet::ConstLanelet & lane : map.laneletLayer) {
    std::vector<lanelet::TrafficLightConstPtr> lane_tl_reg_elems =
      lane.regulatoryElementsAs<lanelet::TrafficLight>();
    if (!lane_tl_reg_elems.empty()) {
      traffic_light_referrers.push_back(lane);
      for (const auto & tl_ptr : lane_tl_reg_elems) {
        traffic_light_reg_elems.insert(tl_ptr);
      }
    }
  }
  lanelet::LaneletSubmapConstUPtr referrers_submap =
    lanelet::utils::createConstSubmap(traffic_light_referrers, {});

  // Get all traffic lights with a std::map of status
  std::map<lanelet::Id, int> traffic_light_facing_status;

  // Main validation procedure
  for (const lanelet::TrafficLightConstPtr & reg_elem : traffic_light_reg_elems) {
    lanelet::Optional<lanelet::ConstLineString3d> stop_line = reg_elem->stopLine();
    if (!stop_line) {
      // This case should be filtered out by mapping.traffic_light.regulatory_element_details
      continue;
    }

    for (const lanelet::ConstLineString3d & refers_linestring :
         reg_elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers)) {
      if (!is_red_yellow_green_traffic_light(refers_linestring)) {
        continue;
      }

      traffic_light_facing_status.insert({refers_linestring.id(), NOT_EXAMINED});

      const lanelet::ConstLanelets referring_lanelets =
        referrers_submap->laneletLayer.findUsages(reg_elem);

      if (referring_lanelets.empty()) {
        // This case should be filtered out by mapping.traffic_light.missing_referrers
        continue;
      }

      // Estimate the pseudo stop line from the first lanelet and check whether it is similar to
      // those of other lanelets
      lanelet::ConstLineString3d temp_pseudo_stop_line =
        get_starting_edge_from_lanelet(referring_lanelets[0], *stop_line);

      Eigen::Vector3d pseudo_stop_line = linestring_to_vector3d(temp_pseudo_stop_line);

      for (size_t i = 1; i < referring_lanelets.size(); i++) {
        Eigen::Vector3d comparing_line =
          linestring_to_vector3d(get_starting_edge_from_lanelet(referring_lanelets[i], *stop_line));
        double cosine_angle =
          pseudo_stop_line.dot(comparing_line) / (pseudo_stop_line.norm() * comparing_line.norm());
        if (cosine_angle < 0.707) {  // about 45 deg
          issues.emplace_back(
            lanelet::validation::Severity::Info, lanelet::validation::Primitive::LineString,
            refers_linestring.id(),
            append_issue_code_prefix(
              this->name(), 1,
              "Lanelets referring this traffic_light have several divergent starting lines"));
        }
      }

      // A traffic light is facing correct if the inner product is positive to the pseudo stop line
      Eigen::Vector3d traffic_light = linestring_to_vector3d(refers_linestring);
      double cosine_pseudo_stop_line_and_traffic_light =
        pseudo_stop_line.dot(traffic_light) / (pseudo_stop_line.norm() * traffic_light.norm());
      if (cosine_pseudo_stop_line_and_traffic_light > 0) {
        traffic_light_facing_status[refers_linestring.id()] |= FOUND_CORRECT;
      } else {
        traffic_light_facing_status[refers_linestring.id()] |= FOUND_WRONG;
      }
    }
  }

  // Digest the stop line non-existence and the traffic light facing error to issues
  for (const auto & [id, status] : traffic_light_facing_status) {
    if (status == FOUND_WRONG) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString, id,
        append_issue_code_prefix(this->name(), 2, "The linestring direction seems to be wrong."));
    } else if (status == FOUND_AMBIGUOUS) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::LineString, id,
        append_issue_code_prefix(
          this->name(), 3, "The linestring direction has been judged as both correct and wrong."));
    }
  }

  return issues;
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

Eigen::Vector3d TrafficLightFacingValidator::linestring_to_vector3d(
  const lanelet::ConstLineString3d linestring)
{
  Eigen::Vector3d front_point = linestring.front().basicPoint();
  Eigen::Vector3d back_point = linestring.back().basicPoint();

  return back_point - front_point;
}

}  // namespace lanelet::autoware::validation

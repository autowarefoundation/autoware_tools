// Copyright 2023 Autoware Foundation
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

#include "autoware_lanelet2_map_validator/vals/regulatory_element_details.hpp"

namespace lanelet
{
namespace validation
{
namespace
{
lanelet::validation::RegisterMapValidator<RegulatoryElementDetailsChecker> reg;
}  // namespace

lanelet::validation::Issues RegulatoryElementDetailsChecker::operator()(
  const lanelet::LaneletMap & map)
{
  // All issues found by all validators
  lanelet::validation::Issues issues;

  // Append issues found by each validator
  lanelet::autoware::validation::appendIssues(issues, checkRegulatoryElementOfTrafficLight(map));
  lanelet::autoware::validation::appendIssues(issues, checkRegulatoryElementOfCrosswalk(map));
  return issues;
}

bool RegulatoryElementDetailsChecker::isPedestrianTrafficLight(
  const std::vector<lanelet::ConstLineString3d> & traffic_lights)
{
  for (const auto & tl : traffic_lights) {
    const auto & attrs = tl.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Subtype);
    if (it == attrs.end() || it->second != "red_green") {
      return false;
    }
  }
  return true;
}

lanelet::validation::Issues RegulatoryElementDetailsChecker::checkRegulatoryElementOfTrafficLight(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;
  // filter regulatory element whose Subtype is traffic light
  auto elems =
    map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
      const auto & attrs = elem->attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Subtype);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::TrafficLight;
    });

  for (const auto & elem : elems) {
    // Get line strings of traffic light referred by regulatory element
    auto refers = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
    // Get stop line referred by regulatory element
    auto ref_lines = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
    const auto & issue_tl = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString,
      lanelet::utils::getId(),
      "Refers of traffic light regulatory element must have type of traffic_light.");
    lanelet::autoware::validation::checkPrimitivesType(
      refers, lanelet::AttributeValueString::TrafficLight, issue_tl, issues);

    const auto & issue_sl = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString,
      lanelet::utils::getId(),
      "Refline of traffic light regulatory element must have type of stop_line.");
    lanelet::autoware::validation::checkPrimitivesType(
      ref_lines, lanelet::AttributeValueString::StopLine, issue_sl, issues);

    if (refers.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of traffic light must have a traffic light(refers).");
    }
    // TODO(sgk-000): Check correct behavior if regulatory element has two or more traffic light
    //  else if (refers.size() != 1) {
    //   issues.emplace_back(
    //     lanelet::validation::Severity::Error,
    //     lanelet::validation::Primitive::RegulatoryElement, elem->id(), "Regulatory element of
    //     traffic light must have only one traffic light(refers).");
    // }

    // Report error if regulatory element does not have stop line and this is not a pedestrian
    // traffic light
    if (ref_lines.empty() && !isPedestrianTrafficLight(refers)) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of traffic light must have a stop line(ref_line).");
    }
  }
  return issues;
}

lanelet::validation::Issues RegulatoryElementDetailsChecker::checkRegulatoryElementOfCrosswalk(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;
  // filter elem whose Subtype is crosswalk
  auto elems = map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
                 const auto & attrs = elem->attributes();
                 const auto & it = attrs.find(lanelet::AttributeName::Subtype);
                 return it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk;
               });

  for (const auto & elem : elems) {
    // Get lanelet of crosswalk referred by regulatory element
    auto refers = elem->getParameters<lanelet::ConstLanelet>(lanelet::RoleName::Refers);
    // Get stop line referred by regulatory element
    auto ref_lines = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
    // Get crosswalk polygon referred by regulatory element
    auto crosswalk_polygons = elem->getParameters<lanelet::ConstPolygon3d>(
      lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon);

    const auto & issue_cw = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::Lanelet,
      lanelet::utils::getId(),
      "Refers of crosswalk regulatory element must have type of crosswalk.");
    lanelet::autoware::validation::checkPrimitivesType(
      refers, lanelet::AttributeValueString::Lanelet, lanelet::AttributeValueString::Crosswalk,
      issue_cw, issues);

    const auto & issue_sl = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString,
      lanelet::utils::getId(),
      "Refline of crosswalk regulatory element must have type of stopline.");
    lanelet::autoware::validation::checkPrimitivesType(
      ref_lines, lanelet::AttributeValueString::StopLine, issue_sl, issues);

    const auto & issue_poly = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::Polygon,
      lanelet::utils::getId(),
      "Crosswalk polygon of crosswalk regulatory element must have type of Crosswalk_polygon.");
    lanelet::autoware::validation::checkPrimitivesType(
      crosswalk_polygons, lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon,
      issue_poly, issues);

    // Report warning if regulatory element does not have crosswalk polygon
    if (crosswalk_polygons.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk is nice to have crosswalk_polygon.");
    } else if (crosswalk_polygons.size() > 1) {  // Report error if regulatory element has two or
                                                 // more crosswalk polygon
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk must have only one crosswalk_polygon.");
    }
    // Report Info if regulatory element does not have stop line
    if (ref_lines.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Info, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk does not have stop line(ref_line).");
    }
    // Report error if regulatory element does not have lanelet of crosswalk
    if (refers.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk must have lanelet of crosswalk(refers).");
    } else if (refers.size() > 1) {  // Report error if regulatory element has two or more lanelet
                                     // of crosswalk
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(),
        "Regulatory element of cross walk must have only one lanelet of crosswalk(refers).");
    }
  }
  return issues;
}

}  // namespace validation
}  // namespace lanelet

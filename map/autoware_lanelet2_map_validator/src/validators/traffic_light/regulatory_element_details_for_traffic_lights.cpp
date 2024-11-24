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

#include "lanelet2_map_validator/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp"

#include "lanelet2_map_validator/utils.hpp"

#include <range/v3/view/filter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>

namespace lanelet::autoware::validation
{
namespace
{
lanelet::validation::RegisterMapValidator<RegulatoryElementsDetailsForTrafficLightsValidator> reg;
}  // namespace

lanelet::validation::Issues RegulatoryElementsDetailsForTrafficLightsValidator::operator()(
  const lanelet::LaneletMap & map)
{
  // All issues found by all validators
  lanelet::validation::Issues issues;

  // Append issues found by each validator
  lanelet::autoware::validation::appendIssues(issues, checkRegulatoryElementOfTrafficLights(map));
  return issues;
}

bool RegulatoryElementsDetailsForTrafficLightsValidator::isPedestrianTrafficLight(
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

lanelet::validation::Issues
RegulatoryElementsDetailsForTrafficLightsValidator::checkRegulatoryElementOfTrafficLights(
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
        elem->id(),
        issue_code_prefix(this->name(), 1) +
          "Regulatory element of traffic light must have a stop line(ref_line).");
    }

    const auto & issue_tl = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString,
      lanelet::utils::getId(),
      issue_code_prefix(this->name(), 2) +
        "Refers of traffic light regulatory element must have type of traffic_light.");
    lanelet::autoware::validation::checkPrimitivesType(
      refers, lanelet::AttributeValueString::TrafficLight, issue_tl, issues);

    const auto & issue_sl = lanelet::validation::Issue(
      lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString,
      lanelet::utils::getId(),
      issue_code_prefix(this->name(), 3) +
        "ref_line of traffic light regulatory element must have type of stop_line.");
    lanelet::autoware::validation::checkPrimitivesType(
      ref_lines, lanelet::AttributeValueString::StopLine, issue_sl, issues);
  }
  return issues;
}

}  // namespace lanelet::autoware::validation

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

#include "validators/traffic_light/missing_regulatory_elements_for_traffic_lights.hpp"

#include "lib/utils.hpp"

#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <set>

namespace lanelet
{
namespace validation
{
namespace
{
lanelet::validation::RegisterMapValidator<MissingRegulatoryElementsForTrafficLightsValidator> reg;
}

lanelet::validation::Issues MissingRegulatoryElementsForTrafficLightsValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(
    issues, checkMissingRegulatoryElementsForTrafficLights(map));

  return issues;
}

lanelet::validation::Issues
MissingRegulatoryElementsForTrafficLightsValidator::checkMissingRegulatoryElementsForTrafficLights(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Get all line strings whose type is traffic light
  auto tl_ids =
    map.lineStringLayer | ranges::views::filter([](auto && ls) {
      const auto & attrs = ls.attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Type);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::TrafficLight;
    }) |
    ranges::views::transform([](auto && ls) { return ls.id(); }) | ranges::views::unique;

  // Filter regulatory elements whose type is traffic light and has refers
  auto reg_elem_tl = map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
                       const auto & attrs = elem->attributes();
                       const auto & it = attrs.find(lanelet::AttributeName::Subtype);
                       const auto & params = elem->getParameters();
                       return it != attrs.end() &&
                              it->second == lanelet::AttributeValueString::TrafficLight &&
                              params.find(lanelet::RoleNameString::Refers) != params.end();
                     });

  // Get all line strings of traffic light referred by regulatory elements
  std::set<lanelet::Id> tl_ids_reg_elem;
  for (const auto & elem : reg_elem_tl) {
    const auto & refers =
      elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
    for (const auto & refer : refers) {
      tl_ids_reg_elem.insert(refer.id());
    }
  }

  // Check if all line strings of traffic light referred by regulatory elements
  for (const auto & tl_id : tl_ids) {
    if (tl_ids_reg_elem.find(tl_id) == tl_ids_reg_elem.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString, tl_id,
        "No regulatory element refers to this traffic light.");
    }
  }

  return issues;
}

}  // namespace validation
}  // namespace lanelet

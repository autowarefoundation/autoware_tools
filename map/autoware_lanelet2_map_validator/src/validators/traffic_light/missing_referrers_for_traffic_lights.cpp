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

#include "lanelet2_map_validator/validators/traffic_light/missing_referrers_for_traffic_lights.hpp"

#include "lanelet2_map_validator/utils.hpp"

#include <lanelet2_core/LaneletMap.h>

namespace lanelet::autoware::validation
{
namespace
{
lanelet::validation::RegisterMapValidator<MissingReferrersForTrafficLightsValidator> reg;
}

lanelet::validation::Issues MissingReferrersForTrafficLightsValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(
    issues, check_missing_referrers_for_traffic_lights(map));

  return issues;
}

lanelet::validation::Issues
MissingReferrersForTrafficLightsValidator::check_missing_referrers_for_traffic_lights(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  for (const lanelet::RegulatoryElementConstPtr & reg_elem : map.regulatoryElementLayer) {
    // Skip non traffic light regulatory elements
    if (
      reg_elem->attribute(lanelet::AttributeName::Subtype).value() !=
      lanelet::AttributeValueString::TrafficLight) {
      continue;
    }

    // At least one lanelet should refer a traffic_light regulatory element
    const lanelet::ConstLanelets referring_lanelets = map.laneletLayer.findUsages(reg_elem);

    if (referring_lanelets.size() == 0) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        reg_elem->id(),
        append_issue_code_prefix(
          this->name(), 1,
          "Regulatory element of traffic light must be referred by at least one lanelet."));
      continue;
    }
  }

  return issues;
}
}  // namespace lanelet::autoware::validation

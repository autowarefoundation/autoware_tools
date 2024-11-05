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

#include "lanelet2_map_validator/utils.hpp"
#include "lanelet2_map_validator/validators/crosswalk/missing_regulatory_elements_for_crosswalks.hpp"

#include <range/v3/view/filter.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <set>

namespace lanelet
{
namespace validation
{
namespace
{
lanelet::validation::RegisterMapValidator<MissingRegulatoryElementsForCrosswalksValidator> reg;
}

lanelet::validation::Issues MissingRegulatoryElementsForCrosswalksValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(
    issues, checkMissingRegulatoryElementsForCrosswalks(map));

  return issues;
}

lanelet::validation::Issues
MissingRegulatoryElementsForCrosswalksValidator::checkMissingRegulatoryElementsForCrosswalks(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Get all lanelets whose type is crosswalk
  std::set<lanelet::Id> cw_ids;

  for (const auto & ll : map.laneletLayer) {
    const auto & attrs = ll.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Subtype);
    // Check if this lanelet is crosswalk
    if (it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk) {
      cw_ids.insert(ll.id());
    }
  }

  // Filter regulatory elements whose type is crosswalk and has refers
  auto reg_elem_cw =
    map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
      const auto & attrs = elem->attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Subtype);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk;
    }) |
    ranges::views::filter([](auto && elem) {
      const auto & param = elem->getParameters();
      return param.find(lanelet::RoleNameString::Refers) != param.end();
    });

  // Get all lanelets of crosswalk referred by regulatory elements
  std::set<lanelet::Id> cw_ids_reg_elem;
  for (const auto & elem : reg_elem_cw) {
    const auto & refers = elem->getParameters<lanelet::ConstLanelet>(lanelet::RoleName::Refers);
    for (const lanelet::ConstLanelet & refer : refers) {
      cw_ids_reg_elem.insert(refer.id());
    }
  }

  // Check if all lanelets of crosswalk referred by regulatory elements
  for (const auto & cw_id : cw_ids) {
    if (cw_ids_reg_elem.find(cw_id) == cw_ids_reg_elem.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Lanelet, cw_id,
        "No regulatory element refers to this crosswalk.");
    }
  }

  return issues;
}
}  // namespace validation
}  // namespace lanelet

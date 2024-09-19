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

#include <autoware_lanelet2_map_validator/utils.hpp>
#include <autoware_lanelet2_map_validator/validators/stop_line/missing_regulatory_elements_for_stop_lines.hpp>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <set>

namespace lanelet
{
namespace validation
{
namespace 
{
  lanelet::validation::RegisterMapValidator<MissingRegulatoryElementsForStopLinesValidator> reg;
}

  lanelet::validation::Issues MissingRegulatoryElementsForStopLinesValidator::operator()(
    const lanelet::LaneletMap & map)
  {
    lanelet::validation::Issues issues;
    
    lanelet::autoware::validation::appendIssues(
      issues, checkMissingRegulatoryElementsForStopLines(map));
    
    return issues;
  }

  lanelet::validation::Issues
  MissingRegulatoryElementsForStopLinesValidator::checkMissingRegulatoryElementsForStopLines(
    const lanelet::LaneletMap & map)
  {
    lanelet::validation::Issues issues;

    // Get all line strings whose type is stop line
    auto sl_ids = map.lineStringLayer | ranges::views::filter([](auto && ls) {
                    const auto & attrs = ls.attributes();
                    const auto & it = attrs.find(lanelet::AttributeName::Type);
                    return it != attrs.end() && it->second == lanelet::AttributeValueString::StopLine;
                  }) |
                  ranges::views::transform([](auto && ls) { return ls.id(); }) |
                  ranges::views::unique;

    // Filter regulatory elements whose refline type is stop line
    auto reg_elem_sl = map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
                        const auto & params = elem->getParameters();
                        return params.find(lanelet::RoleNameString::RefLine) != params.end();
                      });

    // Get all line strings of stop line referred by regulatory elements
    std::set<lanelet::Id> sl_ids_reg_elem;
    for (const auto & elem : reg_elem_sl) {
      const auto & ref_lines =
        elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
      for (const auto & ref_line : ref_lines) {
        const auto & attrs = ref_line.attributes();
        const auto & it = attrs.find(lanelet::AttributeName::Type);
        if (it != attrs.end() && it->second == lanelet::AttributeValueString::StopLine) {
          sl_ids_reg_elem.insert(ref_line.id());
        }
      }
    }

    // Check if all line strings of stop line referred by regulatory elements
    for (const auto & sl_id : sl_ids) {
      if (sl_ids_reg_elem.find(sl_id) == sl_ids_reg_elem.end()) {
        issues.emplace_back(
          lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString, sl_id,
          "No regulatory element refers to this stop line.");
      }
    }

    return issues;
  }

} // namespace validation
} // namespace lanelet2
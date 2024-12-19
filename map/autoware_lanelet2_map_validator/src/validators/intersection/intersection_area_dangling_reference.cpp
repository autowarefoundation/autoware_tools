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

#include "lanelet2_map_validator/validators/intersection/intersection_area_dangling_reference.hpp"

#include "lanelet2_map_validator/utils.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace lanelet::autoware::validation
{

namespace
{
lanelet::validation::RegisterMapValidator<IntersectionAreaDanglingReferenceValidator> reg;
}  // namespace

lanelet::validation::Issues IntersectionAreaDanglingReferenceValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(
    issues, check_intersection_area_dangling_reference(map));

  return issues;
}

lanelet::validation::Issues
IntersectionAreaDanglingReferenceValidator::check_intersection_area_dangling_reference(
  const lanelet::LaneletMap & map)
{
  // returns the VALUE of intersection_area key
  auto is_intersection_with_area = [](const auto & lanelet) -> std::optional<lanelet::Id> {
    if (lanelet.attributeOr("turn_direction", "none") == std::string("none")) {
      return std::nullopt;
    }

    const std::string id_str = lanelet.attributeOr("intersection_area", "none");
    if (id_str == std::string("none")) {
      return std::nullopt;
    }

    const auto id = static_cast<lanelet::Id>(std::atoi(id_str.c_str()));
    return id;
  };

  std::vector<std::pair<lanelet::ConstLanelet, lanelet::Id>> intersection_with_area_lanelets;
  for (const auto & lanelet : map.laneletLayer) {
    if (const auto id_opt = is_intersection_with_area(lanelet); id_opt) {
      intersection_with_area_lanelets.emplace_back(lanelet, id_opt.value());
    }
  }

  std::unordered_set<lanelet::Id> intersection_area_ids;
  for (const auto & area : map.polygonLayer) {
    if (
      area.attributeOr(lanelet::AttributeName::Type, "none") == std::string("intersection_area")) {
      intersection_area_ids.emplace(area.id());
    }
  }

  lanelet::validation::Issues issues;
  for (const auto & [lanelet, intersection_area_id] : intersection_with_area_lanelets) {
    if (intersection_area_ids.find(intersection_area_id) == intersection_area_ids.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Lanelet, lanelet.id(),
        append_issue_code_prefix(
          this->name(), 1,
          "Lanelet of ID " + std::to_string(lanelet.id()) +
            " has dangling reference to non-existing intersection area of ID " +
            std::to_string(intersection_area_id)));
    }
  }

  return issues;
}

}  // namespace lanelet::autoware::validation

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

#include "lanelet2_map_validator/validators/lane/local_coordinates_declaration.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_map_validator/utils.hpp"

#include <set>

namespace lanelet::autoware::validation
{
namespace
{
lanelet::validation::RegisterMapValidator<LocalCoordinatesDeclarationValidator> reg;
}

lanelet::validation::Issues LocalCoordinatesDeclarationValidator::operator()(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  lanelet::autoware::validation::appendIssues(issues, check_local_coordinates_declaration(map));

  return issues;
}

lanelet::validation::Issues
LocalCoordinatesDeclarationValidator::check_local_coordinates_declaration(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  bool is_local_mode = false;

  std::set<lanelet::Id> non_local_point_ids;

  for (const lanelet::ConstPoint3d & point : map.pointLayer) {
    bool local_x = point.hasAttribute("local_x");
    bool local_y = point.hasAttribute("local_y");

    if (!is_local_mode) {
      if (local_x || local_y) {
        is_local_mode = true;
        for (const lanelet::Id & id : non_local_point_ids) {
          issues.emplace_back(
            lanelet::validation::Severity::Error, lanelet::validation::Primitive::Point, id,
            append_issue_code_prefix(
              this->name(), 1, "This point doesn't have local coordinates while others do."));
        }
      } else {
        non_local_point_ids.insert(point.id());
      }
      continue;
    }

    // Points are assumed to have local_x and local_y from here
    if (!local_x && !local_y) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Point, point.id(),
        append_issue_code_prefix(
          this->name(), 1, "This point doesn't have local coordinates while others do."));
    }

    // Only one coordinate (local_x or local_y) is defined
    if (local_x && !local_y) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Point, point.id(),
        append_issue_code_prefix(
          this->name(), 2, "\"local_x\" is declared but \"local_y\" is not."));
      continue;
    }

    if (!local_x && local_y) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Point, point.id(),
        append_issue_code_prefix(
          this->name(), 3, "\"local_y\" is declared but \"local_x\" is not."));
      continue;
    }
  }

  return issues;
}
}  // namespace lanelet::autoware::validation

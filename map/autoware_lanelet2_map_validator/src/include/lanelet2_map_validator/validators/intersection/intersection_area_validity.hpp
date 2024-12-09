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

#ifndef LANELET2_MAP_VALIDATOR__VALIDATORS__INTERSECTION__INTERSECTION_AREA_VALIDITY_HPP_  // NOLINT
#define LANELET2_MAP_VALIDATOR__VALIDATORS__INTERSECTION__INTERSECTION_AREA_VALIDITY_HPP_  // NOLINT

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

namespace lanelet::autoware::validation
{
class IntersectionAreaValidityValidator : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.intersection.intersection_area_validity"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues check_intersection_area_validity(const lanelet::LaneletMap & map);
  lanelet::BoundingBox2d get_circumscribed_box_from_polygon(
    const lanelet::ConstPolygon3d & polygon);
};
}  // namespace lanelet::autoware::validation

// clang-format off
#endif  // LANELET2_MAP_VALIDATOR__VALIDATORS__INTERSECTION__INTERSECTION_AREA_VALIDITY_HPP_  // NOLINT
// clang-format on

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

#ifndef LANELET2_MAP_VALIDATOR__VALIDATORS__CROSSWALK__REGULATORY_ELEMENT_DETAILS_FOR_CROSSWALKS_HPP_  // NOLINT
#define LANELET2_MAP_VALIDATOR__VALIDATORS__CROSSWALK__REGULATORY_ELEMENT_DETAILS_FOR_CROSSWALKS_HPP_  // NOLINT

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

namespace lanelet
{
namespace validation
{
class RegulatoryElementsDetailsForCrosswalksValidator : public lanelet::validation::MapValidator
{
public:
  // Write the validator's name here
  constexpr static const char * name() { return "mapping.crosswalk.regulatory_element_details"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues checkRegulatoryElementOfCrosswalks(const lanelet::LaneletMap & map);
};
}  // namespace validation
}  // namespace lanelet

// clang-format off
#endif  // LANELET2_MAP_VALIDATOR__VALIDATORS__CROSSWALK__REGULATORY_ELEMENT_DETAILS_FOR_CROSSWALKS_HPP_  // NOLINT
// clang-format on

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

#ifndef AUTOWARE_LANELET2_MAP_VALIDATOR__VALIDATORS__MISSING_REGULATORY_ELEMENTS_HPP_
#define AUTOWARE_LANELET2_MAP_VALIDATOR__VALIDATORS__MISSING_REGULATORY_ELEMENTS_HPP_

#include "autoware_lanelet2_map_validator/utils.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>

#include <set>

namespace lanelet
{
namespace validation
{

class MissingRegulatoryElementsChecker : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.missing_regulatory_elements"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues checkMissingReglatoryElementsInTrafficLight(
    const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkMissingReglatoryElementsInCrosswalk(
    const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkMissingReglatoryElementsInStopLine(
    const lanelet::LaneletMap & map);
  std::set<lanelet::Id> tl_elem_with_cw_;
};
}  // namespace validation
}  // namespace lanelet

#endif  // AUTOWARE_LANELET2_MAP_VALIDATOR__VALS__MISSING_REGULATORY_ELEMENTS_HPP_

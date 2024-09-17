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

#include "autoware_lanelet2_map_validator/utils.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <range/v3/view/filter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>

#include <set>
#include <vector>

namespace lanelet
{
namespace validation
{

#ifndef AUTOWARE_LANELET2_MAP_VALIDATOR__VALIDATORS__REGULATORY_ELEMENT_DETAILS_HPP_
#define AUTOWARE_LANELET2_MAP_VALIDATOR__VALIDATORS__REGULATORY_ELEMENT_DETAILS_HPP_

class RegulatoryElementDetailsChecker : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.regulatory_elements_details"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  bool isPedestrianTrafficLight(const std::vector<lanelet::ConstLineString3d> & traffic_lights);
  lanelet::validation::Issues checkRegulatoryElementOfTrafficLight(const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegulatoryElementOfCrosswalk(const lanelet::LaneletMap & map);
};
}  // namespace validation
}  // namespace lanelet

#endif  // AUTOWARE_LANELET2_MAP_VALIDATOR__VALS__REGULATORY_ELEMENT_DETAILS_HPP_

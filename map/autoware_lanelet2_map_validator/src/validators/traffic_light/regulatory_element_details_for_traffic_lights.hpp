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

<<<<<<< HEAD:map/autoware_lanelet2_map_validator/include/autoware_lanelet2_map_validator/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp
#ifndef VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
#define VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
=======
#ifndef VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
#define VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
>>>>>>> 0fe0b37 (Changed the entire structure.):map/autoware_lanelet2_map_validator/src/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

#include <vector>

namespace lanelet
{
namespace validation
{
class RegulatoryElementsDetailsForTrafficLightsValidator : public lanelet::validation::MapValidator
{
public:
  // Write the validator's name here
  constexpr static const char * name()
  {
    return "mapping.traffic_light.regulatory_element_details";
  }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  bool isPedestrianTrafficLight(const std::vector<lanelet::ConstLineString3d> & traffic_lights);
  lanelet::validation::Issues checkRegulatoryElementOfTrafficLights(
    const lanelet::LaneletMap & map);
};
}  // namespace validation
}  // namespace lanelet

<<<<<<< HEAD:map/autoware_lanelet2_map_validator/include/autoware_lanelet2_map_validator/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp
#endif  // AUTOWARE_LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
        // REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
=======
#endif  // VALIDATORS__TRAFFIC_LIGHT__REGULATORY_ELEMENT_DETAILS_FOR_TRAFFIC_LIGHTS_HPP_
>>>>>>> 0fe0b37 (Changed the entire structure.):map/autoware_lanelet2_map_validator/src/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp

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

#ifndef LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__MISSING_REFERRERS_FOR_TRAFFIC_LIGHTS_HPP_  // NOLINT
#define LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__MISSING_REFERRERS_FOR_TRAFFIC_LIGHTS_HPP_  // NOLINT

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

namespace lanelet::autoware::validation
{
class MissingReferrersForTrafficLightsValidator : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.traffic_light.missing_referrers"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  /**
   * @brief This is the main part of this validator.
   *
   * @param map Lanelet map to validate
   * @return lanelet::validation::Issues
   */
  lanelet::validation::Issues check_missing_referrers_for_traffic_lights(
    const lanelet::LaneletMap & map);

  /**
   * @brief Returns lanelets that refers the regulatory element specified by the input id
   * from the input lanelet map.
   *
   * @param map
   * @param reg_elem_id
   * @return lanelet::ConstLanelets
   */
  lanelet::ConstLanelets collect_referring_lanelets(
    const lanelet::LaneletMap & map, const lanelet::Id reg_elem_id);
};
}  // namespace lanelet::autoware::validation

// clang-format off
#endif  // LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__MISSING_REFERRERS_FOR_TRAFFIC_LIGHTS_HPP_  // NOLINT
// clang-format on

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

#ifndef LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__TRAFFIC_LIGHT_FACING_HPP_  // NOLINT
#define LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__TRAFFIC_LIGHT_FACING_HPP_  // NOLINT

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

namespace lanelet::autoware::validation
{
class TrafficLightFacingValidator : public lanelet::validation::MapValidator
{
public:
  // Write the validator's name here
  constexpr static const char * name() { return "mapping.traffic_light.correct_facing"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  /**
   * @brief This is the main part of this validator.
   *
   * @param map Lanelet map to validate
   * @return lanelet::validation::Issues
   */
  lanelet::validation::Issues check_traffic_light_facing(const lanelet::LaneletMap & map);

  /**
   * @brief Check whether the input linestring has a "traffic_light" type AND
   * "red_yellow_green" subtype.
   * @param linestring
   * @return true
   * @return false
   */
  bool is_red_yellow_green_traffic_light(const lanelet::ConstLineString3d & linestring);

  /**
   * @brief Get a stop line from a regulatory element
   *
   * @param reg_elem Pointer to the regulatory element object
   * @return lanelet::ConstLineString3d The stop_line referred by the regulatory element
   */
  lanelet::ConstLineString3d get_stop_line_from_reg_elem(
    const lanelet::RegulatoryElementConstPtr & reg_elem);

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

  /**
   * @brief Convert lanelet::ConstLineString3d to a Eigen::Vector3d.
   * The linestring must be made only from two points.
   *
   * @param linestring
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d linestring_to_vector3d(const lanelet::ConstLineString3d linestring);

  /**
   * @brief Returns a linestring that connects both ends of the left and right bounds
   * of the input lanelet. There might be two candidates (front and back) but this function
   * only returns the one near to the input 'reference'. This is used to get a directional
   * stop line from the lanelet here.
   *
   * @param lanelet
   * @param reference
   * @return lanelet::LineString3d
   */
  lanelet::LineString3d get_starting_edge_from_lanelet(
    const lanelet::ConstLanelet & lanelet, const lanelet::ConstLineString3d & reference);
};
}  // namespace lanelet::autoware::validation

// clang-format off
#endif  // LANELET2_MAP_VALIDATOR__VALIDATORS__TRAFFIC_LIGHT__TRAFFIC_LIGHT_FACING_HPP_  // NOLINT
// clang-format on

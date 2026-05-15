// Copyright 2026 TIER IV, Inc.
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

#ifndef METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_
#define METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <string>

namespace autoware::planning_data_analyzer::metrics
{

struct DrivableAreaComplianceResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
};

DrivableAreaComplianceResult calculate_drivable_area_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const lanelet::ConstLanelets & drivable_lanelets,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__DRIVABLE_AREA_COMPLIANCE_HPP_

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

#include "drivable_area_compliance.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <array>
#include <cstddef>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

constexpr double kContainmentEpsilon = 1.0e-6;

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return vehicle_info.vehicle_length_m > 0.0 && vehicle_info.vehicle_width_m > 0.0;
}

bool is_point_inside_any_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & drivable_lanelets)
{
  return std::any_of(
    drivable_lanelets.begin(), drivable_lanelets.end(), [&point](const auto & lanelet) {
      return boost::geometry::distance(lanelet.polygon2d().basicPolygon(), point) <=
             kContainmentEpsilon;
    });
}

}  // namespace

double calculate_drivable_area_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const lanelet::ConstLanelets & drivable_lanelets,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (
    trajectory.points.empty() || drivable_lanelets.empty() ||
    !is_vehicle_info_valid(vehicle_info)) {
    return 0.0;
  }

  const auto base_footprint = vehicle_info.createFootprint(0.0);
  const std::array<std::size_t, 4> corner_indices{
    autoware::vehicle_info_utils::VehicleInfo::FrontLeftIndex,
    autoware::vehicle_info_utils::VehicleInfo::FrontRightIndex,
    autoware::vehicle_info_utils::VehicleInfo::RearRightIndex,
    autoware::vehicle_info_utils::VehicleInfo::RearLeftIndex};

  for (const auto & point : trajectory.points) {
    const auto footprint = autoware_utils_geometry::transform_vector(
      base_footprint, autoware_utils_geometry::pose2transform(point.pose));

    for (const auto corner_index : corner_indices) {
      if (corner_index >= footprint.size()) {
        return 0.0;
      }
      if (!is_point_inside_any_lanelet(footprint.at(corner_index), drivable_lanelets)) {
        return 0.0;
      }
    }
  }

  return 1.0;
}

}  // namespace autoware::planning_data_analyzer::metrics

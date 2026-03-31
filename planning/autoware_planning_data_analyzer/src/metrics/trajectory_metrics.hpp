// Copyright 2025 TIER IV, Inc.
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

#ifndef METRICS__TRAJECTORY_METRICS_HPP_
#define METRICS__TRAJECTORY_METRICS_HPP_

#include "../data_types.hpp"
#include "driving_direction_compliance.hpp"
#include "lane_keeping.hpp"
#include "traffic_light_compliance.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <limits>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct HistoryComfortParameters
{
  double finite_difference_epsilon{1.0e-3};
  double max_longitudinal_acceleration{2.40};
  double min_longitudinal_acceleration{-4.05};
  double max_lateral_acceleration{4.89};
  double max_jerk_magnitude{8.37};
  double max_longitudinal_jerk{4.13};
  double max_yaw_rate{0.95};
  double max_yaw_acceleration{1.93};
};

// Structure for trajectory point-wise metrics
struct TrajectoryPointMetrics
{
  std::vector<double> longitudinal_accelerations;
  std::vector<double> lateral_accelerations;
  std::vector<double> lateral_jerks;
  std::vector<double> jerk_magnitudes;
  std::vector<double> longitudinal_jerks;
  std::vector<double> yaw_rates;
  std::vector<double> yaw_accelerations;
  std::vector<double> ttc_values;
  std::vector<double> lateral_deviations;
  std::vector<double> travel_distances;
  double history_comfort{0.0};
  double time_to_collision_within_bound{0.0};
  bool time_to_collision_within_bound_available{false};
  std::string time_to_collision_within_bound_reason{"unavailable"};
  double time_to_collision_infraction_time_s{std::numeric_limits<double>::infinity()};
  double lane_keeping{0.0};
  bool lane_keeping_available{false};
  std::string lane_keeping_reason{"unavailable"};
  double drivable_area_compliance{0.0};
  bool drivable_area_compliance_available{false};
  std::string drivable_area_compliance_reason{"unavailable"};
  double no_at_fault_collision{0.0};
  bool no_at_fault_collision_available{false};
  std::string no_at_fault_collision_reason{"unavailable"};
  double time_to_at_fault_collision_s{std::numeric_limits<double>::infinity()};
  double driving_direction_compliance{0.0};
  bool driving_direction_compliance_available{false};
  std::string driving_direction_compliance_reason{"unavailable"};
  double max_oncoming_progress_m{0.0};
  double traffic_light_compliance{0.0};
  bool traffic_light_compliance_available{false};
  std::string traffic_light_compliance_reason{"unavailable"};
};

/**
 * @brief Calculate point-wise metrics for a trajectory
 * @param sync_data Synchronized data containing trajectory and other information
 * @param route_handler Optional route handler for lane deviation calculation
 * @return Calculated metrics for each trajectory point
 */
TrajectoryPointMetrics calculate_trajectory_point_metrics(
  const std::shared_ptr<SynchronizedData> & sync_data,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler = nullptr,
  const HistoryComfortParameters & history_comfort_params = HistoryComfortParameters{},
  const LaneKeepingParameters & lane_keeping_params = LaneKeepingParameters{},
  const DrivingDirectionComplianceParameters & driving_direction_params =
    DrivingDirectionComplianceParameters{},
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info =
    autoware::vehicle_info_utils::VehicleInfo{});

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__TRAJECTORY_METRICS_HPP_

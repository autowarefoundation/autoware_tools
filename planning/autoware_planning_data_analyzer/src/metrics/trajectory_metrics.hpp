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

#include <autoware/route_handler/route_handler.hpp>

#include <memory>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

// Structure for trajectory point-wise metrics
struct TrajectoryPointMetrics
{
  std::vector<double> lateral_accelerations;
  std::vector<double> longitudinal_jerks;
  std::vector<double> ttc_values;
  std::vector<double> lateral_deviations;
  std::vector<double> travel_distances;
};

/**
 * @brief Calculate point-wise metrics for a trajectory
 * @param sync_data Synchronized data containing trajectory and other information
 * @param route_handler Optional route handler for lane deviation calculation
 * @return Calculated metrics for each trajectory point
 */
TrajectoryPointMetrics calculate_trajectory_point_metrics(
  const std::shared_ptr<SynchronizedData> & sync_data,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler = nullptr);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__TRAJECTORY_METRICS_HPP_

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

#ifndef METRICS__EXTENDED_COMFORT_HPP_
#define METRICS__EXTENDED_COMFORT_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>

namespace autoware::planning_data_analyzer::metrics
{

struct ExtendedComfortParameters
{
  double max_acceleration_rms{0.7};
  double max_jerk_rms{0.5};
  double max_yaw_rate_rms{0.1};
  double max_yaw_acceleration_rms{0.1};
  double finite_difference_epsilon{1.0e-3};
};

struct ExtendedComfortResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
};

/**
 * @brief Compare consecutive planned trajectories using RMS dynamic-signal discrepancies.
 *
 * Returns an available binary subscore only when both trajectories are long enough and the
 * configured thresholds are valid.
 */
ExtendedComfortResult calculate_extended_comfort(
  const autoware_planning_msgs::msg::Trajectory & previous_trajectory,
  const autoware_planning_msgs::msg::Trajectory & current_trajectory,
  const ExtendedComfortParameters & parameters);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EXTENDED_COMFORT_HPP_

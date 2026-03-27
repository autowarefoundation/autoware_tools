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

#ifndef METRICS__HISTORY_COMFORT_HPP_
#define METRICS__HISTORY_COMFORT_HPP_

#include "trajectory_metrics.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>

namespace autoware::planning_data_analyzer::metrics
{

void calculate_history_comfort_metrics(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const HistoryComfortParameters & history_comfort_params, TrajectoryPointMetrics & metrics);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__HISTORY_COMFORT_HPP_

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

#ifndef METRICS__LANE_KEEPING_HPP_
#define METRICS__LANE_KEEPING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct LaneKeepingParameters
{
  double max_lateral_deviation{0.5};
  double max_continuous_violation_time{2.0};
};

struct LaneKeepingSample
{
  rclcpp::Duration time_from_start{0, 0};
  double lateral_deviation{0.0};
  bool is_in_intersection{false};
};

double calculate_lane_keeping_score(
  const std::vector<LaneKeepingSample> & samples,
  const LaneKeepingParameters & parameters = LaneKeepingParameters{});

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__LANE_KEEPING_HPP_

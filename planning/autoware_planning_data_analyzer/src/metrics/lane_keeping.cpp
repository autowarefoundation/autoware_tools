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

#include "lane_keeping.hpp"

#include <cmath>
#include <optional>

namespace autoware::planning_data_analyzer::metrics
{

double calculate_lane_keeping_score(
  const std::vector<LaneKeepingSample> & samples, const LaneKeepingParameters & parameters)
{
  if (
    samples.empty() || parameters.max_lateral_deviation < 0.0 ||
    parameters.max_continuous_violation_time < 0.0) {
    return 0.0;
  }

  std::optional<rclcpp::Duration> violation_start_time;

  for (const auto & sample : samples) {
    if (sample.is_in_intersection || !std::isfinite(sample.lateral_deviation)) {
      violation_start_time.reset();
      continue;
    }

    if (std::abs(sample.lateral_deviation) <= parameters.max_lateral_deviation) {
      violation_start_time.reset();
      continue;
    }

    if (!violation_start_time.has_value()) {
      violation_start_time = sample.time_from_start;
    }

    const double violation_duration =
      (sample.time_from_start - violation_start_time.value()).seconds();
    if (violation_duration >= parameters.max_continuous_violation_time) {
      return 0.0;
    }
  }

  return 1.0;
}

}  // namespace autoware::planning_data_analyzer::metrics

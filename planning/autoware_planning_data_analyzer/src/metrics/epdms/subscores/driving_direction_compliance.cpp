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

#include "driving_direction_compliance.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

DrivingDirectionComplianceResult calculate_driving_direction_compliance(
  const std::vector<DrivingDirectionEvaluationPoint> & evaluation_points,
  const DrivingDirectionComplianceParameters & params)
{
  DrivingDirectionComplianceResult result;

  if (evaluation_points.empty()) {
    result.reason = "unavailable_no_evaluation_points";
    return result;
  }

  result.available = true;
  result.reason = "available";
  result.score = 1.0;

  std::vector<double> filtered_progress;
  filtered_progress.reserve(evaluation_points.size());
  for (const auto & point : evaluation_points) {
    filtered_progress.push_back(
      point.in_oncoming_traffic && !point.is_intersection ? std::max(0.0, point.progress_m) : 0.0);
  }

  for (size_t i = 0; i < evaluation_points.size(); ++i) {
    const double t_i = evaluation_points.at(i).time_from_start_s;
    double horizon_sum = 0.0;
    std::size_t horizon_count = 0U;
    double horizon_start_time_s = t_i;
    for (size_t j = 0; j <= i; ++j) {
      const double t_j = evaluation_points.at(j).time_from_start_s;
      if (t_i - t_j <= params.horizon_s + 1.0e-6) {
        horizon_sum += filtered_progress[j];
        ++horizon_count;
        horizon_start_time_s = std::min(horizon_start_time_s, t_j);
      }
    }
    if (horizon_sum > result.max_oncoming_progress_m) {
      result.max_oncoming_progress_m = horizon_sum;
      result.worst_window_start_time_s = horizon_start_time_s;
      result.worst_window_end_time_s = t_i;
      result.worst_window_sample_count = horizon_count;
    }
  }

  if (result.max_oncoming_progress_m < params.compliance_threshold_m) {
    result.score = 1.0;
  } else if (result.max_oncoming_progress_m < params.violation_threshold_m) {
    result.score = 0.5;
    result.reason = "minor_oncoming_progress";
  } else {
    result.score = 0.0;
    result.reason = "major_oncoming_progress";
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

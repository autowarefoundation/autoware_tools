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

#ifndef METRICS__EPDMS_AGGREGATION_HPP_
#define METRICS__EPDMS_AGGREGATION_HPP_

#include <cstddef>

namespace autoware::planning_data_analyzer::metrics
{

struct EpdmsMetricSnapshot
{
  double history_comfort{0.0};
  bool history_comfort_available{false};

  double extended_comfort{0.0};
  bool extended_comfort_available{false};

  double ego_progress{0.0};
  bool ego_progress_available{false};

  double time_to_collision_within_bound{0.0};
  bool time_to_collision_within_bound_available{false};

  double lane_keeping{0.0};
  bool lane_keeping_available{false};

  double drivable_area_compliance{0.0};
  bool drivable_area_compliance_available{false};

  double no_at_fault_collision{0.0};
  bool no_at_fault_collision_available{false};

  double driving_direction_compliance{0.0};
  bool driving_direction_compliance_available{false};

  double traffic_light_compliance{0.0};
  bool traffic_light_compliance_available{false};
};

struct HumanFilterMetrics
{
  double human_history_comfort{0.0};
  bool human_history_comfort_available{false};
  double filtered_history_comfort{0.0};
  bool history_comfort_filter_applied{false};

  double human_extended_comfort{0.0};
  bool human_extended_comfort_available{false};
  double filtered_extended_comfort{0.0};
  bool extended_comfort_filter_applied{false};

  double human_ego_progress{0.0};
  bool human_ego_progress_available{false};
  double filtered_ego_progress{0.0};
  bool ego_progress_filter_applied{false};

  double human_time_to_collision_within_bound{0.0};
  bool human_time_to_collision_within_bound_available{false};
  double filtered_time_to_collision_within_bound{0.0};
  bool time_to_collision_within_bound_filter_applied{false};

  double human_lane_keeping{0.0};
  bool human_lane_keeping_available{false};
  double filtered_lane_keeping{0.0};
  bool lane_keeping_filter_applied{false};

  double human_drivable_area_compliance{0.0};
  bool human_drivable_area_compliance_available{false};
  double filtered_drivable_area_compliance{0.0};
  bool drivable_area_compliance_filter_applied{false};

  double human_no_at_fault_collision{0.0};
  bool human_no_at_fault_collision_available{false};
  double filtered_no_at_fault_collision{0.0};
  bool no_at_fault_collision_filter_applied{false};

  double human_driving_direction_compliance{0.0};
  bool human_driving_direction_compliance_available{false};
  double filtered_driving_direction_compliance{0.0};
  bool driving_direction_compliance_filter_applied{false};

  double human_traffic_light_compliance{0.0};
  bool human_traffic_light_compliance_available{false};
  double filtered_traffic_light_compliance{0.0};
  bool traffic_light_compliance_filter_applied{false};
};

struct SyntheticEpdmsMetrics
{
  bool raw_available{false};
  double raw_multiplicative_metrics_prod{0.0};
  double raw_weighted_metrics{0.0};
  double raw_epdms{0.0};

  bool human_filtered_available{false};
  double human_filtered_multiplicative_metrics_prod{0.0};
  double human_filtered_weighted_metrics{0.0};
  double human_filtered_epdms{0.0};
};

HumanFilterMetrics calculate_human_filter_metrics(
  const EpdmsMetricSnapshot & agent_metrics, const EpdmsMetricSnapshot & human_metrics);

SyntheticEpdmsMetrics calculate_synthetic_epdms(
  const EpdmsMetricSnapshot & agent_metrics, const HumanFilterMetrics & human_filter_metrics);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS_AGGREGATION_HPP_

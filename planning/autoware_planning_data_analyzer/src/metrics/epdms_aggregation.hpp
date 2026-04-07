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
  struct Metric
  {
    double human_reference{0.0};
    bool human_reference_available{false};
    double filtered{0.0};
    bool filter_applied{false};
  };

  Metric history_comfort;
  Metric extended_comfort;
  Metric ego_progress;
  Metric time_to_collision_within_bound;
  Metric lane_keeping;
  Metric drivable_area_compliance;
  Metric no_at_fault_collision;
  Metric driving_direction_compliance;
  Metric traffic_light_compliance;
};

struct SyntheticEpdmsMetrics
{
  struct Stage
  {
    bool available{false};
    double multiplicative_metrics_prod{0.0};
    double weighted_metrics{0.0};
    double epdms{0.0};
  };

  Stage raw;
  Stage human_filtered;
};

HumanFilterMetrics calculate_human_filter_metrics(
  const EpdmsMetricSnapshot & agent_metrics, const EpdmsMetricSnapshot & human_metrics);

SyntheticEpdmsMetrics calculate_synthetic_epdms(
  const EpdmsMetricSnapshot & agent_metrics, const HumanFilterMetrics & human_filter_metrics);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS_AGGREGATION_HPP_

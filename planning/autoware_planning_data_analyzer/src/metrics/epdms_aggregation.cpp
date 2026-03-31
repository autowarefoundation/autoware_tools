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

#include "epdms_aggregation.hpp"

#include <cmath>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

constexpr double kHumanFilterZeroEpsilon = 1.0e-9;

double apply_human_filter(
  const double agent_value, const bool agent_available, const double human_value,
  const bool human_available, bool & applied)
{
  applied = agent_available && human_available && std::abs(human_value) <= kHumanFilterZeroEpsilon;
  return applied ? 1.0 : agent_value;
}

}  // namespace

HumanFilterMetrics calculate_human_filter_metrics(
  const EpdmsMetricSnapshot & agent_metrics, const EpdmsMetricSnapshot & human_metrics)
{
  HumanFilterMetrics result;

  result.human_history_comfort = human_metrics.history_comfort;
  result.human_history_comfort_available = human_metrics.history_comfort_available;
  result.filtered_history_comfort = apply_human_filter(
    agent_metrics.history_comfort, agent_metrics.history_comfort_available,
    human_metrics.history_comfort, human_metrics.history_comfort_available,
    result.history_comfort_filter_applied);

  result.human_extended_comfort = human_metrics.extended_comfort;
  result.human_extended_comfort_available = human_metrics.extended_comfort_available;
  result.filtered_extended_comfort = apply_human_filter(
    agent_metrics.extended_comfort, agent_metrics.extended_comfort_available,
    human_metrics.extended_comfort, human_metrics.extended_comfort_available,
    result.extended_comfort_filter_applied);

  result.human_ego_progress = human_metrics.ego_progress;
  result.human_ego_progress_available = human_metrics.ego_progress_available;
  result.filtered_ego_progress = apply_human_filter(
    agent_metrics.ego_progress, agent_metrics.ego_progress_available, human_metrics.ego_progress,
    human_metrics.ego_progress_available, result.ego_progress_filter_applied);

  result.human_time_to_collision_within_bound = human_metrics.time_to_collision_within_bound;
  result.human_time_to_collision_within_bound_available =
    human_metrics.time_to_collision_within_bound_available;
  result.filtered_time_to_collision_within_bound = apply_human_filter(
    agent_metrics.time_to_collision_within_bound,
    agent_metrics.time_to_collision_within_bound_available,
    human_metrics.time_to_collision_within_bound,
    human_metrics.time_to_collision_within_bound_available,
    result.time_to_collision_within_bound_filter_applied);

  result.human_lane_keeping = human_metrics.lane_keeping;
  result.human_lane_keeping_available = human_metrics.lane_keeping_available;
  result.filtered_lane_keeping = apply_human_filter(
    agent_metrics.lane_keeping, agent_metrics.lane_keeping_available, human_metrics.lane_keeping,
    human_metrics.lane_keeping_available, result.lane_keeping_filter_applied);

  result.human_drivable_area_compliance = human_metrics.drivable_area_compliance;
  result.human_drivable_area_compliance_available =
    human_metrics.drivable_area_compliance_available;
  result.filtered_drivable_area_compliance = apply_human_filter(
    agent_metrics.drivable_area_compliance, agent_metrics.drivable_area_compliance_available,
    human_metrics.drivable_area_compliance, human_metrics.drivable_area_compliance_available,
    result.drivable_area_compliance_filter_applied);

  result.human_no_at_fault_collision = human_metrics.no_at_fault_collision;
  result.human_no_at_fault_collision_available = human_metrics.no_at_fault_collision_available;
  result.filtered_no_at_fault_collision = apply_human_filter(
    agent_metrics.no_at_fault_collision, agent_metrics.no_at_fault_collision_available,
    human_metrics.no_at_fault_collision, human_metrics.no_at_fault_collision_available,
    result.no_at_fault_collision_filter_applied);

  result.human_driving_direction_compliance = human_metrics.driving_direction_compliance;
  result.human_driving_direction_compliance_available =
    human_metrics.driving_direction_compliance_available;
  result.filtered_driving_direction_compliance = apply_human_filter(
    agent_metrics.driving_direction_compliance,
    agent_metrics.driving_direction_compliance_available,
    human_metrics.driving_direction_compliance,
    human_metrics.driving_direction_compliance_available,
    result.driving_direction_compliance_filter_applied);

  result.human_traffic_light_compliance = human_metrics.traffic_light_compliance;
  result.human_traffic_light_compliance_available =
    human_metrics.traffic_light_compliance_available;
  result.filtered_traffic_light_compliance = apply_human_filter(
    agent_metrics.traffic_light_compliance, agent_metrics.traffic_light_compliance_available,
    human_metrics.traffic_light_compliance, human_metrics.traffic_light_compliance_available,
    result.traffic_light_compliance_filter_applied);

  return result;
}

SyntheticEpdmsMetrics calculate_synthetic_epdms(
  const EpdmsMetricSnapshot & agent_metrics, const HumanFilterMetrics & human_filter_metrics)
{
  SyntheticEpdmsMetrics result;
  constexpr double kSyntheticWeightedDenominator = 16.0;

  const bool raw_multiplicative_available = agent_metrics.no_at_fault_collision_available &&
                                            agent_metrics.drivable_area_compliance_available &&
                                            agent_metrics.driving_direction_compliance_available &&
                                            agent_metrics.traffic_light_compliance_available;
  const bool raw_weighted_available = agent_metrics.ego_progress_available &&
                                      agent_metrics.time_to_collision_within_bound_available &&
                                      agent_metrics.lane_keeping_available &&
                                      agent_metrics.history_comfort_available &&
                                      agent_metrics.extended_comfort_available;
  result.raw_available = raw_multiplicative_available && raw_weighted_available;
  if (result.raw_available) {
    result.raw_multiplicative_metrics_prod =
      agent_metrics.no_at_fault_collision * agent_metrics.drivable_area_compliance *
      agent_metrics.driving_direction_compliance * agent_metrics.traffic_light_compliance;
    result.raw_weighted_metrics =
      (5.0 * agent_metrics.ego_progress + 5.0 * agent_metrics.time_to_collision_within_bound +
       2.0 * agent_metrics.lane_keeping + 2.0 * agent_metrics.history_comfort +
       2.0 * agent_metrics.extended_comfort) /
      kSyntheticWeightedDenominator;
    result.raw_epdms = result.raw_multiplicative_metrics_prod * result.raw_weighted_metrics;
  }

  const bool human_filtered_multiplicative_available =
    agent_metrics.no_at_fault_collision_available &&
    agent_metrics.drivable_area_compliance_available &&
    agent_metrics.driving_direction_compliance_available &&
    agent_metrics.traffic_light_compliance_available;
  const bool human_filtered_weighted_available =
    agent_metrics.ego_progress_available &&
    agent_metrics.time_to_collision_within_bound_available &&
    agent_metrics.lane_keeping_available && agent_metrics.history_comfort_available &&
    agent_metrics.extended_comfort_available;
  result.human_filtered_available =
    human_filtered_multiplicative_available && human_filtered_weighted_available;
  if (result.human_filtered_available) {
    result.human_filtered_multiplicative_metrics_prod =
      human_filter_metrics.filtered_no_at_fault_collision *
      human_filter_metrics.filtered_drivable_area_compliance *
      human_filter_metrics.filtered_driving_direction_compliance *
      human_filter_metrics.filtered_traffic_light_compliance;
    const double filtered_ego_progress =
      human_filter_metrics.ego_progress_filter_applied ? 1.0 : agent_metrics.ego_progress;
    result.human_filtered_weighted_metrics =
      (5.0 * filtered_ego_progress +
       5.0 * human_filter_metrics.filtered_time_to_collision_within_bound +
       2.0 * human_filter_metrics.filtered_lane_keeping +
       2.0 * human_filter_metrics.filtered_history_comfort +
       2.0 * human_filter_metrics.filtered_extended_comfort) /
      kSyntheticWeightedDenominator;
    result.human_filtered_epdms =
      result.human_filtered_multiplicative_metrics_prod * result.human_filtered_weighted_metrics;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

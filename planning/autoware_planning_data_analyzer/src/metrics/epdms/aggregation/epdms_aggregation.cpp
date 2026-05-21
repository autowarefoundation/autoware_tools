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
constexpr double kSyntheticWeightedDenominator = 16.0;

double apply_human_filter(
  const double agent_value, const bool agent_available, const double human_value,
  const bool human_available, bool & applied)
{
  applied = agent_available && human_available && std::abs(human_value) <= kHumanFilterZeroEpsilon;
  return applied ? 1.0 : agent_value;
}

void populate_human_filter_metric(
  HumanFilterMetrics::Metric & metric, const double agent_value, const bool agent_available,
  const double human_value, const bool human_available)
{
  metric.human_reference = human_value;
  metric.human_reference_available = human_available;
  metric.filtered = apply_human_filter(
    agent_value, agent_available, human_value, human_available, metric.filter_applied);
}

double calculate_multiplicative_metrics_prod(
  const double no_at_fault_collision, const double drivable_area_compliance,
  const double driving_direction_compliance, const double traffic_light_compliance)
{
  return no_at_fault_collision * drivable_area_compliance * driving_direction_compliance *
         traffic_light_compliance;
}

double calculate_weighted_metrics(
  const double ego_progress, const double time_to_collision_within_bound, const double lane_keeping,
  const double history_comfort, const double extended_comfort)
{
  return (5.0 * ego_progress + 5.0 * time_to_collision_within_bound + 2.0 * lane_keeping +
          2.0 * history_comfort + 2.0 * extended_comfort) /
         kSyntheticWeightedDenominator;
}

}  // namespace

HumanFilterMetrics calculate_human_filter_metrics(
  const EpdmsMetricSnapshot & agent_metrics, const EpdmsMetricSnapshot & human_metrics)
{
  HumanFilterMetrics result;

  populate_human_filter_metric(
    result.history_comfort, agent_metrics.history_comfort, agent_metrics.history_comfort_available,
    human_metrics.history_comfort, human_metrics.history_comfort_available);

  populate_human_filter_metric(
    result.extended_comfort, agent_metrics.extended_comfort,
    agent_metrics.extended_comfort_available, human_metrics.extended_comfort,
    human_metrics.extended_comfort_available);

  populate_human_filter_metric(
    result.ego_progress, agent_metrics.ego_progress, agent_metrics.ego_progress_available,
    human_metrics.ego_progress, human_metrics.ego_progress_available);

  populate_human_filter_metric(
    result.time_to_collision_within_bound, agent_metrics.time_to_collision_within_bound,
    agent_metrics.time_to_collision_within_bound_available,
    human_metrics.time_to_collision_within_bound,
    human_metrics.time_to_collision_within_bound_available);

  populate_human_filter_metric(
    result.lane_keeping, agent_metrics.lane_keeping, agent_metrics.lane_keeping_available,
    human_metrics.lane_keeping, human_metrics.lane_keeping_available);

  populate_human_filter_metric(
    result.drivable_area_compliance, agent_metrics.drivable_area_compliance,
    agent_metrics.drivable_area_compliance_available, human_metrics.drivable_area_compliance,
    human_metrics.drivable_area_compliance_available);

  populate_human_filter_metric(
    result.no_at_fault_collision, agent_metrics.no_at_fault_collision,
    agent_metrics.no_at_fault_collision_available, human_metrics.no_at_fault_collision,
    human_metrics.no_at_fault_collision_available);

  populate_human_filter_metric(
    result.driving_direction_compliance, agent_metrics.driving_direction_compliance,
    agent_metrics.driving_direction_compliance_available,
    human_metrics.driving_direction_compliance,
    human_metrics.driving_direction_compliance_available);

  populate_human_filter_metric(
    result.traffic_light_compliance, agent_metrics.traffic_light_compliance,
    agent_metrics.traffic_light_compliance_available, human_metrics.traffic_light_compliance,
    human_metrics.traffic_light_compliance_available);

  return result;
}

SyntheticEpdmsMetrics calculate_synthetic_epdms(
  const EpdmsMetricSnapshot & agent_metrics, const HumanFilterMetrics & human_filter_metrics)
{
  SyntheticEpdmsMetrics result;

  const bool raw_multiplicative_available = agent_metrics.no_at_fault_collision_available &&
                                            agent_metrics.drivable_area_compliance_available &&
                                            agent_metrics.driving_direction_compliance_available &&
                                            agent_metrics.traffic_light_compliance_available;
  const bool raw_weighted_available = agent_metrics.ego_progress_available &&
                                      agent_metrics.time_to_collision_within_bound_available &&
                                      agent_metrics.lane_keeping_available &&
                                      agent_metrics.history_comfort_available &&
                                      agent_metrics.extended_comfort_available;
  result.raw.available = raw_multiplicative_available && raw_weighted_available;
  if (result.raw.available) {
    result.raw.multiplicative_metrics_prod = calculate_multiplicative_metrics_prod(
      agent_metrics.no_at_fault_collision, agent_metrics.drivable_area_compliance,
      agent_metrics.driving_direction_compliance, agent_metrics.traffic_light_compliance);
    result.raw.weighted_metrics = calculate_weighted_metrics(
      agent_metrics.ego_progress, agent_metrics.time_to_collision_within_bound,
      agent_metrics.lane_keeping, agent_metrics.history_comfort, agent_metrics.extended_comfort);
    result.raw.epdms = result.raw.multiplicative_metrics_prod * result.raw.weighted_metrics;
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
  result.human_filtered.available =
    human_filtered_multiplicative_available && human_filtered_weighted_available;
  if (result.human_filtered.available) {
    result.human_filtered.multiplicative_metrics_prod = calculate_multiplicative_metrics_prod(
      human_filter_metrics.no_at_fault_collision.filtered,
      human_filter_metrics.drivable_area_compliance.filtered,
      human_filter_metrics.driving_direction_compliance.filtered,
      human_filter_metrics.traffic_light_compliance.filtered);
    result.human_filtered.weighted_metrics = calculate_weighted_metrics(
      human_filter_metrics.ego_progress.filtered,
      human_filter_metrics.time_to_collision_within_bound.filtered,
      human_filter_metrics.lane_keeping.filtered, human_filter_metrics.history_comfort.filtered,
      agent_metrics.extended_comfort);
    result.human_filtered.epdms =
      result.human_filtered.multiplicative_metrics_prod * result.human_filtered.weighted_metrics;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics

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

#ifndef METRICS__EVALUATOR__EVALUATOR_HPP_
#define METRICS__EVALUATOR__EVALUATOR_HPP_

#include "data_types.hpp"
#include "metrics/metric_types.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics::evaluator
{

struct EvaluatorConfig
{
  std::string name;
  std::vector<std::string> topics;
  std::vector<std::string> exclusion_rules;
};

struct TimestampedDouble
{
  int64_t timestamp_ns{0};
  double value{0.0};
};

/** @brief One evaluator metric value synced to an odometry timestamp from the evaluation rosbag. */
struct EvaluatorMetricMeasurement
{
  double value{0.0};
  std::vector<std::string> matched_exclusion_rules;
};

struct EvaluatorMetricStats
{
  ::metrics::Statistics value;  // max, mean, min
  std::size_t count{0};         // number of measurements
  double percentile_95{0.0};
  double percentile_99{0.0};
};

struct EvaluatorMetricResult
{
  std::string metric_key;
  std::string rule;  // "all", "in_<region>", or "outside_<region>"
  EvaluatorMetricStats stats;
};

/** @brief One evaluator_config.yaml entry: topics sharing the same exclusion rules. */
struct EvaluatorMetricGroup
{
  std::string group_name;
  std::vector<std::string> exclusion_rules;
  std::map<std::string, std::vector<EvaluatorMetricMeasurement>> measurements_by_metric_key;
};

std::vector<EvaluatorConfig> load_evaluator_configs_from_yaml_file(const std::string & path);

std::vector<EvaluatorConfig> load_evaluator_configs(rclcpp::Node & node);

std::unordered_set<std::string> collect_evaluator_topics(
  const std::vector<EvaluatorConfig> & evaluator_configs);

bool try_append_evaluator_metric_message(
  const std::string & topic_name,
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & serialized_message,
  const std::unordered_set<std::string> & evaluator_topics,
  std::map<std::string, std::vector<TimestampedDouble>> & values_by_topic,
  const rclcpp::Logger & logger);

/** @brief Drop odometry samples while ego remains near the first recorded pose (pre-departure). */
std::vector<std::shared_ptr<Odometry>> filter_odometry_outside_initial_pose(
  const std::vector<std::shared_ptr<Odometry>> & kinematic_states,
  double initial_pose_radius_m = 2.0);

std::vector<EvaluatorMetricGroup> build_evaluator_metric_groups(
  const std::vector<EvaluatorConfig> & evaluator_configs,
  std::map<std::string, std::vector<TimestampedDouble>> & values_by_topic,
  const std::vector<std::shared_ptr<Odometry>> & kinematic_states,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  double sync_tolerance_ms, const rclcpp::Logger & logger);

std::vector<EvaluatorMetricResult> aggregate_metric_measurements(
  const std::vector<EvaluatorMetricMeasurement> & measurements,
  const std::vector<std::string> & exclusion_rule_names, const std::string & metric_key);

nlohmann::json evaluator_metric_results_to_json(
  const std::vector<EvaluatorMetricResult> & metric_results);

}  // namespace autoware::planning_data_analyzer::metrics::evaluator

#endif  // METRICS__EVALUATOR__EVALUATOR_HPP_

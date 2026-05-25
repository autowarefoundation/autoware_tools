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

#include "metrics/evaluator/evaluator.hpp"

#include "metrics/geometry/lanelet_queries.hpp"
#include "serialized_bag_message.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>

#include <tier4_simulation_msgs/msg/user_defined_value.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics::evaluator
{

namespace
{

EvaluatorMetricStats compute_stats(const std::vector<double> & values)
{
  EvaluatorMetricStats stats;
  stats.count = values.size();
  if (values.empty()) {
    return stats;
  }

  stats.value = ::metrics::calculate_statistics(values);
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  stats.percentile_95 = ::metrics::compute_percentile(sorted, 0.95);
  stats.percentile_99 = ::metrics::compute_percentile(sorted, 0.99);
  return stats;
}

std::string make_description_format(const std::string & view_prefix)
{
  if (view_prefix == "all") {
    return "Aggregate of all values in the recorded rosbag and synchronized to odometry.";
  }
  if (view_prefix == "included") {
    return "Aggregate of odometry-synced values in the recorded rosbag outside excluded map "
           "regions.";
  }

  const std::string excluded_prefix = "excluded/";
  if (view_prefix.rfind(excluded_prefix, 0) == 0) {
    const auto rule_name = view_prefix.substr(excluded_prefix.size());
    return "Aggregate of odometry-synced values in the recorded rosbag where map exclusion rule '" +
           rule_name + "' applies.";
  }

  return "";
}

std::string make_view_metric_description(
  const std::string & view_prefix, const std::string & metric_key)
{
  const auto view_desc = make_description_format(view_prefix);
  if (view_desc.empty()) {
    return metric_key;
  }
  return metric_key + ": " + view_desc;
}

void emit_view_stats(
  nlohmann::json & json_output, const std::string & view_prefix, const std::string & metric_key,
  const EvaluatorMetricStats & stats)
{
  if (stats.count == 0) {
    return;
  }

  // JSON output format per topic
  const std::string base = view_prefix + "/" + metric_key;
  json_output[base + "/description"] = make_view_metric_description(view_prefix, metric_key);
  json_output[base + "/count"] = stats.count;
  json_output[base + "/mean"] = stats.value.mean;
  json_output[base + "/min"] = stats.value.min_val;
  json_output[base + "/max"] = stats.value.max_val;
  json_output[base + "/percentile_95"] = stats.percentile_95;
  json_output[base + "/percentile_99"] = stats.percentile_99;
}

std::vector<std::string> read_string_sequence(const YAML::Node & node)
{
  if (!node || !node.IsSequence()) {
    return {};
  }

  std::vector<std::string> values;
  values.reserve(node.size());
  for (const auto & item : node) {
    values.push_back(item.as<std::string>());
  }
  return values;
}

EvaluatorConfig parse_evaluator_entry(const YAML::Node & entry)
{
  if (!entry.IsMap()) {
    throw std::runtime_error("Each evaluator_config entry must be a map");
  }

  EvaluatorConfig config;
  config.name = entry["name"] ? entry["name"].as<std::string>() : "";
  config.topics = read_string_sequence(entry["topics"]);
  if (entry["exclusion"] && entry["exclusion"]["rules"]) {
    config.exclusion_rules = read_string_sequence(entry["exclusion"]["rules"]);
  }

  if (config.name.empty()) {
    throw std::runtime_error("evaluator_config entry is missing 'name'");
  }
  if (config.topics.empty()) {
    throw std::runtime_error(
      "evaluator_config entry '" + config.name + "' is missing non-empty 'topics'");
  }

  return config;
}

std::optional<double> parse_user_defined_value_string(const std::string & value)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> find_closest_value(
  const std::vector<TimestampedDouble> & timestamped_values, const int64_t target_time_ns,
  const double tolerance_ms)
{
  if (timestamped_values.empty()) {
    return std::nullopt;
  }

  const double tolerance_ns = tolerance_ms * 1e6;
  auto closest = timestamped_values.begin();
  double min_diff = std::abs(static_cast<double>(closest->timestamp_ns - target_time_ns));

  for (auto itr = timestamped_values.begin(); itr != timestamped_values.end(); ++itr) {
    const double diff = std::abs(static_cast<double>(itr->timestamp_ns - target_time_ns));
    if (diff < min_diff) {
      min_diff = diff;
      closest = itr;
    }
  }

  if (min_diff > tolerance_ns) {
    return std::nullopt;
  }

  return closest->value;
}

/** @brief Evaluate exclusion rules filtering. */
std::vector<std::string> evaluate_exclusions(
  const geometry_msgs::msg::Pose & pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<std::string> & exclusion_rule_names)
{
  std::vector<std::string> matched_rules;
  for (const auto & rule_name : exclusion_rule_names) {
    if (rule_name == "intersection_area") {
      if (metrics::is_pose_in_intersection_area(pose, route_handler)) {
        matched_rules.push_back(rule_name);
      }
    } else {
      throw std::runtime_error(
        "Unknown evaluator exclusion rule: " + rule_name + ". Supported rules: intersection_area.");
    }
  }
  return matched_rules;
}

/** @brief Build evaluator metric measurements synced to odometry timestamps. */
std::vector<EvaluatorMetricMeasurement> build_synced_metric_measurements(
  const std::vector<std::shared_ptr<Odometry>> & kinematic_states,
  const std::vector<TimestampedDouble> & metric_values,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<std::string> & exclusion_rule_names, const double sync_tolerance_ms)
{
  std::vector<EvaluatorMetricMeasurement> measurements;
  measurements.reserve(kinematic_states.size());

  for (const auto & odometry : kinematic_states) {
    if (!odometry) {
      continue;
    }

    // Sync metric values to odometry timestamps
    const int64_t timestamp_ns = rclcpp::Time(odometry->header.stamp).nanoseconds();
    const auto metric_value = find_closest_value(metric_values, timestamp_ns, sync_tolerance_ms);
    if (!metric_value.has_value()) {
      continue;
    }

    // Build evaluator metric measurement
    EvaluatorMetricMeasurement measurement;
    measurement.value = *metric_value;
    measurement.matched_exclusion_rules =
      evaluate_exclusions(odometry->pose.pose, route_handler, exclusion_rule_names);
    measurements.push_back(measurement);
  }

  return measurements;
}

}  // namespace

/** @brief Drop odometry samples while ego remains near the first recorded pose (pre-departure). */
std::vector<std::shared_ptr<Odometry>> filter_odometry_outside_initial_pose(
  const std::vector<std::shared_ptr<Odometry>> & kinematic_states,
  const double initial_pose_radius_m)
{
  std::vector<std::shared_ptr<Odometry>> filtered;
  filtered.reserve(kinematic_states.size());

  std::optional<std::pair<double, double>> initial_xy;
  for (const auto & odometry : kinematic_states) {
    if (!odometry) {
      continue;
    }

    const double x = odometry->pose.pose.position.x;
    const double y = odometry->pose.pose.position.y;
    if (!initial_xy.has_value()) {
      initial_xy = std::make_pair(x, y);
      continue;
    }

    const double dx = x - initial_xy->first;
    const double dy = y - initial_xy->second;
    if (std::hypot(dx, dy) < initial_pose_radius_m) {
      continue;
    }

    filtered.push_back(odometry);
  }

  return filtered;
}

std::vector<EvaluatorConfig> load_evaluator_configs_from_yaml_file(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("evaluator_config file not found: " + path);
  }

  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node evaluator_config_node =
    root["evaluator_config"] ? root["evaluator_config"] : root;

  if (!evaluator_config_node || !evaluator_config_node.IsSequence()) {
    throw std::runtime_error(
      "evaluator_config file must contain a sequence at key 'evaluator_config': " + path);
  }

  std::vector<EvaluatorConfig> configs;
  configs.reserve(evaluator_config_node.size());
  for (const auto & entry : evaluator_config_node) {
    configs.push_back(parse_evaluator_entry(entry));
  }

  return configs;
}

std::vector<EvaluatorConfig> load_evaluator_configs(rclcpp::Node & node)
{
  constexpr const char * kEvaluatorConfigPathParam = "open_loop.evaluator_config_path";
  if (!node.has_parameter(kEvaluatorConfigPathParam)) {
    node.declare_parameter<std::string>(kEvaluatorConfigPathParam, "");
  }

  std::string config_path = node.get_parameter(kEvaluatorConfigPathParam).as_string();
  if (config_path.empty()) {
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_planning_data_analyzer");
    config_path = (std::filesystem::path(share_dir) / "config" / "evaluator_config.yaml").string();
  }

  try {
    auto configs = load_evaluator_configs_from_yaml_file(config_path);
    RCLCPP_INFO(
      node.get_logger(), "Loaded %zu evaluator config(s) from %s", configs.size(),
      config_path.c_str());
    return configs;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node.get_logger(), "Failed to load evaluator_config from '%s': %s", config_path.c_str(),
      e.what());
    return {};
  }
}

std::unordered_set<std::string> collect_evaluator_topics(
  const std::vector<EvaluatorConfig> & evaluator_configs)
{
  std::unordered_set<std::string> evaluator_topics;
  for (const auto & evaluator_config : evaluator_configs) {
    for (const auto & topic : evaluator_config.topics) {
      if (!topic.empty()) {
        evaluator_topics.insert(topic);
      }
    }
  }
  return evaluator_topics;
}

/** @brief Check if this topic is an evaluator aggregation target. */
bool try_append_evaluator_metric_message(
  const std::string & topic_name,
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & serialized_message,
  const std::unordered_set<std::string> & evaluator_topics,
  std::map<std::string, std::vector<TimestampedDouble>> & values_by_topic,
  const rclcpp::Logger & logger)
{
  if (evaluator_topics.count(topic_name) == 0) {
    return false;
  }

  using tier4_simulation_msgs::msg::UserDefinedValue;
  const auto timestamp_ns = static_cast<int64_t>(get_timestamp_ns(*serialized_message));
  try {
    UserDefinedValue msg;
    rclcpp::Serialization<UserDefinedValue> serializer;
    rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
    serializer.deserialize_message(&serialized_msg, &msg);
    if (const auto value = parse_user_defined_value_string(msg.value)) {
      values_by_topic[topic_name].push_back({timestamp_ns, *value});
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger, "Failed to deserialize evaluator metric on %s: %s", topic_name.c_str(), e.what());
  }
  return true;
}

std::vector<EvaluatorMetricGroup> build_evaluator_metric_groups(
  const std::vector<EvaluatorConfig> & evaluator_configs,
  std::map<std::string, std::vector<TimestampedDouble>> & values_by_topic,
  const std::vector<std::shared_ptr<Odometry>> & kinematic_states,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const double sync_tolerance_ms, const rclcpp::Logger & logger)
{
  const auto evaluation_kinematic_states = filter_odometry_outside_initial_pose(kinematic_states);
  if (evaluation_kinematic_states.empty()) {
    RCLCPP_INFO(
      logger, "Evaluator metrics skipped: no odometry samples outside the initial pose radius.");
    return {};
  }

  for (auto & [topic, values] : values_by_topic) {
    std::sort(values.begin(), values.end(), [](const auto & a, const auto & b) {
      return a.timestamp_ns < b.timestamp_ns;
    });
  }

  // Build evaluator metric groups
  std::vector<EvaluatorMetricGroup> evaluator_metric_groups;
  for (const auto & evaluator_config : evaluator_configs) {
    EvaluatorMetricGroup group;
    group.group_name = evaluator_config.name;
    group.exclusion_rules = evaluator_config.exclusion_rules;

    // Build evaluator metric measurements for each topic
    for (const auto & topic : evaluator_config.topics) {
      const auto slash = topic.rfind('/');
      const std::string metric_key = slash == std::string::npos ? topic : topic.substr(slash + 1);
      const auto values_itr = values_by_topic.find(topic);
      if (values_itr == values_by_topic.end() || values_itr->second.empty()) {
        RCLCPP_INFO(
          logger, "Evaluator '%s': no messages on %s", evaluator_config.name.c_str(),
          topic.c_str());
        continue;
      }

      // Analyze values and sync them to odometry timestamps
      auto measurements = build_synced_metric_measurements(
        evaluation_kinematic_states, values_itr->second, route_handler,
        evaluator_config.exclusion_rules, sync_tolerance_ms);
      if (measurements.empty()) {
        continue;
      }

      // Add measurements to the group
      group.measurements_by_metric_key[metric_key] = std::move(measurements);
      RCLCPP_INFO(
        logger, "Evaluator '%s': collected %zu measurements for %s (%s)",
        evaluator_config.name.c_str(), group.measurements_by_metric_key[metric_key].size(),
        metric_key.c_str(), topic.c_str());
    }

    // Add group to the list if it has measurements
    if (!group.measurements_by_metric_key.empty()) {
      evaluator_metric_groups.push_back(std::move(group));
    }
  }

  return evaluator_metric_groups;
}

EvaluatorMetricAggregationResult aggregate_metric_measurements(
  const std::vector<EvaluatorMetricMeasurement> & measurements,
  const std::vector<std::string> & exclusion_rule_names, const std::string & metric_key)
{
  EvaluatorMetricAggregationResult result;
  result.metric_key = metric_key;
  for (const auto & rule_name : exclusion_rule_names) {
    result.excluded_stats_by_rule.emplace(rule_name, EvaluatorMetricStats{});
  }

  std::vector<double> all_values;
  std::vector<double> included_values;
  std::map<std::string, std::vector<double>> excluded_values_by_rule;

  all_values.reserve(measurements.size());
  included_values.reserve(measurements.size());

  for (const auto & measurement : measurements) {
    all_values.push_back(measurement.value);

    if (measurement.matched_exclusion_rules.empty()) {
      included_values.push_back(measurement.value);
      continue;
    }

    for (const auto & rule_name : measurement.matched_exclusion_rules) {
      excluded_values_by_rule[rule_name].push_back(measurement.value);
    }
  }

  // Aggregate all values, included values, and excluded values per exclusion rule.
  // Metrics count: all = each_excluded_topic + included_topic
  // (There may be some discrepancies due to synchronization issues.)
  result.all_stats = compute_stats(all_values);
  result.included_stats = compute_stats(included_values);
  for (const auto & rule_name : exclusion_rule_names) {
    const auto itr = excluded_values_by_rule.find(rule_name);
    if (itr == excluded_values_by_rule.end()) {
      continue;
    }
    result.excluded_stats_by_rule[rule_name] = compute_stats(itr->second);
  }

  return result;
}

nlohmann::json evaluator_group_aggregations_to_json(
  const std::vector<EvaluatorMetricAggregationResult> & metric_results)
{
  nlohmann::json json_output;
  for (const auto & result : metric_results) {
    emit_view_stats(json_output, "all", result.metric_key, result.all_stats);
    emit_view_stats(json_output, "included", result.metric_key, result.included_stats);
    for (const auto & [rule_name, stats] : result.excluded_stats_by_rule) {
      emit_view_stats(json_output, "excluded/" + rule_name, result.metric_key, stats);
    }
  }
  return json_output;
}

}  // namespace autoware::planning_data_analyzer::metrics::evaluator

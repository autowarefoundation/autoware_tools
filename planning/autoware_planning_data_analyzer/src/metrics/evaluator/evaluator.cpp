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

std::string make_in_rule_name(const std::string & region_name)
{
  return "in_" + region_name;
}

std::string make_outside_rule_name(const std::string & region_name)
{
  return "outside_" + region_name;
}

std::string make_description_format(const std::string & rule)
{
  if (rule == "all") {
    return "Aggregate of all values in the recorded rosbag and synchronized to odometry.";
  }

  const std::string in_prefix = "in_";
  const std::string outside_prefix = "outside_";
  if (rule.rfind(in_prefix, 0) == 0) {
    const auto region_name = rule.substr(in_prefix.size());
    return "Aggregate of odometry-synced values in the recorded rosbag inside map region '" +
           region_name + "'.";
  }
  if (rule.rfind(outside_prefix, 0) == 0) {
    const auto region_name = rule.substr(outside_prefix.size());
    return "Aggregate of odometry-synced values in the recorded rosbag outside map region '" +
           region_name + "'.";
  }

  return "";
}

std::string make_view_metric_description(const std::string & rule, const std::string & metric_key)
{
  return metric_key + ": " + make_description_format(rule);
}

void emit_metric_result(nlohmann::json & json_output, const EvaluatorMetricResult & result)
{
  if (result.stats.count == 0) {
    return;
  }

  const std::string base = result.rule + "/" + result.metric_key;
  json_output[base + "/description"] = make_view_metric_description(result.rule, result.metric_key);
  json_output[base + "/count"] = result.stats.count;
  json_output[base + "/mean"] = result.stats.value.mean;
  json_output[base + "/min"] = result.stats.value.min_val;
  json_output[base + "/max"] = result.stats.value.max_val;
  json_output[base + "/percentile_95"] = result.stats.percentile_95;
  json_output[base + "/percentile_99"] = result.stats.percentile_99;
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
  const auto itr = std::lower_bound(
    timestamped_values.begin(), timestamped_values.end(), target_time_ns,
    [](const TimestampedDouble & value, int64_t target) { return value.timestamp_ns < target; });

  auto closest_itr = itr;
  if (itr == timestamped_values.end()) {
    closest_itr = std::prev(itr);
  } else if (itr != timestamped_values.begin()) {
    const auto prev_itr = std::prev(itr);
    const double diff_prev = std::abs(static_cast<double>(prev_itr->timestamp_ns - target_time_ns));
    const double diff_curr = std::abs(static_cast<double>(itr->timestamp_ns - target_time_ns));
    if (diff_prev < diff_curr) {
      closest_itr = prev_itr;
    }
  }
  const double min_diff = std::abs(static_cast<double>(closest_itr->timestamp_ns - target_time_ns));
  if (min_diff > tolerance_ns) {
    return std::nullopt;
  }
  return closest_itr->value;
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

std::vector<EvaluatorMetricResult> aggregate_metric_measurements(
  const std::vector<EvaluatorMetricMeasurement> & measurements,
  const std::vector<std::string> & exclusion_rule_names, const std::string & metric_key)
{
  std::vector<double> all_values;
  all_values.reserve(measurements.size());
  for (const auto & measurement : measurements) {
    all_values.push_back(measurement.value);
  }

  std::vector<EvaluatorMetricResult> results;
  results.push_back({metric_key, "all", compute_stats(all_values)});
  for (const auto & region_name : exclusion_rule_names) {
    std::vector<double> in_region_values;
    std::vector<double> outside_region_values;
    in_region_values.reserve(measurements.size());
    outside_region_values.reserve(measurements.size());
    for (const auto & measurement : measurements) {
      const bool in_region =
        std::find(
          measurement.matched_exclusion_rules.begin(), measurement.matched_exclusion_rules.end(),
          region_name) != measurement.matched_exclusion_rules.end();
      if (in_region) {
        in_region_values.push_back(measurement.value);
      } else {
        outside_region_values.push_back(measurement.value);
      }
    }
    results.push_back(
      {metric_key, make_in_rule_name(region_name), compute_stats(in_region_values)});
    results.push_back(
      {metric_key, make_outside_rule_name(region_name), compute_stats(outside_region_values)});
  }
  return results;
}

nlohmann::json evaluator_metric_results_to_json(
  const std::vector<EvaluatorMetricResult> & metric_results)
{
  nlohmann::json json_output;
  for (const auto & result : metric_results) {
    emit_metric_result(json_output, result);
  }
  return json_output;
}

}  // namespace autoware::planning_data_analyzer::metrics::evaluator

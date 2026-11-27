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

#include "closed_loop/closed_loop_scene.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

constexpr int kSchemaVersion = 1;

const nlohmann::json & require_field(
  const nlohmann::json & object, const std::string & key, const std::string & where)
{
  if (!object.is_object() || !object.contains(key)) {
    throw std::invalid_argument("scene file: missing " + where + key);
  }
  return object.at(key);
}

std::string require_string(
  const nlohmann::json & object, const std::string & key, const std::string & where = "")
{
  const auto & value = require_field(object, key, where);
  if (!value.is_string() || value.get<std::string>().empty()) {
    throw std::invalid_argument("scene file: " + where + key + " must be a non-empty string");
  }
  return value.get<std::string>();
}

std::int64_t require_nanoseconds(
  const nlohmann::json & object, const std::string & key, const std::string & where)
{
  const auto & value = require_field(object, key, where);
  if (!value.is_number_integer()) {
    throw std::invalid_argument("scene file: " + where + key + " must be an integer (ns)");
  }
  return value.get<std::int64_t>();
}

rclcpp::Time to_time(const std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

}  // namespace

ClosedLoopScene parse_closed_loop_scene(const nlohmann::json & scene)
{
  const auto & version = require_field(scene, "schema_version", "");
  if (!version.is_number_integer() || version.get<int>() != kSchemaVersion) {
    throw std::invalid_argument(
      "scene file: schema_version must be " + std::to_string(kSchemaVersion));
  }

  ClosedLoopScene parsed;
  parsed.scene_id = require_string(scene, "scene_id");
  parsed.eval_label = require_string(scene, "eval_label");
  if (scene.contains("difficulty") && !scene.at("difficulty").is_null()) {
    parsed.difficulty = require_string(scene, "difficulty");
  }

  if (scene.contains("window_clock")) {
    const auto clock = require_string(scene, "window_clock");
    if (clock != "evaluated" && clock != "recorded") {
      throw std::invalid_argument("scene file: window_clock must be evaluated or recorded");
    }
    parsed.window_in_recorded_clock = clock == "recorded";
  }
  const auto & window = require_field(scene, "window", "");
  const auto start_ns = require_nanoseconds(window, "start_ns", "window.");
  const auto end_ns = require_nanoseconds(window, "end_ns", "window.");
  if (end_ns <= start_ns) {
    throw std::invalid_argument("scene file: window.end_ns must be after window.start_ns");
  }
  parsed.window_start = to_time(start_ns);
  parsed.window_end = to_time(end_ns);
  const auto run_end_ns =
    scene.contains("run_end_ns") ? require_nanoseconds(scene, "run_end_ns", "") : end_ns;
  if (run_end_ns < start_ns) {
    throw std::invalid_argument("scene file: run_end_ns must not be before window.start_ns");
  }
  parsed.run_end = to_time(run_end_ns);

  if (scene.contains("reference") && !scene.at("reference").is_null()) {
    const auto & reference = scene.at("reference");
    if (!reference.is_object()) {
      throw std::invalid_argument("scene file: reference must be an object");
    }
    if (reference.contains("bag_path") && !reference.at("bag_path").is_null()) {
      parsed.reference_bag_path = require_string(reference, "bag_path", "reference.");
    }
    if (reference.contains("time_offset_ns")) {
      const auto & offset = reference.at("time_offset_ns");
      if (offset.is_string() && offset.get<std::string>() == "auto") {
        parsed.reference_time_offset_auto = true;
      } else {
        parsed.reference_time_offset_ns =
          require_nanoseconds(reference, "time_offset_ns", "reference.");
      }
    }
    if (reference.contains("match_s")) {
      const auto & match = reference.at("match_s");
      if (!match.is_number() || !(match.get<double>() > 0.0)) {
        throw std::invalid_argument("scene file: reference.match_s must be a positive number");
      }
      parsed.reference_match_s = match.get<double>();
    }
  }
  if (scene.contains("margin") && !scene.at("margin").is_null()) {
    const auto & margin = scene.at("margin");
    if (!margin.is_object()) {
      throw std::invalid_argument("scene file: margin must be an object");
    }
    if (margin.contains("kind")) {
      parsed.margin_kind = require_string(margin, "kind", "margin.");
    }
    if (margin.contains("trailing_vehicle")) {
      if (!margin.at("trailing_vehicle").is_boolean()) {
        throw std::invalid_argument("scene file: margin.trailing_vehicle must be a boolean");
      }
      parsed.margin_trailing_vehicle = margin.at("trailing_vehicle").get<bool>();
    }
    if (margin.contains("every_road_user")) {
      if (!margin.at("every_road_user").is_boolean()) {
        throw std::invalid_argument("scene file: margin.every_road_user must be a boolean");
      }
      parsed.margin_every_road_user = margin.at("every_road_user").get<bool>();
    }
  }
  if (scene.contains("ignored_track_ids")) {
    const auto & ignored = scene.at("ignored_track_ids");
    if (!ignored.is_array()) {
      throw std::invalid_argument("scene file: ignored_track_ids must be a list of strings");
    }
    for (const auto & id : ignored) {
      if (!id.is_string()) {
        throw std::invalid_argument("scene file: ignored_track_ids must be a list of strings");
      }
      parsed.ignored_track_ids.push_back(id.get<std::string>());
    }
  }
  return parsed;
}

ClosedLoopScene load_closed_loop_scene(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open scene file " + path);
  }
  nlohmann::json scene;
  try {
    file >> scene;
  } catch (const nlohmann::json::exception & error) {
    throw std::runtime_error("scene file " + path + " is not JSON: " + error.what());
  }
  return parse_closed_loop_scene(scene);
}

nlohmann::json to_json(const ClosedLoopScene & scene)
{
  nlohmann::json json;
  json["schema_version"] = kSchemaVersion;
  json["scene_id"] = scene.scene_id;
  json["eval_label"] = scene.eval_label;
  json["difficulty"] =
    scene.difficulty.empty() ? nlohmann::json(nullptr) : nlohmann::json(scene.difficulty);
  json["window"] = {
    {"start_ns", scene.window_start.nanoseconds()}, {"end_ns", scene.window_end.nanoseconds()}};
  json["run_end_ns"] = scene.run_end.nanoseconds();
  json["window_clock"] = scene.window_in_recorded_clock ? "recorded" : "evaluated";
  json["reference"] = {
    {"time_offset_ns", scene.reference_time_offset_ns},
    {"time_offset_found", scene.reference_time_offset_auto},
    {"match_s", scene.reference_match_s}};
  if (!scene.reference_bag_path.empty()) {
    json["reference"]["bag_path"] = scene.reference_bag_path;
  }
  if (
    scene.margin_kind.has_value() || scene.margin_every_road_user.has_value() ||
    scene.margin_trailing_vehicle) {
    json["margin"] = {{"trailing_vehicle", scene.margin_trailing_vehicle}};
    if (scene.margin_kind.has_value()) {
      json["margin"]["kind"] = *scene.margin_kind;
    }
    if (scene.margin_every_road_user.has_value()) {
      json["margin"]["every_road_user"] = *scene.margin_every_road_user;
    }
  }
  if (!scene.ignored_track_ids.empty()) {
    json["ignored_track_ids"] = scene.ignored_track_ids;
  }
  return json;
}

}  // namespace autoware::planning_data_analyzer::closed_loop

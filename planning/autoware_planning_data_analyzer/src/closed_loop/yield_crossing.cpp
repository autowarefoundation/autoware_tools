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

// The yielding metric: did the ego yield in one executed closed-loop run? See README.md for the
// rules.

#include "closed_loop/yield_crossing.hpp"

#include "closed_loop/contacts.hpp"
#include "closed_loop/road_users.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

// Tolerance of the line comparison, against float noise in rows times the interval.
constexpr double kLineToleranceS = 1.0e-9;

std::string one_decimal(const double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", value);
  return buffer;
}

}  // namespace

YieldResult evaluate_yield(
  const ClosedLoopEpisode & episode, const std::vector<std::string> & target_track_ids,
  const std::vector<std::string> & ignored_track_ids, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const YieldParameters & parameters)
{
  if (episode.recorded_poses.size() < 2U) {
    throw std::invalid_argument("yield needs the recorded drive on at least two rows");
  }
  const double interval = episode.interval_s;
  const auto half_window =
    static_cast<std::size_t>(std::max<std::int64_t>(1, std::llround(0.5 / interval)));
  const std::set<std::string> ignored(ignored_track_ids.begin(), ignored_track_ids.end());
  const std::set<std::string> targets(target_track_ids.begin(), target_track_ids.end());
  const std::set<std::string> short_tracks(
    episode.short_track_ids.begin(), episode.short_track_ids.end());

  // Contacts: the first one of every road user that counts, target or not.
  const auto kept = without_road_users(episode, ignored_track_ids);
  const auto road = road_user_columns(kept, half_window);
  const auto collisions = episode_contacts(kept, layout, vehicle_info, parameters.rear_heading_deg);
  std::map<std::string, const metrics::NoAtFaultCollisionContact *> contact_of;
  for (const auto & contact : collisions.contacts) {
    contact_of.emplace(contact.object_id, &contact);
  }
  const auto type_of = [&](const std::string & track_id) {
    const auto column = road.column(track_id);
    return column < road.columns() ? to_string(road.types.at(column)) : std::string{};
  };

  YieldResult result;
  // Targets: every one that can be judged, measured in one call.
  std::vector<std::pair<std::size_t, std::size_t>> measured;  // (record, column)
  for (const auto & track_id : target_track_ids) {
    YieldTargetRecord record;
    record.track_id = track_id;
    record.type = type_of(track_id);
    const auto column = road.column(track_id);
    const auto contact = contact_of.find(track_id);
    if (ignored.count(track_id) != 0U) {
      record.unjudged = "never counts (ignored)";
    } else if (short_tracks.count(track_id) != 0U) {
      record.unjudged = "observed for less than " + one_decimal(parameters.min_track_s) + " s";
    } else if (column >= road.columns()) {
      record.unjudged = "not observed in the episode";
    } else if (contact != contact_of.end() && contact->second->at_fault) {
      record.unjudged = "the ego hit it, at fault";
    } else {
      measured.emplace_back(result.targets.size(), column);
    }
    result.targets.push_back(record);
  }
  if (!measured.empty()) {
    const auto rows = kept.executed_rows();
    std::vector<std::vector<BoxCorners>> corners(rows);
    std::vector<std::vector<char>> valid(rows);
    std::vector<std::vector<double>> yaw(rows);
    for (std::size_t row = 0; row < rows; ++row) {
      for (const auto & [_, column] : measured) {
        corners.at(row).push_back(road.corners.at(row).at(column));
        valid.at(row).push_back(road.valid.at(row).at(column));
        yaw.at(row).push_back(road.yaw.at(row).at(column));
      }
    }
    std::vector<double> ego_yaw;
    for (const auto & point : kept.ego.points) {
      ego_yaw.push_back(metrics::get_yaw(point.pose.orientation));
    }
    const auto & origin_pose = kept.ego.points.front().pose.position;
    const auto crossings = measure_crossings(
      ego_boxes(kept, vehicle_info), ego_yaw, corners, valid, yaw,
      Point2d{origin_pose.x, origin_pose.y}, parameters.grid);
    for (std::size_t i = 0; i < measured.size(); ++i) {
      auto & record = result.targets.at(measured.at(i).first);
      const auto & crossing = crossings.at(i);
      record.crossing = crossing;
      const auto line = parameters.pet_line_s.find(record.type);
      if (crossing.order == CrossingOrder::NONE) {
        record.unjudged = "never shared a cell with the ego";
      } else if (crossing.order == CrossingOrder::NO_CROSSING) {
        record.unjudged = "shared cells without a crossing";
      } else if (crossing.order == CrossingOrder::ROAD_USER_FIRST) {
        record.measure = "pet";
        record.value_s = static_cast<double>(crossing.pet_rows()) * interval;
        if (line == parameters.pet_line_s.end()) {
          record.unjudged = "no PET line for its type";
        } else {
          record.line_s = line->second;
          record.passed = *record.value_s >= line->second - kLineToleranceS;
        }
      } else {
        record.measure = "entry_gap";
        record.value_s = static_cast<double>(crossing.entry_gap_rows()) * interval;
        record.line_s = parameters.entry_gap_line_s;
        const double exit_speed =
          kept.ego.points.at(static_cast<std::size_t>(std::max<std::int64_t>(0, crossing.exit_row)))
            .speed_mps;
        if (!(exit_speed >= parameters.ego_stopped_mps)) {
          record.unjudged = "the ego went first while nearly stopped";
        } else {
          record.passed = *record.value_s >= parameters.entry_gap_line_s - kLineToleranceS;
        }
      }
    }
  }

  // The route: the ego stays near the recorded path while that path runs alongside it.
  LineString2d path;
  for (std::size_t row = 0; row < std::min(episode.window_rows, episode.recorded_poses.size());
       ++row) {
    const auto & position = episode.recorded_poses.at(row).position;
    path.emplace_back(position.x, position.y);
  }
  const double path_length = boost::geometry::length(path);
  if (path.size() >= 2U && path_length > 0.0) {
    for (std::size_t row = 0; row < kept.executed_rows(); ++row) {
      const auto & position = kept.ego.points.at(row).pose.position;
      const Point2d point{position.x, position.y};
      if (path_progress(point, path) >= path_length - parameters.route_end_margin_m) {
        continue;
      }
      const double lateral = distance_to_path(point, path);
      result.max_route_deviation_m = std::max(result.max_route_deviation_m, lateral);
      if (lateral > parameters.route_deviation_m && !result.left_route_row.has_value()) {
        result.left_route_row = row;
      }
    }
  }

  // Contacts and the verdict.
  std::vector<std::tuple<std::size_t, std::string, std::string>> at_fault;
  for (const auto & contact : collisions.contacts) {
    YieldContactRecord record;
    record.track_id = contact.object_id;
    record.type = type_of(contact.object_id);
    record.row = contact.point_index;
    record.kind = contact_text(contact);
    record.at_fault = contact.at_fault;
    record.target = targets.count(contact.object_id) != 0U;
    result.contacts.push_back(record);
    if (contact.at_fault) {
      at_fault.emplace_back(record.row, record.track_id, record.kind);
    }
  }
  std::sort(at_fault.begin(), at_fault.end());
  for (const auto & [row, track_id, kind] : at_fault) {
    result.fail_reasons.push_back(
      "collision with " + track_id + " (" + type_of(track_id) + ") at " +
      one_decimal(static_cast<double>(row) * interval) + " s: the ego's fault (" + kind + ")");
  }
  for (const auto & target : result.targets) {
    if (!target.passed.has_value() || *target.passed) {
      continue;
    }
    if (target.measure == "pet") {
      result.fail_reasons.push_back(
        "PET " + one_decimal(*target.value_s) + " s < " + one_decimal(*target.line_s) +
        " s: " + target.track_id + " (" + target.type + ") went first");
    } else {
      result.fail_reasons.push_back(
        "entry gap " + one_decimal(*target.value_s) + " s < " + one_decimal(*target.line_s) +
        " s: the ego went first ahead of " + target.track_id + " (" + target.type + ")");
    }
  }
  if (result.left_route_row.has_value()) {
    result.fail_reasons.push_back(
      "left the recorded route at " +
      one_decimal(static_cast<double>(*result.left_route_row) * interval) + " s (up to " +
      one_decimal(result.max_route_deviation_m) + " m from it)");
  }
  result.passed = result.fail_reasons.empty();
  return result;
}

nlohmann::json to_json(const YieldResult & result)
{
  const auto optional_value = [](const std::optional<double> & value) {
    return value.has_value() ? nlohmann::json(*value) : nlohmann::json(nullptr);
  };
  nlohmann::json targets = nlohmann::json::array();
  for (const auto & target : result.targets) {
    nlohmann::json record = {
      {"track_id", target.track_id},
      {"type", target.type.empty() ? nlohmann::json(nullptr) : nlohmann::json(target.type)},
      {"measure",
       target.measure.empty() ? nlohmann::json(nullptr) : nlohmann::json(target.measure)},
      {"value_s", optional_value(target.value_s)},
      {"line_s", optional_value(target.line_s)},
      {"passed",
       target.passed.has_value() ? nlohmann::json(*target.passed) : nlohmann::json(nullptr)},
      {"unjudged",
       target.unjudged.empty() ? nlohmann::json(nullptr) : nlohmann::json(target.unjudged)},
    };
    if (target.crossing.has_value()) {
      const auto & crossing = *target.crossing;
      const bool ordered = crossing.ordered();
      record["order"] = to_string(crossing.order);
      record["shared_cells"] = crossing.shared_cells;
      record["zone_cells"] = crossing.zone_cells;
      record["exit_row"] = ordered ? nlohmann::json(crossing.exit_row) : nlohmann::json(nullptr);
      record["entry_row"] = ordered ? nlohmann::json(crossing.entry_row) : nlohmann::json(nullptr);
      record["pet_rows"] = ordered ? nlohmann::json(crossing.pet_rows()) : nlohmann::json(nullptr);
      record["pet_cell_xy"] = ordered ? nlohmann::json{crossing.pet_cell.x(), crossing.pet_cell.y()}
                                      : nlohmann::json(nullptr);
      record["ego_zone_rows"] =
        ordered ? nlohmann::json(crossing.ego_zone_rows) : nlohmann::json(nullptr);
      record["road_user_zone_rows"] =
        ordered ? nlohmann::json(crossing.road_user_zone_rows) : nlohmann::json(nullptr);
    }
    targets.push_back(record);
  }
  nlohmann::json contacts = nlohmann::json::array();
  for (const auto & contact : result.contacts) {
    contacts.push_back(
      {{"track_id", contact.track_id},
       {"type", contact.type},
       {"row", contact.row},
       {"kind", contact.kind},
       {"at_fault", contact.at_fault},
       {"target", contact.target}});
  }
  return {
    {"passed", result.passed},
    {"targets", targets},
    {"contacts", contacts},
    {"route",
     {{"left_row", result.left_route_row.has_value() ? nlohmann::json(*result.left_route_row)
                                                     : nlohmann::json(nullptr)},
      {"max_deviation_m", result.max_route_deviation_m}}},
    {"fail_reasons", result.fail_reasons},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

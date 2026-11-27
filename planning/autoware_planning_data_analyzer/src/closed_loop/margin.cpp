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

// The margin metric: did the ego keep clear of the road users that count, in one executed
// closed-loop run? See README.md for the rules.

#include "closed_loop/margin.hpp"

#include "closed_loop/contacts.hpp"
#include "closed_loop/road_users.hpp"
#include "metrics/epdms/subscores/no_at_fault_collision.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

template <typename T>
using Grid = std::vector<std::vector<T>>;

// torch.nanmedian: the lower of the two middle values; NaN without values.
double lower_median(std::vector<double> values)
{
  if (values.empty()) {
    return kNaN;
  }
  std::sort(values.begin(), values.end());
  return values.at((values.size() - 1U) / 2U);
}

// torch.nanquantile with linear interpolation; NaN without values.
double quantile(std::vector<double> values, const double q)
{
  if (values.empty()) {
    return kNaN;
  }
  std::sort(values.begin(), values.end());
  const double position = q * static_cast<double>(values.size() - 1U);
  const auto low = static_cast<std::size_t>(std::floor(position));
  const auto high = static_cast<std::size_t>(std::ceil(position));
  return values.at(low) +
         (position - static_cast<double>(low)) * (values.at(high) - values.at(low));
}

double wrap(const double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

std::string seconds_text(const double seconds, const int digits)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(digits) << seconds;
  return text.str();
}

struct Measured
{
  RoadUserColumns road;
  std::vector<BoxCorners> ego;
  Grid<double> gaps;
  std::vector<char> in_window;
  std::vector<char> out_of_lane;
  std::vector<char> same_way;
  Grid<char> on_road;
  std::vector<char> candidate;
  std::vector<geometry_msgs::msg::Pose> recorded;
};

// The role in which each road user's distance counts on each row.
Grid<MarginRole> roles_of(
  const Measured & m, const MarginReference & reference, const MarginParameters & parameters)
{
  const auto rows = m.road.rows();
  const auto columns = m.road.columns();
  std::vector<char> counted(columns, 0);
  for (std::size_t column = 0; column < columns; ++column) {
    bool on_map = false;
    for (std::size_t row = 0; row < rows; ++row) {
      on_map = on_map || m.on_road.at(row).at(column) != 0;
    }
    counted.at(column) = m.candidate.at(column) != 0 && on_map;
  }
  Grid<MarginRole> roles(rows, std::vector<MarginRole>(columns, MarginRole::NOT_COUNTED));
  if (reference.every_road_user || reference.kind != "avoidance") {
    // An easy scene: every row; a lane change or a pull-out: while out of the start lane.
    for (std::size_t row = 0; row < rows; ++row) {
      if (!reference.every_road_user && m.out_of_lane.at(row) == 0) {
        continue;
      }
      for (std::size_t column = 0; column < columns; ++column) {
        if (counted.at(column) != 0) {
          roles.at(row).at(column) = MarginRole::ROAD_USER;
        }
      }
    }
    return roles;
  }

  // Avoidance: roles read on the recorded drive, on the window's rows.
  const auto recorded_rows = std::min(m.recorded.size(), rows);
  std::vector<MarginRole> role(
    columns, reference.trailing_vehicle ? MarginRole::ROAD_USER : MarginRole::NOT_COUNTED);
  std::vector<char> oncoming(columns, 0);
  for (std::size_t column = 0; column < columns; ++column) {
    std::vector<double> speeds;
    for (std::size_t row = 0; row < rows; ++row) {
      if (
        m.road.valid.at(row).at(column) != 0 && m.in_window.at(row) != 0 &&
        m.road.fitted.at(row).at(column) != 0) {
        speeds.push_back(m.road.speed.at(row).at(column));
      }
    }
    const double median_speed = lower_median(speeds);
    const double fast_speed = quantile(speeds, 0.9);
    std::vector<double> near_turns;
    std::vector<double> seen_turns;
    for (std::size_t row = 0; row < recorded_rows; ++row) {
      if (m.road.valid.at(row).at(column) == 0 || m.in_window.at(row) == 0) {
        continue;
      }
      const auto & pose = m.recorded.at(row);
      const double off_deg =
        std::abs(wrap(m.road.yaw.at(row).at(column) - metrics::get_yaw(pose.orientation))) * 180.0 /
        M_PI;
      seen_turns.push_back(off_deg);
      const auto & xy = m.road.xy.at(row).at(column);
      if (
        std::hypot(xy.x() - pose.position.x, xy.y() - pose.position.y) <=
        parameters.oncoming_range_m) {
        near_turns.push_back(off_deg);
      }
    }
    const double heading_off = lower_median(near_turns.empty() ? seen_turns : near_turns);
    const bool against = heading_off > parameters.oncoming_deg;
    const bool vehicle = m.road.types.at(column) == RoadUserType::VEHICLE;
    const bool standing = vehicle && median_speed < parameters.standing_mps;
    const bool moving = vehicle && median_speed >= parameters.standing_mps;
    if (!vehicle && against && fast_speed >= parameters.vru_moving_mps) {
      role.at(column) = MarginRole::ONCOMING_VRU;
    }
    if (moving && against) {
      role.at(column) = MarginRole::ONCOMING;
    }
    if (standing) {
      role.at(column) = MarginRole::AVOIDED_OBJECT;
    }
    if (counted.at(column) == 0) {
      role.at(column) = MarginRole::NOT_COUNTED;
    }
    oncoming.at(column) =
      role.at(column) == MarginRole::ONCOMING || role.at(column) == MarginRole::ONCOMING_VRU;
  }
  for (std::size_t row = 0; row < rows; ++row) {
    const bool across = m.same_way.at(row) == 0;
    for (std::size_t column = 0; column < columns; ++column) {
      if (m.out_of_lane.at(row) == 0 || (oncoming.at(column) != 0 && !across)) {
        continue;
      }
      auto value = role.at(column);
      const bool fitted = m.road.fitted.at(row).at(column) != 0;
      if (
        value == MarginRole::ONCOMING && fitted &&
        m.road.speed.at(row).at(column) < parameters.standing_mps) {
        value = MarginRole::AVOIDED_OBJECT;
      }
      roles.at(row).at(column) = value;
    }
  }
  return roles;
}

}  // namespace

std::string to_string(const MarginRole role)
{
  switch (role) {
    case MarginRole::ROAD_USER:
      return "road_user";
    case MarginRole::AVOIDED_OBJECT:
      return "avoided_object";
    case MarginRole::ONCOMING:
      return "oncoming";
    case MarginRole::ONCOMING_VRU:
      return "oncoming_vru";
    case MarginRole::NOT_COUNTED:
    default:
      return "not_counted";
  }
}

MarginResult evaluate_margin(
  const ClosedLoopEpisode & episode, const MarginReference & reference, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const MarginParameters & parameters)
{
  if (
    reference.kind != "lane_change" && reference.kind != "avoidance" &&
    reference.kind != "pull_out") {
    throw std::invalid_argument("margin: unknown kind " + reference.kind);
  }
  if (reference.trailing_vehicle && reference.kind != "avoidance") {
    throw std::invalid_argument("margin: trailing_vehicle applies to avoidance only");
  }
  if (episode.recorded_poses.size() < 2U) {
    throw std::invalid_argument("margin needs the recorded drive on at least two rows");
  }
  const double interval = episode.interval_s;
  const auto cut_rows = static_cast<std::size_t>(std::llround(parameters.cut_window_s / interval));
  const auto half_window =
    static_cast<std::size_t>(std::llround(parameters.speed_half_window_s / interval));
  if (cut_rows < 1U || half_window < 1U) {
    throw std::invalid_argument("margin: cut_window_s and speed_half_window_s must cover a row");
  }

  // Road users kept: not ignored (blips are dropped when the episode is built).
  const auto kept = without_road_users(episode, reference.ignored_track_ids);

  Measured m;
  m.road = road_user_columns(kept, half_window);
  m.ego = ego_boxes(kept, vehicle_info);
  m.recorded = kept.recorded_poses;
  const auto rows = kept.executed_rows();
  const auto columns = m.road.columns();
  const auto window = std::min(kept.window_rows, rows);

  // Contacts until the run ends; straddling lanes on the recorded path is the human's way too.
  LineString2d recorded_path;
  for (const auto & pose : m.recorded) {
    recorded_path.emplace_back(pose.position.x, pose.position.y);
  }
  std::vector<bool> on_path(rows, false);
  for (std::size_t row = 0; row < rows; ++row) {
    const auto & position = kept.ego.points.at(row).pose.position;
    on_path.at(row) =
      distance_to_path(Point2d{position.x, position.y}, recorded_path) <= parameters.on_path_m;
  }
  const auto collisions =
    episode_contacts(kept, layout, vehicle_info, parameters.rear_heading_deg, on_path);

  m.gaps.assign(rows, std::vector<double>(columns, kInf));
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t column = 0; column < columns; ++column) {
      if (m.road.valid.at(row).at(column) != 0) {
        m.gaps.at(row).at(column) = box_clearance(m.ego.at(row), m.road.corners.at(row).at(column));
      }
    }
  }

  // The first contact of each road user, and the cut: a road user that drives deep into the ego
  // after a contact that is not its fault stops the judging at that contact's row.
  std::vector<const metrics::NoAtFaultCollisionContact *> contact_of(columns, nullptr);
  for (const auto & contact : collisions.contacts) {
    const auto column = m.road.column(contact.object_id);
    if (column < columns && contact_of.at(column) == nullptr) {
      contact_of.at(column) = &contact;
    }
  }
  MarginResult result;
  for (std::size_t column = 0; column < columns; ++column) {
    const auto * contact = contact_of.at(column);
    if (contact == nullptr || contact->at_fault) {
      continue;
    }
    double depth = -kInf;
    for (std::size_t row = contact->point_index;
         row < std::min(rows, contact->point_index + cut_rows); ++row) {
      depth = std::max(depth, -m.gaps.at(row).at(column));
    }
    if (depth > parameters.cut_depth_m) {
      result.judged_until_row =
        std::min(result.judged_until_row.value_or(contact->point_index), contact->point_index);
    }
  }
  const auto judged_until = result.judged_until_row.value_or(rows);

  // Who can count, and when.
  StartLaneOptions start_options;
  start_options.reach_m = parameters.start_lane_reach_m;
  start_options.tolerance_m = parameters.lane_tolerance_m;
  const auto lane = layout.start_lane(m.ego.front(), recorded_path, start_options);
  result.start_lane = lane.has_value();
  m.out_of_lane.assign(rows, 1);
  if (lane.has_value()) {
    for (std::size_t row = 0; row < rows; ++row) {
      m.out_of_lane.at(row) = !LaneLayout::inside(m.ego.at(row), *lane);
    }
  }
  m.same_way.assign(rows, 0);
  if (reference.kind == "avoidance" && !reference.every_road_user) {
    for (std::size_t row = 0; row < rows; ++row) {
      m.same_way.at(row) = layout.same_way(
        m.ego.at(row), metrics::get_yaw(kept.ego.points.at(row).pose.orientation),
        parameters.lane_tolerance_m);
    }
  }
  m.on_road.assign(rows, std::vector<char>(columns, 0));
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t column = 0; column < columns; ++column) {
      if (m.road.valid.at(row).at(column) != 0) {
        m.on_road.at(row).at(column) = layout.on_road(m.road.corners.at(row).at(column));
      }
    }
  }
  m.in_window.assign(rows, 0);
  for (std::size_t row = 0; row < window; ++row) {
    m.in_window.at(row) = 1;
  }
  const auto compared_rows = std::min(m.recorded.size(), rows);
  m.candidate.assign(columns, 0);
  for (std::size_t column = 0; column < columns; ++column) {
    const double reach = m.road.types.at(column) == RoadUserType::VEHICLE
                           ? parameters.candidate_vehicle_m
                           : parameters.candidate_vru_m;
    double nearest = kInf;
    for (std::size_t row = 0; row < window; ++row) {
      nearest = std::min(nearest, m.gaps.at(row).at(column));
      if (row < compared_rows && m.road.valid.at(row).at(column) != 0) {
        nearest = std::min(
          nearest, box_clearance(
                     ego_box(m.recorded.at(row), vehicle_info), m.road.corners.at(row).at(column)));
      }
    }
    m.candidate.at(column) = nearest <= reach;
  }
  const auto roles = roles_of(m, reference, parameters);

  // Where each road user counts: in the window, while the ego moves, on the road, before the cut,
  // and never for a road user whose contact was not the ego's fault.
  const std::vector<MarginRole> lined{
    MarginRole::ROAD_USER, MarginRole::AVOIDED_OBJECT, MarginRole::ONCOMING,
    MarginRole::ONCOMING_VRU};
  std::vector<char> judged_distance(columns, 0);
  for (const auto role : lined) {
    const auto line = parameters.line_m.find(to_string(role));
    if (line == parameters.line_m.end()) {
      continue;
    }
    for (std::size_t column = 0; column < columns; ++column) {
      const auto * contact = contact_of.at(column);
      if (contact != nullptr && !contact->at_fault) {
        continue;
      }
      double closest = kInf;
      std::size_t closest_row = 0;
      for (std::size_t row = 0; row < std::min(window, judged_until); ++row) {
        const bool counts = m.road.valid.at(row).at(column) != 0 &&
                            m.on_road.at(row).at(column) != 0 &&
                            kept.ego.points.at(row).speed_mps >= parameters.moving_mps &&
                            roles.at(row).at(column) == role;
        if (counts && m.gaps.at(row).at(column) < closest) {
          closest = m.gaps.at(row).at(column);
          closest_row = row;
        }
      }
      if (!std::isfinite(closest)) {
        continue;
      }
      MarginRecord record;
      record.track_id = m.road.track_ids.at(column);
      record.type = to_string(m.road.types.at(column));
      record.role = to_string(role);
      // An overlap reads 0 m: a collision is judged by the contact rule, not by the line.
      record.distance_m = std::max(closest, 0.0);
      record.row = closest_row;
      record.line_m = line->second;
      record.passed = record.distance_m >= record.line_m;
      result.road_users.push_back(record);
      judged_distance.at(column) = 1;
    }
  }
  std::sort(
    result.road_users.begin(), result.road_users.end(), [](const auto & lhs, const auto & rhs) {
      return std::tie(lhs.row, lhs.track_id, lhs.role) < std::tie(rhs.row, rhs.track_id, rhs.role);
    });

  std::vector<MarginContactRecord> collisions_judged;
  for (std::size_t column = 0; column < columns; ++column) {
    const auto * contact = contact_of.at(column);
    if (contact == nullptr) {
      continue;
    }
    MarginContactRecord record;
    record.track_id = m.road.track_ids.at(column);
    record.type = to_string(m.road.types.at(column));
    record.counted = judged_distance.at(column) != 0;
    record.row = contact->point_index;
    record.kind = contact_text(*contact);
    record.at_fault = contact->at_fault;
    record.judged = contact->at_fault ? contact->point_index <= judged_until : true;
    result.contacts.push_back(record);
    if (record.at_fault && record.judged) {
      collisions_judged.push_back(record);
    }
  }
  const auto by_row = [](const auto & lhs, const auto & rhs) {
    return std::tie(lhs.row, lhs.track_id) < std::tie(rhs.row, rhs.track_id);
  };
  std::sort(result.contacts.begin(), result.contacts.end(), by_row);
  std::sort(collisions_judged.begin(), collisions_judged.end(), by_row);
  for (const auto & collision : collisions_judged) {
    result.fail_reasons.push_back(
      "collision with " + collision.track_id + " (" + collision.type + ") at " +
      seconds_text(static_cast<double>(collision.row) * interval, 1) + " s: the ego's fault (" +
      collision.kind + ")");
  }
  std::set<std::string> counted_ids;
  for (const auto & record : result.road_users) {
    counted_ids.insert(record.track_id);
    if (!result.min_distance_m.has_value() || record.distance_m < *result.min_distance_m) {
      result.min_distance_m = record.distance_m;
    }
    if (!record.passed) {
      ++result.margin_failure_count;
      result.fail_reasons.push_back(
        "distance " + seconds_text(record.distance_m, 2) + " m < " +
        seconds_text(record.line_m, 2) + " m to " + record.track_id + " (" + record.type + ", " +
        record.role + ") at " + seconds_text(static_cast<double>(record.row) * interval, 1) + " s");
    }
  }
  result.at_fault_collision_count = collisions_judged.size();
  result.counted_road_user_count = counted_ids.size();
  result.passed = result.fail_reasons.empty();
  return result;
}

nlohmann::json to_json(const MarginResult & result, const MarginReference & reference)
{
  nlohmann::json road_users = nlohmann::json::array();
  for (const auto & record : result.road_users) {
    road_users.push_back(
      {{"track_id", record.track_id},
       {"type", record.type},
       {"role", record.role},
       {"distance_m", record.distance_m},
       {"row", record.row},
       {"line_m", record.line_m},
       {"passed", record.passed}});
  }
  nlohmann::json contacts = nlohmann::json::array();
  for (const auto & record : result.contacts) {
    contacts.push_back(
      {{"track_id", record.track_id},
       {"type", record.type},
       {"counted", record.counted},
       {"row", record.row},
       {"kind", record.kind},
       {"at_fault", record.at_fault},
       {"judged", record.judged}});
  }
  return {
    {"kind", reference.kind},
    {"trailing_vehicle", reference.trailing_vehicle},
    {"every_road_user", reference.every_road_user},
    {"passed", result.passed},
    {"start_lane", result.start_lane},
    {"judged_until_row", result.judged_until_row.has_value()
                           ? nlohmann::json(*result.judged_until_row)
                           : nlohmann::json(nullptr)},
    {"at_fault_collision_count", result.at_fault_collision_count},
    {"margin_failure_count", result.margin_failure_count},
    {"counted_road_user_count", result.counted_road_user_count},
    {"min_distance_m", result.min_distance_m.has_value() ? nlohmann::json(*result.min_distance_m)
                                                         : nlohmann::json(nullptr)},
    {"road_users", road_users},
    {"contacts", contacts},
    {"fail_reasons", result.fail_reasons},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

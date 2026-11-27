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

// The lane goal: does an easy lane change or avoidance finish in its lane inside the window? See
// README.md for the rules.

#include "closed_loop/lane_goal.hpp"

#include "closed_loop/road_users.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

std::size_t rows_of(const double seconds, const double interval)
{
  return static_cast<std::size_t>(std::max<std::int64_t>(1, std::llround(seconds / interval)));
}

std::optional<std::size_t> first_of(
  const std::vector<char> & mask, const std::size_t start, const std::size_t stop)
{
  for (std::size_t row = start; row < std::min(stop, mask.size()); ++row) {
    if (mask.at(row) != 0) {
      return row;
    }
  }
  return std::nullopt;
}

void check_rows(
  const std::vector<char> & executed, const std::vector<char> & recorded, const std::size_t window,
  const std::size_t hold_rows, const std::size_t start_rows)
{
  if (window < 1U || hold_rows < 1U || start_rows < 1U) {
    throw std::invalid_argument("window, hold_rows and start_rows must be at least 1 row");
  }
  if (recorded.size() < window) {
    throw std::invalid_argument("the recorded rows must cover the window");
  }
  if (executed.empty()) {
    throw std::invalid_argument("the run needs at least one row");
  }
}

std::string format(const char * pattern, const double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), pattern, value);
  return buffer;
}

std::string percent(const double share)
{
  return format("%.0f%%", share * 100.0);
}

double median_lower(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  return values.at((values.size() - 1U) / 2U);
}

BoxCorners wheels_of(
  const geometry_msgs::msg::Pose & pose, const double wheel_base_m, const double wheel_tread_m)
{
  return box_corners(
    pose.position.x, pose.position.y, metrics::get_yaw(pose.orientation), wheel_base_m, 0.0,
    wheel_tread_m / 2.0, wheel_tread_m / 2.0);
}

double rear_progress(
  const geometry_msgs::msg::Pose & pose, const LineString2d & path, const double rear_m)
{
  const double yaw = metrics::get_yaw(pose.orientation);
  return path_progress(
    Point2d{pose.position.x - rear_m * std::cos(yaw), pose.position.y - rear_m * std::sin(yaw)},
    path);
}

std::optional<AvoidedVehicle> avoided_vehicle(
  const ClosedLoopEpisode & episode, const LineString2d & path, const std::size_t window,
  const LaneGoalParameters & parameters, const std::vector<std::string> & ignored_track_ids,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto & recorded = episode.recorded_poses;
  if (std::all_of(path.begin(), path.end(), [&](const Point2d & point) {
        return point.x() == path.front().x() && point.y() == path.front().y();
      })) {
    return std::nullopt;
  }
  const auto half = rows_of(parameters.speed_half_window_s, episode.interval_s);
  const auto road = road_user_columns(episode, half);
  const auto rows = std::min({window, road.rows(), recorded.size()});
  const auto recorded_rows = std::min(window, recorded.size());
  if (road.columns() == 0U || rows < 1U) {
    return std::nullopt;
  }
  const double rear_m = -vehicle_info.min_longitudinal_offset_m;
  std::vector<double> rear(recorded_rows);
  for (std::size_t row = 0; row < recorded_rows; ++row) {
    rear.at(row) = rear_progress(recorded.at(row), path, rear_m);
  }
  const std::set<std::string> ignored(ignored_track_ids.begin(), ignored_track_ids.end());
  std::optional<AvoidedVehicle> chosen;
  std::size_t candidates = 0;
  for (std::size_t column = 0; column < road.columns(); ++column) {
    std::vector<double> speeds;
    double near = std::numeric_limits<double>::infinity();
    double far = -std::numeric_limits<double>::infinity();
    bool any = false;
    for (std::size_t row = 0; row < rows; ++row) {
      if (road.valid.at(row).at(column) == 0) {
        continue;
      }
      any = true;
      if (road.fitted.at(row).at(column) != 0) {
        speeds.push_back(road.speed.at(row).at(column));
      }
      const auto & corners = road.corners.at(row).at(column);
      near = std::min(near, box_clearance(ego_box(recorded.at(row), vehicle_info), corners));
      for (const auto & corner : corners) {
        far = std::max(far, path_progress(corner, path));
      }
    }
    if (!any) {
      continue;
    }
    const bool standing = road.types.at(column) == RoadUserType::VEHICLE &&
                          median_lower(speeds) < parameters.standing_mps &&
                          ignored.count(road.track_ids.at(column)) == 0U;
    const bool ahead = !rear.empty() && far > rear.front();
    const bool passed =
      std::any_of(rear.begin(), rear.end(), [&](const double value) { return value >= far; });
    if (!(standing && near <= parameters.avoided_reach_m && ahead && passed)) {
      continue;
    }
    ++candidates;
    if (!chosen.has_value() || far > chosen->far_m) {
      chosen = AvoidedVehicle{road.track_ids.at(column), far, near, 0};
    }
  }
  if (chosen.has_value()) {
    chosen->candidates = candidates;
  }
  return chosen;
}

}  // namespace

std::optional<std::pair<double, double>> EndRule::measure(
  const std::vector<double> & shares, const std::size_t window) const
{
  if (window < 1U || shares.size() < window || window - 1U < rise_rows) {
    return std::nullopt;
  }
  const auto last = window - 1U;
  return std::make_pair(shares.at(last), shares.at(last) - shares.at(last - rise_rows));
}

bool EndRule::holds(const std::optional<std::pair<double, double>> & measured) const
{
  return measured.has_value() && measured->first >= share && measured->second >= rise;
}

std::optional<std::size_t> first_hold(
  const std::vector<char> & inside, const std::size_t start, std::size_t stop,
  const std::size_t hold_rows)
{
  if (hold_rows < 1U) {
    throw std::invalid_argument("hold_rows must be at least 1");
  }
  if (inside.size() < hold_rows) {
    return std::nullopt;
  }
  stop = std::min(stop, inside.size() - hold_rows + 1U);
  for (std::size_t row = start; row < stop; ++row) {
    if (std::all_of(inside.begin() + row, inside.begin() + row + hold_rows, [](const char value) {
          return value != 0;
        })) {
      return row;
    }
  }
  return std::nullopt;
}

std::optional<std::size_t> choose_lane(
  const std::vector<std::vector<char>> & held, const double share)
{
  if (held.empty() || held.front().empty()) {
    return std::nullopt;
  }
  std::size_t best = 0;
  std::size_t best_count = 0;
  for (std::size_t index = 0; index < held.size(); ++index) {
    const auto count =
      static_cast<std::size_t>(std::count(held.at(index).begin(), held.at(index).end(), 1));
    if (index == 0U || count > best_count) {
      best = index;
      best_count = count;
    }
  }
  return static_cast<double>(best_count) > share * static_cast<double>(held.front().size())
           ? std::optional<std::size_t>(best)
           : std::nullopt;
}

GoalVerdict lane_change_goal(
  const std::vector<char> & executed_inside, const std::vector<char> & recorded_inside,
  const std::size_t window, const std::size_t hold_rows, const std::size_t start_rows,
  const EndRule & end, const std::vector<double> & executed_share,
  const std::vector<double> & recorded_share)
{
  check_rows(executed_inside, recorded_inside, window, hold_rows, start_rows);
  GoalVerdict verdict;
  const auto start = std::min(start_rows, window);
  if (std::all_of(recorded_inside.begin(), recorded_inside.begin() + start, [](const char value) {
        return value != 0;
      })) {
    verdict.undecided = "recorded_in_target_lane";
    return verdict;
  }
  auto entered = first_hold(recorded_inside, 0, window, hold_rows);
  verdict.recorded_moving_in =
    !entered.has_value() && end.holds(end.measure(recorded_share, window));
  if (verdict.recorded_moving_in) {
    entered = window - 1U;
  }
  if (!entered.has_value()) {
    verdict.undecided = "recorded_not_settled";
    return verdict;
  }
  verdict.recorded_goal_row = entered;
  const auto measured = end.measure(executed_share, window);
  if (measured.has_value()) {
    verdict.end_share = measured->first;
    verdict.end_rise = measured->second;
  }
  const auto goal =
    first_hold(executed_inside, 0, std::min(window, executed_inside.size()), hold_rows);
  if (goal.has_value()) {
    verdict.goal_row = goal;
  } else if (executed_inside.size() < window) {
    verdict.undecided = "run_ended";
  } else if (end.holds(measured)) {
    verdict.goal_row = window - 1U;
    verdict.moving_in = true;
  } else {
    verdict.failure = "not_in_target_lane";
  }
  return verdict;
}

GoalVerdict avoidance_goal(
  const std::vector<char> & executed_inside, const std::vector<char> & recorded_inside,
  const std::vector<double> & executed_progress, const std::vector<double> & recorded_progress,
  const double past_m, const std::size_t window, const std::size_t hold_rows, const EndRule & end,
  const std::vector<double> & executed_share, const std::vector<double> & recorded_share)
{
  check_rows(executed_inside, recorded_inside, window, hold_rows, 1U);
  if (
    executed_progress.size() != executed_inside.size() ||
    recorded_progress.size() != recorded_inside.size()) {
    throw std::invalid_argument("each progress must have the rows of its inside");
  }
  GoalVerdict verdict;
  verdict.past_m = past_m;
  std::vector<char> recorded_past_mask(recorded_progress.size());
  for (std::size_t row = 0; row < recorded_progress.size(); ++row) {
    recorded_past_mask.at(row) = recorded_progress.at(row) >= past_m;
  }
  const auto recorded_past = first_of(recorded_past_mask, 0, window);
  if (!recorded_past.has_value()) {
    verdict.undecided = "no_avoided_vehicle";
    return verdict;
  }
  auto back = first_hold(recorded_inside, *recorded_past, window, hold_rows);
  verdict.recorded_moving_in = !back.has_value() && end.holds(end.measure(recorded_share, window));
  if (verdict.recorded_moving_in) {
    back = window - 1U;
  }
  if (!back.has_value()) {
    verdict.undecided = "recorded_not_back";
    verdict.recorded_progress_row = recorded_past;
    return verdict;
  }
  const auto rows = std::min(window, executed_inside.size());
  std::vector<char> past_mask(executed_progress.size());
  for (std::size_t row = 0; row < executed_progress.size(); ++row) {
    past_mask.at(row) = executed_progress.at(row) >= past_m;
  }
  const auto reached = first_of(past_mask, 0, rows);
  std::optional<std::size_t> goal;
  if (reached.has_value()) {
    goal = first_hold(executed_inside, *reached, rows, hold_rows);
  }
  const auto measured = end.measure(executed_share, window);
  if (measured.has_value()) {
    verdict.end_share = measured->first;
    verdict.end_rise = measured->second;
  }
  verdict.moving_in = !goal.has_value() && reached.has_value() && end.holds(measured);
  if (verdict.moving_in) {
    goal = window - 1U;
  }
  if (!goal.has_value() && executed_inside.size() < window) {
    verdict.undecided = "run_ended";
  } else if (!goal.has_value()) {
    verdict.failure = reached.has_value() ? "not_in_preferred_lane" : "not_past_vehicle";
  }
  verdict.goal_row = goal;
  verdict.recorded_goal_row = back;
  verdict.progress_row = reached;
  verdict.recorded_progress_row = recorded_past;
  verdict.max_progress_m =
    rows > 0U ? *std::max_element(executed_progress.begin(), executed_progress.begin() + rows)
              : 0.0;
  return verdict;
}

LaneGoalResult evaluate_lane_goal(
  const ClosedLoopEpisode & episode, const std::string & kind,
  const std::vector<std::string> & ignored_track_ids, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const LaneGoalParameters & parameters)
{
  if (kind != "lane_change" && kind != "avoidance") {
    throw std::invalid_argument("lane_goal judges avoidance and lane_change, not " + kind);
  }
  const double wheel_base = vehicle_info.wheel_base_m;
  const double tread = vehicle_info.wheel_tread_m;
  if (!(wheel_base > 0.0)) {
    throw std::invalid_argument("lane_goal needs the ego's wheel base");
  }
  if (tread > vehicle_info.vehicle_width_m) {
    throw std::invalid_argument("lane_goal: the wheel tread is wider than the ego");
  }
  const auto & recorded = episode.recorded_poses;
  const auto window = episode.window_rows;
  const double interval = episode.interval_s;
  const bool change = kind == "lane_change";

  LaneGoalResult result;
  result.kind = kind;
  std::vector<BoxCorners> executed_wheels;
  std::vector<BoxCorners> executed_boxes;
  for (const auto & point : episode.ego.points) {
    executed_wheels.push_back(wheels_of(point.pose, wheel_base, tread));
    executed_boxes.push_back(ego_box(point.pose, vehicle_info));
  }
  std::vector<BoxCorners> recorded_wheels;
  std::vector<BoxCorners> recorded_boxes;
  LineString2d path;
  for (const auto & pose : recorded) {
    recorded_wheels.push_back(wheels_of(pose, wheel_base, tread));
    recorded_boxes.push_back(ego_box(pose, vehicle_info));
    path.emplace_back(pose.position.x, pose.position.y);
  }
  if (recorded.size() < window || window < 1U) {
    throw std::invalid_argument("lane_goal: the recorded rows must cover the window");
  }

  const auto hold_rows = rows_of(parameters.hold_s, interval);
  const auto lane_rows = std::min(window, rows_of(parameters.goal_lane_s, interval));
  std::vector<std::size_t> rows;
  for (std::size_t i = 0; i < lane_rows; ++i) {
    rows.push_back(change ? window - 1U - i : i);
  }
  StartLaneOptions options;
  options.reach_m = parameters.start_lane_reach_m;
  options.tolerance_m = parameters.lane_tolerance_m;
  options.successor_lanelets = parameters.successor_lanelets;
  options.one_branch = true;
  options.branch_ahead = !change;
  options.heading_deg = parameters.lane_heading_deg;
  std::vector<MultiPolygon2d> lanes;
  for (const auto row : rows) {
    if (auto lane = layout.start_lane(recorded_wheels.at(row), path, options)) {
      lanes.push_back(std::move(*lane));
    }
  }
  std::vector<std::vector<char>> held(lanes.size(), std::vector<char>(rows.size(), 0));
  std::size_t most = 0;
  for (std::size_t index = 0; index < lanes.size(); ++index) {
    for (std::size_t k = 0; k < rows.size(); ++k) {
      held.at(index).at(k) = LaneLayout::inside(recorded_wheels.at(rows.at(k)), lanes.at(index));
    }
    most = std::max(
      most, static_cast<std::size_t>(std::count(held.at(index).begin(), held.at(index).end(), 1)));
  }
  std::optional<MultiPolygon2d> lane;
  if (change) {
    const auto chosen = choose_lane(held, parameters.goal_lane_share);
    if (chosen.has_value()) {
      lane = lanes.at(*chosen);
    }
    result.lane_from = "wheels";
  } else if (most > 0U) {
    const auto chosen = choose_lane(held, 0.0);
    lane = lanes.at(*chosen);
    result.lane_from = "wheels";
  } else {
    auto loose = options;
    loose.hold = false;
    lane = layout.start_lane(recorded_wheels.at(rows.front()), path, loose);
    result.lane_from = lane.has_value() ? "centre" : "";
  }
  result.lane_found = lane.has_value();
  result.lane_held_rows = most;

  const auto start_rows = std::min(window, rows_of(parameters.recorded_start_s, interval));
  EndRule end;
  end.share = parameters.end_share;
  end.rise = parameters.end_rise;
  end.rise_rows = rows_of(parameters.end_rise_s, interval);

  if (!lane.has_value()) {
    result.verdict.undecided = change ? "no_target_lane" : "no_preferred_lane";
  } else {
    std::vector<char> executed_inside;
    std::vector<double> executed_share;
    for (std::size_t row = 0; row < executed_wheels.size(); ++row) {
      executed_inside.push_back(LaneLayout::inside(executed_wheels.at(row), *lane));
      executed_share.push_back(LaneLayout::share_inside(executed_boxes.at(row), *lane));
    }
    std::vector<char> recorded_inside;
    std::vector<double> recorded_share;
    for (std::size_t row = 0; row < recorded_wheels.size(); ++row) {
      recorded_inside.push_back(LaneLayout::inside(recorded_wheels.at(row), *lane));
      recorded_share.push_back(LaneLayout::share_inside(recorded_boxes.at(row), *lane));
    }
    for (std::size_t row = 0; row < executed_inside.size();) {
      if (executed_inside.at(row) == 0) {
        ++row;
        continue;
      }
      const auto first = row;
      while (row < executed_inside.size() && executed_inside.at(row) != 0) {
        ++row;
      }
      result.lane_runs.emplace_back(first, row);
    }
    if (change) {
      result.verdict = lane_change_goal(
        executed_inside, recorded_inside, window, hold_rows, start_rows, end, executed_share,
        recorded_share);
    } else {
      result.avoided =
        avoided_vehicle(episode, path, window, parameters, ignored_track_ids, vehicle_info);
      if (!result.avoided.has_value()) {
        result.verdict.undecided = "no_avoided_vehicle";
      } else {
        const double rear_m = -vehicle_info.min_longitudinal_offset_m;
        std::vector<double> executed_progress;
        for (const auto & point : episode.ego.points) {
          executed_progress.push_back(rear_progress(point.pose, path, rear_m));
        }
        std::vector<double> recorded_progress;
        for (const auto & pose : recorded) {
          recorded_progress.push_back(rear_progress(pose, path, rear_m));
        }
        result.verdict = avoidance_goal(
          executed_inside, recorded_inside, executed_progress, recorded_progress,
          result.avoided->far_m, window, hold_rows, end, executed_share, recorded_share);
      }
    }
  }

  // The failure, or why there is no verdict, in words.
  const auto & verdict = result.verdict;
  const std::string code = !verdict.failure.empty() ? verdict.failure : verdict.undecided;
  if (code.empty()) {
    return result;
  }
  std::string end_clause;
  if (verdict.end_share.has_value() && verdict.end_rise.has_value()) {
    end_clause = "; on its last row " + percent(*verdict.end_share) + " of its box was in it";
    if (*verdict.end_share < end.share) {
      end_clause += " (on its way in, " + percent(end.share) + " passes)";
    } else {
      end_clause += ", but " + format("%+.0f%%", *verdict.end_rise * 100.0) + " over the last " +
                    format("%.1f", static_cast<double>(end.rise_rows) * interval) +
                    " s: not moving in (+" + percent(end.rise) + ")";
    }
  }
  const std::string held_text =
    hold_rows == 1U ? ""
                    : " for " + format("%.1f", static_cast<double>(hold_rows) * interval) + " s";
  const std::string window_text = format("%.1f", static_cast<double>(window) * interval);
  const std::string track = result.avoided.has_value() ? result.avoided->track_id : "None";
  std::string text;
  if (code == "not_in_target_lane") {
    text = "the wheels were not wholly in the target lane" + held_text +
           " from any row before the window's end at " + window_text + " s" + end_clause;
  } else if (code == "not_past_vehicle") {
    text = "it never got past the avoided vehicle " + track + " (its far end " +
           format("%.1f", verdict.past_m.value_or(0.0)) +
           " m along the recorded path; the rear bumper got to " +
           format("%.1f", verdict.max_progress_m.value_or(0.0)) +
           " m) before the window's end at " + window_text + " s";
  } else if (code == "not_in_preferred_lane") {
    text = "past the avoided vehicle " + track + " at " +
           format("%.1f", static_cast<double>(verdict.progress_row.value_or(0U)) * interval) +
           " s, but the wheels were not wholly in the preferred lane" + held_text +
           " before the window's end at " + window_text + " s" + end_clause;
  } else if (code == "no_target_lane") {
    text = "no lane holds the recorded wheels on more than " + percent(parameters.goal_lane_share) +
           " of the window's last " + format("%.1f", static_cast<double>(lane_rows) * interval) +
           " s";
  } else if (code == "no_preferred_lane") {
    text = "no lanelet under the recorded drive at the window's start";
  } else if (code == "no_avoided_vehicle") {
    text = "no standing vehicle within " + format("%.1f", parameters.avoided_reach_m) +
           " m of the recorded drive that it gets past inside the window";
  } else if (code == "recorded_in_target_lane") {
    text = "the recorded drive is wholly in the target lane all through the window's first " +
           format("%.1f", static_cast<double>(start_rows) * interval) +
           " s: the lane change is over before the window";
  } else if (code == "recorded_not_settled") {
    text = "the recorded drive is not wholly in the target lane" + held_text +
           " before the window's end at " + window_text +
           " s, nor on its way into it on its last row";
  } else if (code == "recorded_not_back") {
    text = "the recorded drive is not wholly back in the preferred lane" + held_text +
           " after the avoided vehicle " + track + " before the window's end at " + window_text +
           " s, nor on its way back on its last row";
  } else if (code == "run_ended") {
    text = "the run ended at " +
           format("%.1f", static_cast<double>(episode.executed_rows()) * interval) +
           " s, before the window's end at " + window_text + " s, without reaching its goal";
  }
  if (!verdict.failure.empty()) {
    result.fail_reasons.push_back(text);
  } else {
    result.undecided_reason = text;
  }
  return result;
}

nlohmann::json to_json(const LaneGoalResult & result)
{
  const auto optional_row = [](const std::optional<std::size_t> & row) {
    return row.has_value() ? nlohmann::json(*row) : nlohmann::json(nullptr);
  };
  const auto optional_value = [](const std::optional<double> & value) {
    return value.has_value() ? nlohmann::json(*value) : nlohmann::json(nullptr);
  };
  const auto text_or_null = [](const std::string & text) {
    return text.empty() ? nlohmann::json(nullptr) : nlohmann::json(text);
  };
  const auto & verdict = result.verdict;
  const auto passed = verdict.passed();
  nlohmann::json runs = nlohmann::json::array();
  for (const auto & [first, end] : result.lane_runs) {
    runs.push_back({first, end});
  }
  nlohmann::json avoided = nullptr;
  if (result.avoided.has_value()) {
    avoided = {
      {"track_id", result.avoided->track_id},
      {"far_m", result.avoided->far_m},
      {"near_m", result.avoided->near_m},
      {"candidates", result.avoided->candidates}};
  }
  return {
    {"kind", result.kind},
    {"passed", passed.has_value() ? nlohmann::json(*passed) : nlohmann::json(nullptr)},
    {"goal_lane", result.kind == "lane_change" ? "target_lane" : "preferred_lane"},
    {"lane_found", result.lane_found},
    {"lane_from", text_or_null(result.lane_from)},
    {"lane_held_rows", result.lane_held_rows},
    {"lane_runs", runs},
    {"avoided", avoided},
    {"goal_row", optional_row(verdict.goal_row)},
    {"recorded_goal_row", optional_row(verdict.recorded_goal_row)},
    {"failure", text_or_null(verdict.failure)},
    {"undecided", text_or_null(verdict.undecided)},
    {"past_m", optional_value(verdict.past_m)},
    {"progress_row", optional_row(verdict.progress_row)},
    {"recorded_progress_row", optional_row(verdict.recorded_progress_row)},
    {"max_progress_m", optional_value(verdict.max_progress_m)},
    {"end_share", optional_value(verdict.end_share)},
    {"end_rise", optional_value(verdict.end_rise)},
    {"moving_in", verdict.moving_in},
    {"recorded_moving_in", verdict.recorded_moving_in},
    {"fail_reasons", result.fail_reasons},
    {"undecided_reason", text_or_null(result.undecided_reason)},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

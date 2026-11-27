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

// A yielding scene's targets, picked from the recorded (human) drive (without track joining).

#include "closed_loop/yield_targets.hpp"

#include "closed_loop/contacts.hpp"
#include "closed_loop/road_users.hpp"
#include "metrics/geometry/metric_utils.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

namespace bg = boost::geometry;

constexpr double kInf = std::numeric_limits<double>::infinity();
// Spacing of the points along the human's path that "near the path" is measured to.
constexpr double kPathStepM = 1.0;

double wrap_angle(const double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

struct Velocity
{
  double x{0.0};
  double y{0.0};
  bool fitted{false};
  double speed() const { return std::hypot(x, y); }
};

// common.kinematics.fitted_velocities of one road user: the slope of the straight line fitted to
// its positions on the valid rows within +-half rows, per row; none with fewer than two.
std::vector<Velocity> fitted_velocities(
  const RoadUserColumns & users, const std::size_t column, const std::size_t rows,
  const std::size_t half, const double interval)
{
  std::vector<Velocity> velocities(rows);
  for (std::size_t row = 0; row < rows; ++row) {
    const std::size_t low = row >= half ? row - half : 0U;
    const std::size_t high = std::min(rows, row + half + 1U);
    double count = 0.0;
    double sum_t = 0.0;
    double sum_tt = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_tx = 0.0;
    double sum_ty = 0.0;
    for (std::size_t other = low; other < high; ++other) {
      if (users.valid.at(other).at(column) == 0) {
        continue;
      }
      const double tau = static_cast<double>(other);
      const auto & xy = users.xy.at(other).at(column);
      count += 1.0;
      sum_t += tau;
      sum_tt += tau * tau;
      sum_x += xy.x();
      sum_y += xy.y();
      sum_tx += tau * xy.x();
      sum_ty += tau * xy.y();
    }
    const double spread = sum_tt - sum_t * sum_t / std::max(count, 1.0);
    if (count < 2.0 || spread <= 1.0e-12) {
      continue;
    }
    velocities.at(row).x = (sum_tx - sum_t * sum_x / count) / spread / interval;
    velocities.at(row).y = (sum_ty - sum_t * sum_y / count) / spread / interval;
    velocities.at(row).fitted = true;
  }
  return velocities;
}

}  // namespace

TargetPick pick_targets(
  const ClosedLoopEpisode & episode, const std::string & kind,
  const lanelet::LaneletMapConstPtr & map,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::vector<std::string> & ignored_track_ids, const TargetRules & rules)
{
  if (kind != "vehicle_yield" && kind != "pedestrian_yield") {
    throw std::invalid_argument("pick_targets: kind must be vehicle_yield or pedestrian_yield");
  }
  const auto & recorded = episode.recorded_poses;
  if (recorded.size() < 2U) {
    throw std::invalid_argument("pick_targets needs the recorded drive on at least two rows");
  }
  const double interval = episode.interval_s;
  const auto half = static_cast<std::size_t>(std::llround(rules.velocity_half_window_s / interval));
  if (half < 1U) {
    throw std::invalid_argument("velocity_half_window_s must cover at least one row either side");
  }
  const auto kept = without_road_users(episode, ignored_track_ids);
  const auto users = road_user_columns(kept, half);
  const auto rows = std::min(users.rows(), recorded.size());
  const auto window = std::min(episode.window_rows, rows);
  TargetPick pick;
  if (window < 1U || users.columns() == 0U) {
    return pick;
  }

  // Road users in the order they first appear.
  std::vector<std::size_t> columns(users.columns());
  std::iota(columns.begin(), columns.end(), 0U);
  const auto first_row = [&](const std::size_t column) {
    for (std::size_t row = 0; row < rows; ++row) {
      if (users.valid.at(row).at(column) != 0) {
        return row;
      }
    }
    return rows;
  };
  std::stable_sort(columns.begin(), columns.end(), [&](const auto lhs, const auto rhs) {
    return std::make_pair(first_row(lhs), users.track_ids.at(lhs)) <
           std::make_pair(first_row(rhs), users.track_ids.at(rhs));
  });
  const auto count = columns.size();
  const auto valid = [&](const std::size_t row, const std::size_t a) {
    return users.valid.at(row).at(columns.at(a)) != 0;
  };
  const auto type = [&](const std::size_t a) { return users.types.at(columns.at(a)); };

  std::vector<BoxCorners> ego;
  std::vector<double> ego_yaw;
  LineString2d path;
  for (std::size_t row = 0; row < rows; ++row) {
    ego.push_back(ego_box(recorded.at(row), vehicle_info));
    ego_yaw.push_back(metrics::get_yaw(recorded.at(row).orientation));
  }
  for (const auto & pose : recorded) {
    path.emplace_back(pose.position.x, pose.position.y);
  }

  // Box distances to the human, velocities, the observed rows.
  std::vector<std::vector<double>> gaps(rows, std::vector<double>(count, kInf));
  std::vector<std::vector<Velocity>> velocity(count);
  std::vector<std::size_t> observed(count, 0U);
  std::vector<double> window_gap(count, kInf);
  std::vector<std::size_t> window_row(count, 0U);
  for (std::size_t a = 0; a < count; ++a) {
    velocity.at(a) = fitted_velocities(users, columns.at(a), rows, half, interval);
    for (std::size_t row = 0; row < rows; ++row) {
      if (!valid(row, a)) {
        continue;
      }
      ++observed.at(a);
      gaps.at(row).at(a) = box_clearance(ego.at(row), users.corners.at(row).at(columns.at(a)));
      if (row < window && gaps.at(row).at(a) < window_gap.at(a)) {
        window_gap.at(a) = gaps.at(row).at(a);
        window_row.at(a) = row;
      }
    }
  }
  const auto min_track_rows = static_cast<std::size_t>(std::llround(rules.min_track_s / interval));

  // Rule 2's test: heading against the human on the first row it moves within range.
  std::vector<char> oncoming(count, 0);
  for (std::size_t a = 0; a < count; ++a) {
    for (std::size_t row = 0; row < rows; ++row) {
      const auto & v = velocity.at(a).at(row);
      if (!valid(row, a) || !v.fitted || v.speed() < rules.oncoming_min_mps) {
        continue;
      }
      const auto & xy = users.xy.at(row).at(columns.at(a));
      const auto & position = recorded.at(row).position;
      if (std::hypot(xy.x() - position.x, xy.y() - position.y) > rules.oncoming_range_m) {
        continue;
      }
      const double turn =
        std::abs(wrap_angle(users.yaw.at(row).at(columns.at(a)) - ego_yaw.at(row)));
      oncoming.at(a) = turn * 180.0 / M_PI > rules.oncoming_heading_deg;
      break;
    }
  }

  // The window's rows on which a pedestrian or bicycle is on a crosswalk the human's path crosses.
  std::vector<Polygon2d> crossed;
  if (map) {
    for (const auto & lanelet : map->laneletLayer) {
      if (std::string(lanelet.attributeOr(lanelet::AttributeName::Subtype, "")) != "crosswalk") {
        continue;
      }
      Polygon2d polygon;
      for (const auto & point : lanelet.polygon2d()) {
        polygon.outer().emplace_back(point.x(), point.y());
      }
      bg::correct(polygon);
      if (bg::intersects(polygon, path)) {
        crossed.push_back(polygon);
      }
    }
  }
  std::vector<std::vector<char>> on_crosswalk(window, std::vector<char>(count, 0));
  std::vector<char> touched(count, 0);
  for (std::size_t row = 0; row < window && !crossed.empty(); ++row) {
    for (std::size_t a = 0; a < count; ++a) {
      if (!valid(row, a) || type(a) == RoadUserType::VEHICLE) {
        continue;
      }
      const auto & xy = users.xy.at(row).at(columns.at(a));
      double distance = kInf;
      for (const auto & polygon : crossed) {
        distance = std::min(distance, bg::distance(xy, polygon));
      }
      on_crosswalk.at(row).at(a) = distance <= rules.crosswalk_tolerance_m;
      touched.at(a) = touched.at(a) || on_crosswalk.at(row).at(a);
    }
  }

  // Near the human's path: points every kPathStepM along it.
  std::vector<Point2d> samples;
  {
    std::vector<double> along{0.0};
    for (std::size_t i = 1; i < path.size(); ++i) {
      along.push_back(
        along.back() +
        std::hypot(path.at(i).x() - path.at(i - 1).x(), path.at(i).y() - path.at(i - 1).y()));
    }
    for (double s = 0.0; s < along.back() + kPathStepM; s += kPathStepM) {
      // numpy.interp over the points where the path moved (clamped at its ends).
      std::size_t i = 1;
      while (i < along.size() && along.at(i) < s) {
        ++i;
      }
      if (i >= along.size()) {
        samples.push_back(path.back());
        continue;
      }
      const double span = along.at(i) - along.at(i - 1);
      const double ratio = span > 0.0 ? (s - along.at(i - 1)) / span : 0.0;
      samples.emplace_back(
        path.at(i - 1).x() + ratio * (path.at(i).x() - path.at(i - 1).x()),
        path.at(i - 1).y() + ratio * (path.at(i).y() - path.at(i - 1).y()));
    }
  }
  double reach = 0.0;
  for (std::size_t row = 0; row < rows; ++row) {
    for (const auto & corner : ego.at(row)) {
      reach = std::max(
        reach,
        std::hypot(
          corner.x() - recorded.at(row).position.x, corner.y() - recorded.at(row).position.y));
    }
  }

  // Measure every road user near the path against the human, on the rows the boxes do not overlap.
  std::vector<char> candidate(count, 0);
  std::vector<char> before(count, 0);
  std::vector<char> after(count, 0);
  std::vector<std::int64_t> pass_row(count, -1);
  std::vector<std::int64_t> pet_rows(count, -1);
  std::vector<double> min_gap(count, kInf);
  std::vector<std::size_t> measured;
  std::vector<std::size_t> closest_row(count, 0U);
  for (std::size_t a = 0; a < count; ++a) {
    double nearest = kInf;
    double size = 0.0;
    std::size_t kept_rows = 0;
    for (std::size_t row = 0; row < rows; ++row) {
      if (!valid(row, a)) {
        continue;
      }
      const auto & xy = users.xy.at(row).at(columns.at(a));
      for (const auto & point : samples) {
        nearest = std::min(nearest, std::hypot(xy.x() - point.x(), xy.y() - point.y()));
      }
      const auto & box = users.corners.at(row).at(columns.at(a));
      size = std::max(size, 0.5 * std::hypot(box[0].x() - box[2].x(), box[0].y() - box[2].y()));
      if (gaps.at(row).at(a) > 0.0) {
        ++kept_rows;
        if (gaps.at(row).at(a) < min_gap.at(a)) {
          min_gap.at(a) = gaps.at(row).at(a);
          closest_row.at(a) = row;
        }
      }
    }
    if (
      nearest <= reach + rules.near_m + kPathStepM + size &&
      kept_rows >= static_cast<std::size_t>(rules.min_kept_rows)) {
      measured.push_back(a);
    } else {
      min_gap.at(a) = kInf;
    }
  }
  if (!measured.empty()) {
    std::vector<std::vector<BoxCorners>> corners(rows);
    std::vector<std::vector<char>> seen(rows);
    std::vector<std::vector<double>> yaw(rows);
    for (std::size_t row = 0; row < rows; ++row) {
      for (const auto a : measured) {
        const bool kept_row = valid(row, a) && gaps.at(row).at(a) > 0.0;
        seen.at(row).push_back(kept_row);
        corners.at(row).push_back(
          kept_row ? users.corners.at(row).at(columns.at(a)) : BoxCorners{});
        yaw.at(row).push_back(kept_row ? users.yaw.at(row).at(columns.at(a)) : 0.0);
      }
    }
    const auto crossings = measure_crossings(
      ego, ego_yaw, corners, seen, yaw,
      Point2d{recorded.front().position.x, recorded.front().position.y}, rules.grid);
    for (std::size_t i = 0; i < measured.size(); ++i) {
      const auto a = measured.at(i);
      const auto & crossing = crossings.at(i);
      before.at(a) = crossing.order == CrossingOrder::ROAD_USER_FIRST;
      after.at(a) = crossing.order == CrossingOrder::EGO_FIRST;
      const bool near = crossing.order == CrossingOrder::NONE && min_gap.at(a) <= rules.near_m;
      candidate.at(a) =
        before.at(a) || after.at(a) || crossing.order == CrossingOrder::NO_CROSSING || near;
      if (candidate.at(a)) {
        pass_row.at(a) = before.at(a)  ? crossing.road_user_zone_rows[1]
                         : after.at(a) ? crossing.road_user_zone_rows[0]
                                       : static_cast<std::int64_t>(closest_row.at(a));
      }
      pet_rows.at(a) = crossing.pet_rows();
    }
  }

  // The rules.
  const auto is = [&](const std::size_t a, const RoadUserType wanted) { return type(a) == wanted; };
  const auto own = [&](const std::size_t a) {
    return kind == "vehicle_yield"
             ? is(a, RoadUserType::VEHICLE)
             : (is(a, RoadUserType::PEDESTRIAN) || is(a, RoadUserType::BICYCLE));
  };
  std::map<std::string, std::vector<char>> picked;
  const std::vector<std::string> order{
    "final",           "ov",          "vru",      "near_car",
    "near_pedestrian", "right_after", "side_car", "same_direction"};
  for (const auto & rule : order) {
    picked[rule].assign(count, 0);
  }
  std::vector<char> crossed_before(count, 0);
  std::vector<char> long_enough(count, 0);
  for (std::size_t a = 0; a < count; ++a) {
    crossed_before.at(a) = before.at(a) && pass_row.at(a) < static_cast<std::int64_t>(window);
    long_enough.at(a) = observed.at(a) >= min_track_rows;
  }
  // 1. final: the scene's own road user, seen inside the window.
  {
    std::optional<std::size_t> best;
    std::int64_t best_key = -1;
    for (std::size_t a = 0; a < count; ++a) {
      bool in_window = false;
      for (std::size_t row = 0; row < window && !in_window; ++row) {
        in_window = valid(row, a);
      }
      if (!own(a) || !in_window || !before.at(a)) {
        continue;
      }
      const std::int64_t key =
        pass_row.at(a) * static_cast<std::int64_t>(count) + static_cast<std::int64_t>(a);
      if (key > best_key) {
        best_key = key;
        best = a;
      }
    }
    if (!best.has_value()) {
      std::int64_t least = std::numeric_limits<std::int64_t>::max();
      for (std::size_t a = 0; a < count; ++a) {
        bool in_window = false;
        for (std::size_t row = 0; row < window && !in_window; ++row) {
          in_window = valid(row, a);
        }
        if (!own(a) || !in_window || !after.at(a)) {
          continue;
        }
        const std::int64_t key =
          (pet_rows.at(a) * static_cast<std::int64_t>(rows) + pass_row.at(a)) *
            static_cast<std::int64_t>(count) +
          static_cast<std::int64_t>(a);
        if (key < least) {
          least = key;
          best = a;
        }
      }
    }
    if (best.has_value()) {
      picked["final"].at(*best) = 1;
    }
  }
  std::vector<char> have(count, 0);
  for (std::size_t a = 0; a < count; ++a) {
    picked["ov"].at(a) = crossed_before.at(a) &&
                         (is(a, RoadUserType::VEHICLE) || is(a, RoadUserType::BICYCLE)) &&
                         oncoming.at(a);
    picked["vru"].at(a) = crossed_before.at(a) &&
                          (is(a, RoadUserType::PEDESTRIAN) || is(a, RoadUserType::BICYCLE)) &&
                          (touched.at(a) || min_gap.at(a) <= rules.vru_near_m);
    have.at(a) = picked["final"].at(a) || picked["ov"].at(a) || picked["vru"].at(a);
  }
  const auto add = [&](const std::string & rule, const std::vector<char> & fits) {
    for (std::size_t a = 0; a < count; ++a) {
      picked[rule].at(a) = fits.at(a) && !have.at(a);
      have.at(a) = have.at(a) || picked[rule].at(a);
    }
  };
  const auto approach = static_cast<std::size_t>(std::llround(rules.approach_s / interval));
  std::vector<char> fits(count, 0);
  for (std::size_t a = 0; a < count; ++a) {
    fits.at(a) = crossed_before.at(a) && is(a, RoadUserType::VEHICLE) &&
                 window_gap.at(a) <= rules.near_car_m && window_row.at(a) >= approach;
  }
  add("near_car", fits);
  for (std::size_t a = 0; a < count; ++a) {
    fits.at(a) = is(a, RoadUserType::PEDESTRIAN) && long_enough.at(a) &&
                 window_gap.at(a) <= rules.near_pedestrian_m;
  }
  add("near_pedestrian", fits);
  const double pet_limit = rules.after_pet_s / interval + 1.0e-9;
  for (std::size_t a = 0; a < count; ++a) {
    fits.at(a) = after.at(a) && !is(a, RoadUserType::VEHICLE) && long_enough.at(a) &&
                 static_cast<double>(pet_rows.at(a)) <= pet_limit && touched.at(a);
  }
  add("right_after", fits);
  // 7. side_car: vehicles that crossed before the human beside a vehicle target.
  for (std::size_t a = 0; a < count; ++a) {
    fits.at(a) = 0;
    if (!(before.at(a) && is(a, RoadUserType::VEHICLE) && !have.at(a))) {
      continue;
    }
    for (std::size_t b = 0; b < count && !fits.at(a); ++b) {
      if (!(have.at(b) && is(b, RoadUserType::VEHICLE))) {
        continue;
      }
      for (std::size_t row = 0; row < window && !fits.at(a); ++row) {
        if (valid(row, a) && valid(row, b)) {
          fits.at(a) = box_clearance(
                         users.corners.at(row).at(columns.at(a)),
                         users.corners.at(row).at(columns.at(b))) <= rules.side_car_m;
        }
      }
    }
  }
  add("side_car", fits);
  // 8. same_direction: pedestrians walking with a pedestrian or bicycle target on the crosswalk.
  const auto around_rows =
    static_cast<std::int64_t>(std::llround(rules.same_direction_window_s / interval));
  for (std::size_t w = 0; w < count; ++w) {
    fits.at(w) = 0;
    if (have.at(w) || !is(w, RoadUserType::PEDESTRIAN) || !long_enough.at(w)) {
      continue;
    }
    for (std::size_t l = 0; l < count && !fits.at(w); ++l) {
      if (!(have.at(l) && !is(l, RoadUserType::VEHICLE) && candidate.at(l))) {
        continue;
      }
      const auto pass = static_cast<std::size_t>(pass_row.at(l));
      for (std::size_t row = 0; row < window && !fits.at(w); ++row) {
        if (std::abs(static_cast<std::int64_t>(row) - pass_row.at(l)) > around_rows) {
          continue;
        }
        const auto & own_velocity = velocity.at(l).at(row);
        const auto & at_pass = velocity.at(l).at(pass);
        const bool lead_fitted = own_velocity.fitted || at_pass.fitted;
        const Velocity lead = own_velocity.fitted ? own_velocity : at_pass;
        const double lead_speed = lead.speed();
        if (!lead_fitted || lead_speed < rules.same_direction_lead_min_mps || !(lead_speed > 0.0)) {
          continue;
        }
        const auto & walker = velocity.at(w).at(row);
        const double speed = walker.speed();
        if (
          !on_crosswalk.at(row).at(w) || !walker.fitted || speed < rules.same_direction_min_mps ||
          !(speed > 0.0)) {
          continue;
        }
        const double dot = walker.x * lead.x + walker.y * lead.y;
        const double cross = lead.x * walker.y - lead.y * walker.x;
        fits.at(w) = std::abs(std::atan2(cross, dot)) * 180.0 / M_PI <= rules.same_direction_deg;
      }
    }
  }
  add("same_direction", fits);

  // Targets: picked by a rule and observed long enough, in the order they first appear.
  for (std::size_t a = 0; a < count; ++a) {
    std::vector<std::string> why;
    for (const auto & rule : order) {
      if (picked.at(rule).at(a)) {
        why.push_back(rule);
      }
    }
    if (why.empty() || !long_enough.at(a)) {
      continue;
    }
    const auto & id = users.track_ids.at(columns.at(a));
    pick.target_track_ids.push_back(id);
    if (std::find(why.begin(), why.end(), "final") != why.end()) {
      pick.final_track_id = id;
    }
    pick.reasons.emplace(id, std::move(why));
  }
  return pick;
}

nlohmann::json to_json(const TargetPick & pick)
{
  nlohmann::json reasons = nlohmann::json::object();
  for (const auto & [id, why] : pick.reasons) {
    reasons[id] = why;
  }
  return {
    {"target_track_ids", pick.target_track_ids},
    {"reasons", reasons},
    {"final_track_id", pick.final_track_id.has_value() ? nlohmann::json(*pick.final_track_id)
                                                       : nlohmann::json(nullptr)},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

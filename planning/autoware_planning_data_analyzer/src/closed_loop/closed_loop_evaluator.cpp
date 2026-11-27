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

#include "closed_loop/closed_loop_evaluator.hpp"

#include "metrics/geometry/ego_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{
namespace
{

constexpr int kResultSchemaVersion = 1;

nlohmann::json finite_or_null(const double value)
{
  return std::isfinite(value) ? nlohmann::json(value) : nlohmann::json(nullptr);
}

nlohmann::json contact_to_json(const metrics::NoAtFaultCollisionContact & contact)
{
  return {
    {"object_id", contact.object_id},
    {"object_label", contact.object_label},
    {"row", contact.point_index},
    {"time_s", contact.time_s},
    {"collision_type", contact.collision_type},
    {"at_fault", contact.at_fault},
    {"exempt", contact.exempt},
    {"multiple_lanes", contact.multiple_lanes},
    {"non_drivable_area", contact.non_drivable_area},
  };
}

}  // namespace

std::optional<MarginReference> margin_reference(const ClosedLoopScene & scene)
{
  MarginReference reference;
  if (scene.margin_kind.has_value()) {
    reference.kind = *scene.margin_kind;
  } else if (scene.eval_label == "lane_change" || scene.eval_label == "pull_out") {
    reference.kind = scene.eval_label;
  } else if (scene.eval_label == "object_avoidance") {
    reference.kind = "avoidance";
  } else {
    return std::nullopt;
  }
  reference.trailing_vehicle = scene.margin_trailing_vehicle;
  reference.every_road_user = scene.margin_every_road_user.value_or(scene.difficulty == "easy");
  reference.ignored_track_ids = scene.ignored_track_ids;
  return reference;
}

bool is_yield_scene(const ClosedLoopScene & scene)
{
  return scene.eval_label == "pedestrian_yield" || scene.eval_label == "vehicle_yield";
}

ClosedLoopEvaluator::ClosedLoopEvaluator(
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, ClosedLoopEvaluatorParameters parameters)
: route_handler_(std::move(route_handler)),
  vehicle_info_(std::move(vehicle_info)),
  parameters_(std::move(parameters))
{
  if (route_handler_ && route_handler_->isMapMsgReady()) {
    lane_layout_ = std::make_unique<LaneLayout>(route_handler_->getLaneletMapPtr());
  }
}

metrics::NoAtFaultCollisionOptions ClosedLoopEvaluator::no_at_fault_collision_options() const
{
  metrics::NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.from_behind_heading_deg = parameters_.from_behind_heading_deg;
  return options;
}

ClosedLoopEvaluationResult ClosedLoopEvaluator::evaluate(
  const ClosedLoopScene & scene, const ClosedLoopEpisode & episode) const
{
  ClosedLoopEvaluationResult result;
  const auto footprints =
    metrics::evaluate_ego_path_footprints(episode.ego, vehicle_info_, route_handler_);
  result.no_at_fault_collision = metrics::calculate_no_at_fault_collision(
    episode.ego, episode.road_users, vehicle_info_, route_handler_, footprints, false,
    no_at_fault_collision_options());

  result.yield_judged = is_yield_scene(scene);
  if (result.yield_judged) {
    if (!lane_layout_) {
      result.yield_error = "no map";
    } else if (!scene.yield_target_track_ids.has_value()) {
      result.yield_error = "no yield targets";
    } else {
      try {
        result.yield = evaluate_yield(
          episode, *scene.yield_target_track_ids, scene.ignored_track_ids, *lane_layout_,
          vehicle_info_, parameters_.yield);
      } catch (const std::invalid_argument & error) {
        result.yield_error = error.what();
      }
    }
    if (result.yield.has_value()) {
      result.passed = result.yield->passed;
      result.fail_reasons = result.yield->fail_reasons;
    }
    return result;
  }

  result.margin_reference = margin_reference(scene);
  if (result.margin_reference.has_value()) {
    if (!lane_layout_) {
      result.margin_error = "no map";
    } else {
      try {
        result.margin = evaluate_margin(
          episode, *result.margin_reference, *lane_layout_, vehicle_info_, parameters_.margin);
      } catch (const std::invalid_argument & error) {
        result.margin_error = error.what();
      }
    }
    // An easy lane change or avoidance must also reach its lane (the lane goal).
    const auto & kind = result.margin_reference->kind;
    result.lane_goal_judged =
      result.margin_reference->every_road_user && (kind == "lane_change" || kind == "avoidance");
    if (result.lane_goal_judged) {
      if (!lane_layout_) {
        result.lane_goal_error = "no map";
      } else {
        try {
          result.lane_goal = evaluate_lane_goal(
            episode, kind, result.margin_reference->ignored_track_ids, *lane_layout_, vehicle_info_,
            parameters_.lane_goal);
        } catch (const std::invalid_argument & error) {
          result.lane_goal_error = error.what();
        }
      }
    }
    // The scene's verdict is its metrics': margin's (which judges collisions with its own contact
    // rule), ANDed with the lane goal's for an easy scene. Any false fails; else any metric
    // without a verdict leaves the scene without one.
    std::vector<std::optional<bool>> verdicts;
    verdicts.push_back(
      result.margin.has_value() ? std::optional<bool>(result.margin->passed) : std::nullopt);
    if (result.margin.has_value()) {
      result.fail_reasons = result.margin->fail_reasons;
    }
    if (result.lane_goal_judged) {
      verdicts.push_back(
        result.lane_goal.has_value() ? result.lane_goal->verdict.passed() : std::nullopt);
      if (result.lane_goal.has_value()) {
        for (const auto & reason : result.lane_goal->fail_reasons) {
          result.fail_reasons.push_back("lane goal: " + reason);
        }
      }
    }
    const bool any_failed = std::any_of(verdicts.begin(), verdicts.end(), [](const auto & verdict) {
      return verdict.has_value() && !*verdict;
    });
    const bool all_judged = std::all_of(
      verdicts.begin(), verdicts.end(), [](const auto & verdict) { return verdict.has_value(); });
    if (any_failed) {
      result.passed = false;
    } else if (all_judged) {
      result.passed = true;
    }
    return result;
  }

  const auto & no_at_fault_collision = result.no_at_fault_collision;
  if (!no_at_fault_collision.available) {
    return result;
  }
  result.passed = no_at_fault_collision.score >= 1.0;
  if (!*result.passed) {
    result.fail_reasons.push_back("collision");
  }
  return result;
}

nlohmann::json to_json(
  const ClosedLoopScene & scene, const ClosedLoopEpisode & episode,
  const ClosedLoopEvaluationResult & result)
{
  nlohmann::json contacts = nlohmann::json::array();
  for (const auto & contact : result.no_at_fault_collision.contacts) {
    contacts.push_back(contact_to_json(contact));
  }
  const auto & no_at_fault_collision = result.no_at_fault_collision;
  nlohmann::json margin = nullptr;
  if (result.margin.has_value()) {
    margin = to_json(*result.margin, *result.margin_reference);
  } else if (result.margin_reference.has_value()) {
    margin = {{"error", result.margin_error}};
  }
  nlohmann::json yield = nullptr;
  if (result.yield.has_value()) {
    yield = to_json(*result.yield);
  } else if (result.yield_judged) {
    yield = {{"error", result.yield_error}};
  }
  nlohmann::json lane_goal = nullptr;
  if (result.lane_goal.has_value()) {
    lane_goal = to_json(*result.lane_goal);
  } else if (result.lane_goal_judged) {
    lane_goal = {{"error", result.lane_goal_error}};
  }
  return {
    {"schema_version", kResultSchemaVersion},
    {"mode", "closed_loop"},
    {"scene", to_json(scene)},
    {"episode",
     {{"interval_s", episode.interval_s},
      {"window_rows", episode.window_rows},
      {"executed_rows", episode.executed_rows()},
      {"recorded_rows", episode.recorded_poses.size()},
      {"road_user_tracks", episode.road_users.size()},
      {"dropped_short_tracks", episode.dropped_short_tracks}}},
    {"no_at_fault_collision",
     {{"available", no_at_fault_collision.available},
      {"score", no_at_fault_collision.score},
      {"reason", no_at_fault_collision.reason},
      {"infraction_time_s", finite_or_null(no_at_fault_collision.infraction_time_s)},
      {"contacts", contacts}}},
    {"margin", margin},
    {"lane_goal", lane_goal},
    {"yield", yield},
    {"verdict",
     {{"passed",
       result.passed.has_value() ? nlohmann::json(*result.passed) : nlohmann::json(nullptr)},
      {"fail_reasons", result.fail_reasons}}},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

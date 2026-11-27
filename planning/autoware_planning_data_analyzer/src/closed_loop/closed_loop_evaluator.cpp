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

ClosedLoopEvaluator::ClosedLoopEvaluator(
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, ClosedLoopEvaluatorParameters parameters)
: route_handler_(std::move(route_handler)),
  vehicle_info_(std::move(vehicle_info)),
  parameters_(std::move(parameters))
{
}

metrics::NoAtFaultCollisionOptions ClosedLoopEvaluator::no_at_fault_collision_options() const
{
  metrics::NoAtFaultCollisionOptions options;
  options.first_contact_decides = true;
  options.from_behind_heading_deg = parameters_.from_behind_heading_deg;
  return options;
}

ClosedLoopEvaluationResult ClosedLoopEvaluator::evaluate(
  const ClosedLoopScene & /*scene*/, const ClosedLoopEpisode & episode) const
{
  ClosedLoopEvaluationResult result;
  const auto footprints =
    metrics::evaluate_ego_path_footprints(episode.ego, vehicle_info_, route_handler_);
  result.no_at_fault_collision = metrics::calculate_no_at_fault_collision(
    episode.ego, episode.road_users, vehicle_info_, route_handler_, footprints, false,
    no_at_fault_collision_options());

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
    {"verdict",
     {{"passed",
       result.passed.has_value() ? nlohmann::json(*result.passed) : nlohmann::json(nullptr)},
      {"fail_reasons", result.fail_reasons}}},
  };
}

}  // namespace autoware::planning_data_analyzer::closed_loop

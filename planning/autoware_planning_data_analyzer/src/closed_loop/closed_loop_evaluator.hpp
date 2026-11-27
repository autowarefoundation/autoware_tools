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

#ifndef CLOSED_LOOP__CLOSED_LOOP_EVALUATOR_HPP_
#define CLOSED_LOOP__CLOSED_LOOP_EVALUATOR_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/closed_loop_scene.hpp"
#include "metrics/epdms/subscores/no_at_fault_collision.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

struct ClosedLoopEvaluatorParameters
{
  // NC's from-behind exemption: the heading difference (deg) within which a road user behind the
  // ego drove into it.
  double from_behind_heading_deg{45.0};
};

struct ClosedLoopEvaluationResult
{
  // NC of the driven path, each road user judged at its first contact, from the window's start
  // until the run's end.
  metrics::NoAtFaultCollisionResult no_at_fault_collision;
  // True when the run passes, false when it fails, empty when it cannot be judged.
  std::optional<bool> passed;
  std::vector<std::string> fail_reasons;
};

// Judges one executed closed-loop run (a ClosedLoopEpisode) of a scene.
class ClosedLoopEvaluator
{
public:
  ClosedLoopEvaluator(
    std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
    autoware::vehicle_info_utils::VehicleInfo vehicle_info,
    ClosedLoopEvaluatorParameters parameters = {});

  ClosedLoopEvaluationResult evaluate(
    const ClosedLoopScene & scene, const ClosedLoopEpisode & episode) const;

  // The NC options of the closed-loop rule: first contact decides, with the from-behind exemption.
  metrics::NoAtFaultCollisionOptions no_at_fault_collision_options() const;

private:
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  ClosedLoopEvaluatorParameters parameters_;
};

// The result file: the scene, the episode's rows, NC with every first contact, and the verdict.
nlohmann::json to_json(
  const ClosedLoopScene & scene, const ClosedLoopEpisode & episode,
  const ClosedLoopEvaluationResult & result);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CLOSED_LOOP_EVALUATOR_HPP_

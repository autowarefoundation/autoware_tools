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

#ifndef METRICS__EPDMS__SUBSCORES__EGO_PROGRESS_HPP_
#define METRICS__EPDMS__SUBSCORES__EGO_PROGRESS_HPP_

#include "../../../data_types.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <memory>
#include <string>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

struct EgoProgressResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double raw_progress_m{0.0};
  double best_raw_progress_m{0.0};
};

EgoProgressResult calculate_ego_progress(
  const std::shared_ptr<Trajectory> & selected_trajectory,
  const std::shared_ptr<CandidateTrajectories> & candidate_trajectories,
  const std::shared_ptr<RouteHandler> & route_handler);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__SUBSCORES__EGO_PROGRESS_HPP_

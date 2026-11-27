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

#ifndef CLOSED_LOOP__YIELD_CROSSING_HPP_
#define CLOSED_LOOP__YIELD_CROSSING_HPP_

#include "closed_loop/closed_loop_episode.hpp"
#include "closed_loop/conflict.hpp"
#include "closed_loop/lane_layout.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// The thresholds of the yielding verdict (the reviewed metric's values).
struct YieldParameters
{
  // Road user first: its PET (s) must reach the line of its type; a type without a line is not
  // judged. The lines started from the human drives' 5th percentile and were lowered by review.
  std::map<std::string, double> pet_line_s{{"pedestrian", 1.3}, {"bicycle", 1.2}, {"vehicle", 1.0}};
  // Ego first: the entry gap (s) must reach this (provisional: only four human drives went first).
  double entry_gap_line_s{2.4};
  // Ego first while slower than this when it leaves the deciding cell: no verdict.
  double ego_stopped_mps{0.5};
  CrossingGrid grid;
  // The ego must stay within route_deviation_m of the recorded path while that path still runs
  // alongside it (up to route_end_margin_m before its end).
  double route_deviation_m{5.0};
  double route_end_margin_m{2.0};
  double rear_heading_deg{45.0};
  double min_track_s{1.0};
};

struct YieldTargetRecord
{
  std::string track_id;
  std::string type;
  std::optional<Crossing> crossing;
  // "pet" or "entry_gap"; empty without a measure.
  std::string measure;
  std::optional<double> value_s;
  std::optional<double> line_s;
  // Empty without a verdict.
  std::optional<bool> passed;
  std::string unjudged;
};

struct YieldContactRecord
{
  std::string track_id;
  std::string type;
  std::size_t row{0};
  std::string kind;
  bool at_fault{false};
  bool target{false};
};

struct YieldResult
{
  bool passed{true};
  std::vector<YieldTargetRecord> targets;
  std::vector<YieldContactRecord> contacts;
  std::optional<std::size_t> left_route_row;
  double max_route_deviation_m{0.0};
  std::vector<std::string> fail_reasons;
};

// Did the ego yield? Every target that crossed its path is judged on its PET (it went first) or
// the entry gap (the ego went first); no contact with any road user may be the ego's fault; and the
// ego must stay near the recorded path. Being slower than the human, or stopping for good, does not
// fail. Throws std::invalid_argument without a recorded drive.
YieldResult evaluate_yield(
  const ClosedLoopEpisode & episode, const std::vector<std::string> & target_track_ids,
  const std::vector<std::string> & ignored_track_ids, const LaneLayout & layout,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const YieldParameters & parameters = {});

nlohmann::json to_json(const YieldResult & result);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__YIELD_CROSSING_HPP_

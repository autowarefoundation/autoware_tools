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

#ifndef CLOSED_LOOP__CLOSED_LOOP_SCENE_HPP_
#define CLOSED_LOOP__CLOSED_LOOP_SCENE_HPP_

#include <nlohmann/json.hpp>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::closed_loop
{

// What one closed-loop run is judged on. The runner that drives the simulation writes it as a
// scene file (JSON, see README.md) next to the run's bag.
struct ClosedLoopScene
{
  std::string scene_id;
  // The scene's manoeuvre, e.g. lane_change, object_avoidance, pull_out, pedestrian_yield.
  std::string eval_label;
  // easy, normal or hard; empty when the scene has no difficulty.
  std::string difficulty;
  // The scene window, and the end of the run (at or after the window's start), in the clock of
  // the evaluated bag. Rows start at window_start; collisions count until run_end.
  rclcpp::Time window_start{0, 0, RCL_ROS_TIME};
  rclcpp::Time window_end{0, 0, RCL_ROS_TIME};
  rclcpp::Time run_end{0, 0, RCL_ROS_TIME};
  // The bag that holds the recorded (human) drive; empty: the evaluated bag itself.
  std::string reference_bag_path;
  // A time of the recorded drive plus this offset is the same moment in the evaluated bag.
  std::int64_t reference_time_offset_ns{0};
  // The offset is found by lining the recorded drive up with the evaluated one where the simulation
  // replays it, the reference_match_s before the window's start (the scene file's
  // "time_offset_ns": "auto").
  bool reference_time_offset_auto{false};
  double reference_match_s{5.0};
  // The window and the run's end are in the recorded drive's clock (the scene file's
  // "window_clock": "recorded"); they move to the evaluated bag's with the offset.
  bool window_in_recorded_clock{false};

  // The margin metric's scene facts (the scene file's optional "margin" object). Without it, the
  // kind follows eval_label (lane_change, object_avoidance, pull_out) and an easy scene counts
  // every road user.
  std::optional<std::string> margin_kind;
  bool margin_trailing_vehicle{false};
  std::optional<bool> margin_every_road_user;
  std::vector<std::string> ignored_track_ids;
};

// Throws std::invalid_argument naming the first missing or wrong field.
ClosedLoopScene parse_closed_loop_scene(const nlohmann::json & scene);

// Reads and parses a scene file; throws std::runtime_error when it cannot be read.
ClosedLoopScene load_closed_loop_scene(const std::string & path);

nlohmann::json to_json(const ClosedLoopScene & scene);

}  // namespace autoware::planning_data_analyzer::closed_loop

#endif  // CLOSED_LOOP__CLOSED_LOOP_SCENE_HPP_

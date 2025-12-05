// Copyright 2025 TIER IV, Inc.
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

#ifndef PLANNING_DEBUG_TOOLS__PERCEPTION_REPPLAYER_HPP_
#define PLANNING_DEBUG_TOOLS__PERCEPTION_REPPLAYER_HPP_

#include "perception_replayer_common.hpp"
#include "time_manager_widget.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::planning_debug_tools
{

class PerceptionReplayer : public PerceptionReplayerCommon
{
public:
  explicit PerceptionReplayer(
    const PerceptionReplayerCommonParam & param, const rclcpp::NodeOptions & node_options);

private:
  void on_set_rate(const QString & rate_text);
  void on_timer();

  // GUI
  TimeManagerWidget * widget;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // State
  double rate = 1.0;
  const double delta_time = 0.1;  // [s] 10 Hz
};
}  // namespace autoware::planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__PERCEPTION_REPPLAYER_HPP_
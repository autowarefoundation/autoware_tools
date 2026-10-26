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

#include "perception_replayer.hpp"

namespace autoware::planning_debug_tools
{

PerceptionReplayer::PerceptionReplayer(
  const PerceptionReplayerCommonParam & param, const rclcpp::NodeOptions & node_options)
: PerceptionReplayerCommon(param, "perception_replayer", node_options)
{
  // Initialize widget with rosbag timestamps
  widget = new TimeManagerWidget(get_bag_start_time(), get_bag_end_timestamp());
  widget->show();

  // Connect signals
  for (auto * rate_button : widget->rate_buttons) {
    QObject::connect(rate_button, &QPushButton::clicked, [this, rate_button]() {
      on_set_rate(rate_button->text());
    });
  }

  QObject::connect(widget->pub_recorded_ego_pose_button, &QPushButton::clicked, [this]() {
    publish_recorded_ego_pose(widget->get_slider_timestamp());
  });

  QObject::connect(
    widget->pub_goal_pose_button, &QPushButton::clicked, [this]() { publish_goal_pose(); });

  QObject::connect(widget, &TimeManagerWidget::windowClosed, []() { rclcpp::shutdown(); });

  // Start timer callback
  timer = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(static_cast<int>(delta_time * 1000)),
    std::bind(&PerceptionReplayer::on_timer, this), callback_group_check_perception_);

  RCLCPP_INFO(get_logger(), "Start timer callback");
}

void PerceptionReplayer::on_timer()
{
  // get current timestamp once at the beginning for consistent synchronization
  const auto current_timestamp = this->get_clock()->now();

  rclcpp::Time bag_timestamp = widget->get_slider_timestamp();

  // Step timestamp if not paused
  if (!widget->pause_button->isChecked()) {
    bag_timestamp += rclcpp::Duration::from_seconds(delta_time * rate);

    // Stop at the end instead of wrapping around
    const auto bag_end_timestamp = get_bag_end_timestamp();
    if (bag_timestamp >= bag_end_timestamp) {
      bag_timestamp = bag_end_timestamp;
    }

    // Update slider value from the updated timestamp
    widget->set_slider_timestamp(bag_timestamp);
  }

  publish_topics_at_timestamp(bag_timestamp, current_timestamp, false);
}

void PerceptionReplayer::on_set_rate(const QString & rate_text)
{
  bool ok;
  double new_rate = rate_text.toDouble(&ok);
  if (ok) {
    rate = new_rate;
  }
}

}  // namespace autoware::planning_debug_tools

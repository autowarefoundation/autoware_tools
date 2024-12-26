// Copyright 2024 TIER IV, Inc.
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

#ifndef STRING_VIEWER_PANEL_HPP_
#define STRING_VIEWER_PANEL_HPP_

#include <QComboBox>
#include <QLabel>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

namespace tier4_string_viewer_rviz_plugin
{

using autoware_internal_debug_msgs::msg::StringStamped;

class QLineEdit;

class StringViewerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StringViewerPanel(QWidget * parent = nullptr);

  void onInitialize() override;

  void save(rviz_common::Config config) const override;

  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void on_topic_name(const QString & topic);

private:
  void on_string(const StringStamped::ConstSharedPtr msg);

  void on_timer();

  QLabel * contents_;

  QComboBox * topic_list_;

  QString default_topic_;

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<StringStamped>::SharedPtr sub_string_;

  size_t topic_num_{0L};
};
}  // namespace tier4_string_viewer_rviz_plugin

#endif  // STRING_VIEWER_PANEL_HPP_

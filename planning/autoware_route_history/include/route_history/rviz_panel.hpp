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

#ifndef ROUTE_HISTORY__RVIZ_PANEL_HPP_
#define ROUTE_HISTORY__RVIZ_PANEL_HPP_

#include "node_logic.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "std_msgs/msg/string.hpp"

#include <qpushbutton.h>

#include <memory>
#include <string>

namespace autoware::route_history
{
class RouteHistoryPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RouteHistoryPanel(QWidget * parent = nullptr);
  ~RouteHistoryPanel() override;
  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Node::SharedPtr pose_replay_node_ptr_;
  std::unique_ptr<autoware::route_history::NodeLogic> node_abstract_;

  QVBoxLayout * dynamic_layout_{nullptr};

private Q_SLOTS:
  void route_entry_factory(const std::string &, const std::string &);
  void sync_read();
  void clear_layout(QLayout *);
  void load_save_file_button_activated();
  void rename_route_button_activated(const std::string &, const std::string &);
};
}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__RVIZ_PANEL_HPP_

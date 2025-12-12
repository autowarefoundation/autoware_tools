//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef RTC_MANAGER_PANEL_HPP_
#define RTC_MANAGER_PANEL_HPP_

#include "rtc_auto_mode_table.hpp"
#include "rtc_command_button.hpp"
#include "rtc_status_table_widget.hpp"
#include "rtc_utils.hpp"

#include <QCheckBox>
#include <QWidget>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <memory>
#endif

namespace rviz_plugins
{

/**
 * @brief Main RTC Manager Panel for RViz
 */
class RTCManagerPanel : public rviz_common::Panel
{
  Q_OBJECT

public Q_SLOTS:
  void on_click_activate_front();
  void on_toggle_auto_mode_panel(bool checked);

public:  // NOLINT
  explicit RTCManagerPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  void on_rtc_status(const CooperateStatusArray::ConstSharedPtr msg);
  void on_auto_mode_status(const AutoModeStatusArray::ConstSharedPtr msg);
  void send_command_to(const CooperateStatus & status, uint8_t command);
  void update_front_module_button();
  const CooperateStatus * find_activatable_module() const;

  // ROS
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr sub_rtc_status_;
  rclcpp::Subscription<AutoModeStatusArray>::SharedPtr sub_auto_mode_status_;
  rclcpp::Client<CooperateCommands>::SharedPtr client_rtc_commands_;

  // Data
  std::shared_ptr<CooperateStatusArray> cooperate_statuses_ptr_;

  // UI components
  QCheckBox * show_auto_mode_panel_checkbox_ = {nullptr};
  RTCAutoModeTable * auto_mode_table_ = {nullptr};
  RTCStatusTableWidget * status_table_ = {nullptr};
  RTCCommandButton * activate_button_ = {nullptr};

  static constexpr char AUTO_MODE_SERVICE_NAMESPACE[] = "/planning/enable_auto_mode";  // NOLINT
};

}  // namespace rviz_plugins

#endif  // RTC_MANAGER_PANEL_HPP_

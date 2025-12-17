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

#include "rtc_manager_panel.hpp"

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{
namespace
{
using std::placeholders::_1;
}  // namespace

RTCManagerPanel::RTCManagerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;

  // Auto mode panel visibility checkbox
  show_auto_mode_panel_checkbox_ = new QCheckBox("Show Auto Mode Panel");
  show_auto_mode_panel_checkbox_->setChecked(true);
  connect(
    show_auto_mode_panel_checkbox_, &QCheckBox::toggled, this,
    &RTCManagerPanel::on_toggle_auto_mode_panel);
  main_layout->addWidget(show_auto_mode_panel_checkbox_);

  // Auto mode table
  auto_mode_table_ = new RTCAutoModeTable(this);
  main_layout->addWidget(auto_mode_table_);

  // Status table
  status_table_ = new RTCStatusTableWidget(this);
  main_layout->addWidget(status_table_);

  // Activate button (text will be updated dynamically)
  activate_button_ = new RTCCommandButton("Activate", bg_green, this);
  activate_button_->set_enabled(false);
  connect(
    activate_button_, &RTCCommandButton::clicked, this, &RTCManagerPanel::on_click_activate_front);
  main_layout->addWidget(activate_button_);

  setLayout(main_layout);
}

void RTCManagerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  client_rtc_commands_ =
    raw_node_->create_client<CooperateCommands>("/api/external/set/rtc_commands");

  auto_mode_table_->initialize_clients(raw_node_, AUTO_MODE_SERVICE_NAMESPACE);

  sub_rtc_status_ = raw_node_->create_subscription<CooperateStatusArray>(
    "/api/external/get/rtc_status", 1, std::bind(&RTCManagerPanel::on_rtc_status, this, _1));

  sub_auto_mode_status_ = raw_node_->create_subscription<AutoModeStatusArray>(
    "/api/external/get/rtc_auto_mode", 1,
    std::bind(&RTCManagerPanel::on_auto_mode_status, this, _1));
}

void RTCManagerPanel::on_toggle_auto_mode_panel(bool checked)
{
  auto_mode_table_->setVisible(checked);
}

void RTCManagerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("show_auto_mode_panel", show_auto_mode_panel_checkbox_->isChecked());
}

void RTCManagerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  bool show_auto_mode_panel = true;
  if (config.mapGetBool("show_auto_mode_panel", &show_auto_mode_panel)) {
    show_auto_mode_panel_checkbox_->setChecked(show_auto_mode_panel);
    auto_mode_table_->setVisible(show_auto_mode_panel);
  }
}

void RTCManagerPanel::on_rtc_status(const CooperateStatusArray::ConstSharedPtr msg)
{
  cooperate_statuses_ptr_ = std::make_shared<CooperateStatusArray>(*msg);
  status_table_->update_statuses(msg);
  update_front_module_button();
}

void RTCManagerPanel::on_auto_mode_status(const AutoModeStatusArray::ConstSharedPtr msg)
{
  auto_mode_table_->update_auto_mode_statuses(msg);
}

const CooperateStatus * RTCManagerPanel::find_activatable_module() const
{
  if (!cooperate_statuses_ptr_) {
    return nullptr;
  }

  const CooperateStatus * front = nullptr;
  for (const auto & status : cooperate_statuses_ptr_->statuses) {
    if (status.start_distance < 0) {
      continue;  // Skip already passed modules
    }
    if (status.safe) {
      continue;  // Skip auto mode modules (no manual activation needed)
    }
    if (status.command_status.type == Command::ACTIVATE) {
      continue;  // Skip modules that are already activated
    }
    if (!front || status.start_distance < front->start_distance) {
      front = &status;
    }
  }
  return front;
}

void RTCManagerPanel::update_front_module_button()
{
  const auto * activatable = find_activatable_module();

  if (activatable) {
    activate_button_->update_module_name(get_module_name(activatable->module.type));
    activate_button_->set_enabled(true);
  } else {
    activate_button_->update_module_name("N/A");
    activate_button_->set_enabled(false);
  }
}

void RTCManagerPanel::send_command_to(const CooperateStatus & status, uint8_t command)
{
  if (!cooperate_statuses_ptr_) {
    return;
  }

  auto request = std::make_shared<CooperateCommands::Request>();
  request->stamp = cooperate_statuses_ptr_->stamp;

  auto cooperate_command = create_command_from_status(status);
  cooperate_command.command.type = command;
  request->commands.emplace_back(cooperate_command);

  client_rtc_commands_->async_send_request(request);
}

void RTCManagerPanel::on_click_activate_front()
{
  const auto * front = find_activatable_module();
  if (!front) {
    return;
  }
  send_command_to(*front, Command::ACTIVATE);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RTCManagerPanel, rviz_common::Panel)

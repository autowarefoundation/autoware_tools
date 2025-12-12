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

#include "rtc_auto_mode_table.hpp"

#include <QVBoxLayout>

#include <string>
namespace rviz_plugins
{

// Define header labels (order must match Column enum)
const QStringList RTCAutoModeTable::HEADER_LABELS = {
  "Module", "ToAutoMode", "ToManualMode", "ApprovalMode"};

RTCAutoModeTable::RTCAutoModeTable(QWidget * parent) : QWidget(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  setup_table();
  layout->addWidget(table_);

  items_.reserve(MODULE_SIZE);
  for (size_t i = 0; i < MODULE_SIZE; ++i) {
    add_module_row(i, static_cast<uint8_t>(i));
  }
}

void RTCAutoModeTable::setup_table()
{
  table_ = new QTableWidget();
  table_->setColumnCount(static_cast<int>(Column::Count));
  table_->setRowCount(static_cast<int>(MODULE_SIZE));
  table_->setHorizontalHeaderLabels(HEADER_LABELS);
  setup_table_headers(table_);
}

void RTCAutoModeTable::add_module_row(size_t index, uint8_t module_type)
{
  const int row = static_cast<int>(index);

  auto * item = new RTCAutoModeItem();
  item->setParent(this);
  item->module_name = get_module_name(module_type);

  // Module name label
  auto * module_label = create_centered_label(item->module_name);
  module_label->setStyleSheet(is_path_change_module(module_type) ? bg_purple : bg_orange);
  table_->setCellWidget(row, static_cast<int>(Column::Module), module_label);

  // Auto mode button
  item->auto_button = new QPushButton("auto mode");
  item->auto_button->setCheckable(true);
  connect(item->auto_button, &QPushButton::clicked, item, &RTCAutoModeItem::on_change_to_auto_mode);
  table_->setCellWidget(row, static_cast<int>(Column::ToAutoMode), item->auto_button);

  // Manual mode button
  item->manual_button = new QPushButton("manual mode");
  item->manual_button->setCheckable(true);
  connect(
    item->manual_button, &QPushButton::clicked, item, &RTCAutoModeItem::on_change_to_manual_mode);
  table_->setCellWidget(row, static_cast<int>(Column::ToManualMode), item->manual_button);

  // Current mode label
  item->mode_label = create_centered_label("INIT");
  table_->setCellWidget(row, static_cast<int>(Column::CurrentMode), item->mode_label);

  items_.emplace_back(item);
}

void RTCAutoModeTable::initialize_clients(
  rclcpp::Node::SharedPtr node, const std::string & service_namespace)
{
  for (auto * item : items_) {
    item->auto_mode_client =
      node->create_client<AutoMode>(service_namespace + "/" + item->module_name);
  }
}

void RTCAutoModeTable::update_auto_mode_statuses(const AutoModeStatusArray::ConstSharedPtr & msg)
{
  for (const auto & status : msg->statuses) {
    const auto module_type = status.module.type;
    if (module_type >= items_.size()) {
      continue;
    }
    items_[module_type]->set_auto_mode(status.is_auto_mode);
  }
}

}  // namespace rviz_plugins

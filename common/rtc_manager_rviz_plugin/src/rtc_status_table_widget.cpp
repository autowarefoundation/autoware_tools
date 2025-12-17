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

#include "rtc_status_table_widget.hpp"

#include <QVBoxLayout>

#include <fmt/format.h>

#include <algorithm>
#include <string>

namespace rviz_plugins
{

// Define header labels (order must match Column enum)
const QStringList RTCStatusTableWidget::HEADER_LABELS = {
  "ID", "Module", "Safe", "Command", "Auto\nMode", "State", "Start\nDistance", "Finish\nDistance"};

RTCStatusTableWidget::RTCStatusTableWidget(QWidget * parent) : QWidget(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  status_count_label_ = new QLabel("Init");
  layout->addWidget(status_count_label_);

  setup_table();
  layout->addWidget(table_);
}

void RTCStatusTableWidget::setup_table()
{
  table_ = new QTableWidget();
  table_->setColumnCount(static_cast<int>(Column::Count));
  table_->setHorizontalHeaderLabels(HEADER_LABELS);
  setup_table_headers(table_);
}

void RTCStatusTableWidget::update_statuses(const CooperateStatusArray::ConstSharedPtr & msg)
{
  table_->clearContents();

  const auto status_count = msg->statuses.size();
  status_count_label_->setText(
    QString::fromStdString("The Number of RTC Statuses: " + std::to_string(status_count)));

  if (msg->statuses.empty()) {
    table_->update();
    return;
  }

  table_->setRowCount(static_cast<int>(status_count));

  // Sort statuses: non-negative start_distance first, then by distance ascending
  auto sorted_statuses = msg->statuses;
  std::sort(sorted_statuses.begin(), sorted_statuses.end(), [](const auto & a, const auto & b) {
    const bool a_non_negative = a.start_distance >= 0;
    const bool b_non_negative = b.start_distance >= 0;
    if (a_non_negative && b_non_negative) {
      return a.start_distance < b.start_distance;
    }
    return a_non_negative > b_non_negative;
  });

  int row = 0;
  for (const auto & status : sorted_statuses) {
    populate_row(row++, status);
  }

  table_->update();
}

void RTCStatusTableWidget::populate_row(int row, const CooperateStatus & status)
{
  // Determine if this row should be dimmed (negative start_distance means already passed)
  const bool is_dimmed = status.start_distance < 0;
  const auto set_cell = [this, row, is_dimmed](Column column, const std::string & text) {
    auto * label = create_centered_label(text);
    if (is_dimmed) {
      label->setStyleSheet(text_dimmed);
    }
    table_->setCellWidget(row, static_cast<int>(column), label);
  };

  // UUID (first byte, zero-padded to 4 digits)
  set_cell(Column::ID, fmt::format("{:04d}", static_cast<int>(status.uuid.uuid.at(0))));

  // Module name
  set_cell(Column::Module, get_module_name(status.module.type));

  // Safe status
  set_cell(Column::Safe, status.safe ? "True" : "False");

  // Command status
  std::string command_text = "NONE";
  if (!status.auto_mode) {
    command_text = (status.command_status.type != 0) ? "ACTIVATE" : "DEACTIVATE";
  }
  set_cell(Column::Command, command_text);

  // Auto mode
  set_cell(Column::AutoMode, status.auto_mode ? "True" : "False");

  // State
  set_cell(Column::State, get_state_name(status.state.type));

  // Start distance
  set_cell(Column::StartDistance, fmt::format("{:.2f}", status.start_distance));

  // Finish distance
  set_cell(Column::FinishDistance, fmt::format("{:.2f}", status.finish_distance));
}

}  // namespace rviz_plugins

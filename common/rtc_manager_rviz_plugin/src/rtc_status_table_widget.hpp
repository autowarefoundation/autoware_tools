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

#ifndef RTC_STATUS_TABLE_WIDGET_HPP_
#define RTC_STATUS_TABLE_WIDGET_HPP_

#include "rtc_utils.hpp"

#include <QLabel>
#include <QStringList>
#include <QTableWidget>
#include <QWidget>

namespace rviz_plugins
{

/**
 * @brief Table widget for displaying RTC status information
 */
class RTCStatusTableWidget : public QWidget
{
  Q_OBJECT

public:
  // Column indices for RTC status table
  enum class Column {
    ID,
    Module,
    Safe,
    Command,
    AutoMode,
    State,
    StartDistance,
    FinishDistance,
    Count  // Must be last
  };

  // Header labels for each column (order must match Column enum)
  static const QStringList HEADER_LABELS;

  explicit RTCStatusTableWidget(QWidget * parent = nullptr);

  void update_statuses(const CooperateStatusArray::ConstSharedPtr & msg);
  QLabel * get_status_count_label() const { return status_count_label_; }

private:
  void setup_table();
  void populate_row(int row, const CooperateStatus & status);

  QTableWidget * table_ = {nullptr};
  QLabel * status_count_label_ = {nullptr};
};

}  // namespace rviz_plugins

#endif  // RTC_STATUS_TABLE_WIDGET_HPP_

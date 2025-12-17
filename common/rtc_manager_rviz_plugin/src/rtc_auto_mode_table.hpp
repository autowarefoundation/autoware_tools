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

#ifndef RTC_AUTO_MODE_TABLE_HPP_
#define RTC_AUTO_MODE_TABLE_HPP_

#include "rtc_auto_mode_item.hpp"
#include "rtc_utils.hpp"

#include <QStringList>
#include <QTableWidget>
#include <QWidget>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#endif

namespace rviz_plugins
{

/**
 * @brief Table widget for managing auto/manual mode for each module
 */
class RTCAutoModeTable : public QWidget
{
  Q_OBJECT

public:
  // Column indices for auto mode table
  enum class Column {
    Module,
    ToAutoMode,
    ToManualMode,
    CurrentMode,
    Count  // Must be last
  };

  // Header labels for each column (order must match Column enum)
  static const QStringList HEADER_LABELS;

  explicit RTCAutoModeTable(QWidget * parent = nullptr);

  void initialize_clients(rclcpp::Node::SharedPtr node, const std::string & service_namespace);
  void update_auto_mode_statuses(const AutoModeStatusArray::ConstSharedPtr & msg);
  QTableWidget * get_table() const { return table_; }
  const std::vector<RTCAutoModeItem *> & get_items() const { return items_; }

private:
  void setup_table();
  void add_module_row(size_t index, uint8_t module_type);

  QTableWidget * table_ = {nullptr};
  std::vector<RTCAutoModeItem *> items_;
};

}  // namespace rviz_plugins

#endif  // RTC_AUTO_MODE_TABLE_HPP_

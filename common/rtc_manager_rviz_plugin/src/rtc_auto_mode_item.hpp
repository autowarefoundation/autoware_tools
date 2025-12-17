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

#ifndef RTC_AUTO_MODE_ITEM_HPP_
#define RTC_AUTO_MODE_ITEM_HPP_

#include "rtc_utils.hpp"

#include <QLabel>
#include <QObject>
#include <QPushButton>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>

#include <string>
#endif

namespace rviz_plugins
{

/**
 * @brief Represents a single module's auto/manual mode control
 */
struct RTCAutoModeItem : public QObject
{
  Q_OBJECT

public Q_SLOTS:
  void on_change_to_auto_mode();
  void on_change_to_manual_mode();

public:  // NOLINT
  void set_auto_mode(bool is_auto_mode) const;

  std::string module_name;
  QPushButton * auto_button = {nullptr};
  QPushButton * manual_button = {nullptr};
  QLabel * mode_label = {nullptr};
  rclcpp::Client<AutoMode>::SharedPtr auto_mode_client;
};

}  // namespace rviz_plugins

#endif  // RTC_AUTO_MODE_ITEM_HPP_

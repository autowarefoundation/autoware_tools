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

#ifndef RTC_UTILS_HPP_
#define RTC_UTILS_HPP_

#include <QLabel>
#include <QString>
#include <QTableWidget>

#ifndef Q_MOC_RUN
#include <tier4_rtc_msgs/msg/auto_mode_status_array.hpp>
#include <tier4_rtc_msgs/msg/command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_response.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/msg/module.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <tier4_rtc_msgs/srv/auto_mode.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

#include <string>
#endif

namespace rviz_plugins
{
// Type aliases for RTC messages
using tier4_rtc_msgs::msg::AutoModeStatusArray;
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::msg::State;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;

// Style constants
inline const QString bg_blue = "background-color: #3dffff;";    // NOLINT
inline const QString bg_yellow = "background-color: #ffff3d;";  // NOLINT
inline const QString bg_purple = "background-color: #9e3dff;";  // NOLINT
inline const QString bg_orange = "background-color: #ff7f00;";  // NOLINT
inline const QString bg_green = "background-color: #3dff3d;";   // NOLINT
inline const QString bg_red = "background-color: #ff3d3d;";     // NOLINT
inline const QString text_dimmed = "color: #999999;";           // NOLINT

// TODO(tanaka): replace this magic number to Module::SIZE
constexpr size_t MODULE_SIZE = 19;  // NOLINT

/**
 * @brief Create a centered QLabel with the given text
 */
QLabel * create_centered_label(const std::string & text);

/**
 * @brief Convert State type to human-readable string
 */
std::string get_state_name(uint8_t state_type);

/**
 * @brief Get the module name string from module type
 */
std::string get_module_name(uint8_t module_type);

/**
 * @brief Check if the module type is a path change module
 */
bool is_path_change_module(uint8_t module_type);

/**
 * @brief Create a CooperateCommand from a CooperateStatus
 */
CooperateCommand create_command_from_status(const CooperateStatus & status);

/**
 * @brief Setup a QTableWidget with hidden vertical header and stretch horizontal header
 */
void setup_table_headers(QTableWidget * table);

}  // namespace rviz_plugins

#endif  // RTC_UTILS_HPP_

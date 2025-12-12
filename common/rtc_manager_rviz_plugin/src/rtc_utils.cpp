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

#include "rtc_utils.hpp"

#include <QHeaderView>

#include <string>

namespace rviz_plugins
{

QLabel * create_centered_label(const std::string & text)
{
  auto * label = new QLabel(QString::fromStdString(text));
  label->setAlignment(Qt::AlignCenter);
  return label;
}

std::string get_state_name(const uint8_t state_type)
{
  switch (state_type) {
    case State::WAITING_FOR_EXECUTION:
      return "Waiting";
    case State::RUNNING:
      return "Running";
    case State::ABORTING:
      return "Aborting";
    case State::SUCCEEDED:
      return "Succeeded";
    case State::FAILED:
      return "Failed";
    default:
      return "NONE";
  }
}

std::string get_module_name(const uint8_t module_type)
{
  switch (module_type) {
    case Module::LANE_CHANGE_LEFT:
      return "lane_change_left";
    case Module::LANE_CHANGE_RIGHT:
      return "lane_change_right";
    case Module::EXT_REQUEST_LANE_CHANGE_LEFT:
      return "external_request_lane_change_left";
    case Module::EXT_REQUEST_LANE_CHANGE_RIGHT:
      return "external_request_lane_change_right";
    case Module::AVOIDANCE_BY_LC_LEFT:
      return "avoidance_by_lane_change_left";
    case Module::AVOIDANCE_BY_LC_RIGHT:
      return "avoidance_by_lane_change_right";
    case Module::AVOIDANCE_LEFT:
      return "static_obstacle_avoidance_left";
    case Module::AVOIDANCE_RIGHT:
      return "static_obstacle_avoidance_right";
    case Module::GOAL_PLANNER:
      return "goal_planner";
    case Module::START_PLANNER:
      return "start_planner";
    case Module::TRAFFIC_LIGHT:
      return "traffic_light";
    case Module::INTERSECTION:
      return "intersection";
    case Module::CROSSWALK:
      return "crosswalk";
    case Module::BLIND_SPOT:
      return "blind_spot";
    case Module::DETECTION_AREA:
      return "detection_area";
    case Module::NO_STOPPING_AREA:
      return "no_stopping_area";
    case Module::OCCLUSION_SPOT:
      return "occlusion_spot";
    case Module::INTERSECTION_OCCLUSION:
      return "intersection_occlusion";
    default:
      return "NONE";
  }
}

bool is_path_change_module(const uint8_t module_type)
{
  switch (module_type) {
    case Module::LANE_CHANGE_LEFT:
    case Module::LANE_CHANGE_RIGHT:
    case Module::EXT_REQUEST_LANE_CHANGE_LEFT:
    case Module::EXT_REQUEST_LANE_CHANGE_RIGHT:
    case Module::AVOIDANCE_BY_LC_LEFT:
    case Module::AVOIDANCE_BY_LC_RIGHT:
    case Module::AVOIDANCE_LEFT:
    case Module::AVOIDANCE_RIGHT:
    case Module::GOAL_PLANNER:
    case Module::START_PLANNER:
      return true;
    default:
      return false;
  }
}

CooperateCommand create_command_from_status(const CooperateStatus & status)
{
  CooperateCommand command;
  command.uuid = status.uuid;
  command.module = status.module;
  command.command = status.command_status;
  return command;
}

void setup_table_headers(QTableWidget * table)
{
  auto * vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  table->setVerticalHeader(vertical_header);

  auto * horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);
  table->setHorizontalHeader(horizontal_header);
}

}  // namespace rviz_plugins

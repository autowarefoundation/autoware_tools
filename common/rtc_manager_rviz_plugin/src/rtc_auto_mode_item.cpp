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

#include "rtc_auto_mode_item.hpp"

#include <memory>

namespace rviz_plugins
{

void RTCAutoModeItem::on_change_to_auto_mode()
{
  auto request = std::make_shared<AutoMode::Request>();
  request->enable = true;
  auto_mode_client->async_send_request(request);
}

void RTCAutoModeItem::on_change_to_manual_mode()
{
  auto request = std::make_shared<AutoMode::Request>();
  request->enable = false;
  auto_mode_client->async_send_request(request);
}

void RTCAutoModeItem::set_auto_mode(bool is_auto_mode) const
{
  if (is_auto_mode) {
    mode_label->setText("AutoMode");
    mode_label->setStyleSheet(bg_blue);
    auto_button->setChecked(true);
    manual_button->setChecked(false);
  } else {
    mode_label->setText("ManualMode");
    mode_label->setStyleSheet(bg_yellow);
    manual_button->setChecked(true);
    auto_button->setChecked(false);
  }
}

}  // namespace rviz_plugins

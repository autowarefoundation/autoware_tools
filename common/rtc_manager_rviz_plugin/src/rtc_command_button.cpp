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

#include "rtc_command_button.hpp"

#include <QHBoxLayout>

#include <string>

namespace rviz_plugins
{

RTCCommandButton::RTCCommandButton(const QString & prefix, const QString & style, QWidget * parent)
: QWidget(parent), prefix_(prefix)
{
  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  button_ = new QPushButton(prefix);
  if (!style.isEmpty()) {
    button_->setStyleSheet(style);
  }
  connect(button_, &QPushButton::clicked, this, &RTCCommandButton::clicked);
  layout->addWidget(button_);
}

void RTCCommandButton::update_module_name(const std::string & module_name)
{
  button_->setText(prefix_ + " " + QString::fromStdString(module_name));
}

void RTCCommandButton::set_enabled(bool enabled)
{
  button_->setEnabled(enabled);
}

}  // namespace rviz_plugins

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

#ifndef RTC_COMMAND_BUTTON_HPP_
#define RTC_COMMAND_BUTTON_HPP_

#include <QPushButton>
#include <QString>
#include <QWidget>

#include <string>

namespace rviz_plugins
{

/**
 * @brief Activate command button
 */
class RTCCommandButton : public QWidget
{
  Q_OBJECT

Q_SIGNALS:
  void clicked();

public:
  RTCCommandButton(
    const QString & prefix, const QString & style = QString(), QWidget * parent = nullptr);

  void update_module_name(const std::string & module_name);
  void set_enabled(bool enabled);

private:
  QString prefix_;
  QPushButton * button_ = {nullptr};
};

}  // namespace rviz_plugins

#endif  // RTC_COMMAND_BUTTON_HPP_

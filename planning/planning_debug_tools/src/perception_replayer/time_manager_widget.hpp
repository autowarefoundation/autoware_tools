// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANNING_DEBUG_TOOLS__TIME_MANAGER_WIDGET_HPP_
#define PLANNING_DEBUG_TOOLS__TIME_MANAGER_WIDGET_HPP_

#include <QGridLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::planning_debug_tools
{

// Custom QSlider that allows jumping to clicked position
// With QSlider, the slider's handle cannot be captured if the mouse cursor is not
// the handle position when pressing the mouse.
class QJumpSlider : public QSlider
{
  Q_OBJECT

public:
  explicit QJumpSlider(Qt::Orientation orientation, QWidget * parent = nullptr);

protected:
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  int mouse_to_value(QMouseEvent * event);

  bool is_mouse_pressed_ = false;
};

class TimeManagerWidget : public QMainWindow
{
  Q_OBJECT

public:
  explicit TimeManagerWidget(
    const rclcpp::Time & start_timestamp, const rclcpp::Time & end_timestamp,
    const int seek_resolution = 1000000, QWidget * parent = nullptr);

  void set_slider_timestamp(rclcpp::Time timestamp);
  rclcpp::Time get_slider_timestamp() const;

  QJumpSlider * slider;
  QPushButton * pause_button;
  std::vector<QPushButton *> rate_buttons;
  QPushButton * pub_recorded_ego_pose_button;
  QPushButton * pub_goal_pose_button;

Q_SIGNALS:
  void windowClosed();

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  int timestamp_to_value(rclcpp::Time timestamp) const;
  rclcpp::Time value_to_timestamp(int value) const;

  const rclcpp::Time start_timestamp_;
  const rclcpp::Time end_timestamp_;
  const int seek_resolution_;

  QWidget * central_widget_;
  QGridLayout * grid_layout_;
};

}  // namespace autoware::planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__TIME_MANAGER_WIDGET_HPP_
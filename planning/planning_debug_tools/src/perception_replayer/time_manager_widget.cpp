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

#include "time_manager_widget.hpp"

#include <QCloseEvent>
#include <QMouseEvent>
#include <QSizePolicy>

#include <vector>

namespace autoware::planning_debug_tools
{

// QJumpSlider implementation
QJumpSlider::QJumpSlider(Qt::Orientation orientation, QWidget * parent)
: QSlider(orientation, parent)
{
}

int QJumpSlider::mouse_to_value(QMouseEvent * event)
{
  int x = event->pos().x();
  return static_cast<int>(maximum() * x / width());
}

void QJumpSlider::mousePressEvent(QMouseEvent * event)
{
  QSlider::mousePressEvent(event);

  if (event->button() == Qt::LeftButton) {
    setValue(mouse_to_value(event));
    is_mouse_pressed_ = true;
  }
}

void QJumpSlider::mouseMoveEvent(QMouseEvent * event)
{
  QSlider::mouseMoveEvent(event);

  if (is_mouse_pressed_) {
    setValue(mouse_to_value(event));
  }
}

void QJumpSlider::mouseReleaseEvent(QMouseEvent * event)
{
  QSlider::mouseReleaseEvent(event);

  if (event->button() == Qt::LeftButton) {
    is_mouse_pressed_ = false;
  }
}

// TimeManagerWidget implementation
TimeManagerWidget::TimeManagerWidget(
  const rclcpp::Time & start_timestamp, const rclcpp::Time & end_timestamp,
  const int seek_resolution, QWidget * parent)
: QMainWindow(parent),
  start_timestamp_(start_timestamp),
  end_timestamp_(end_timestamp),
  seek_resolution_(seek_resolution)
{
  setObjectName("PerceptionReplayer");
  resize(480, 120);
  setWindowFlags(Qt::WindowStaysOnTopHint);

  central_widget_ = new QWidget(this);
  central_widget_->setObjectName("central_widget_");

  grid_layout_ = new QGridLayout(central_widget_);
  grid_layout_->setContentsMargins(10, 10, 10, 10);
  grid_layout_->setObjectName("grid_layout_");

  // Rate buttons
  const std::vector<double> rates = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};
  for (size_t i = 0; i < rates.size(); ++i) {
    QPushButton * rate_button = new QPushButton(QString::number(rates[i]));
    rate_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    rate_buttons.push_back(rate_button);
    grid_layout_->addWidget(rate_button, 0, static_cast<int>(i), 1, 1);
  }

  // Pause button
  pause_button = new QPushButton("pause");
  pause_button->setCheckable(true);
  pause_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  grid_layout_->addWidget(pause_button, 1, 0, 1, -1);

  // Publish recorded ego pose button
  pub_recorded_ego_pose_button = new QPushButton("publish recorded ego pose");
  grid_layout_->addWidget(pub_recorded_ego_pose_button, 2, 0, 1, -1);

  // Publish goal pose button
  pub_goal_pose_button = new QPushButton("publish last recorded ego pose as goal pose");
  grid_layout_->addWidget(pub_goal_pose_button, 3, 0, 1, -1);

  // Slider
  slider = new QJumpSlider(Qt::Horizontal);
  slider->setMinimum(0);
  slider->setMaximum(seek_resolution_);
  slider->setValue(0);
  grid_layout_->addWidget(slider, 4, 0, 1, -1);

  setCentralWidget(central_widget_);
}

int TimeManagerWidget::timestamp_to_value(rclcpp::Time timestamp) const
{
  return static_cast<int>(
    (timestamp - start_timestamp_).seconds() / (end_timestamp_ - start_timestamp_).seconds() *
    slider->maximum());
}

rclcpp::Time TimeManagerWidget::value_to_timestamp(int value) const
{
  return start_timestamp_ + rclcpp::Duration::from_seconds(
                              value / static_cast<double>(slider->maximum()) *
                              (end_timestamp_ - start_timestamp_).seconds());
}

void TimeManagerWidget::set_slider_timestamp(rclcpp::Time timestamp)
{
  slider->setValue(timestamp_to_value(timestamp));
}

rclcpp::Time TimeManagerWidget::get_slider_timestamp() const
{
  return value_to_timestamp(slider->value());
}

void TimeManagerWidget::closeEvent(QCloseEvent * event)
{
  Q_EMIT windowClosed();
  QMainWindow::closeEvent(event);
}

}  // namespace autoware::planning_debug_tools

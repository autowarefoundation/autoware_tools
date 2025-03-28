// Copyright 2021 Tier IV, Inc.
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

#ifndef SCREEN_CAPTURE_PANEL_HPP_
#define SCREEN_CAPTURE_PANEL_HPP_

// Qt
#include <QApplication>
#include <QDesktopWidget>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QScreen>
#include <QSpinBox>
#include <QTimer>

// rviz
#include <opencv2/opencv.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

// ros
#include <tier4_screen_capture_rviz_plugin/srv/capture.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rviz_plugins
{

using tier4_screen_capture_rviz_plugin::srv::Capture;

class AutowareScreenCapturePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareScreenCapturePanel(QWidget * parent = nullptr);
  ~AutowareScreenCapturePanel() override;

  void onInitialize() override;

  void save(rviz_common::Config config) const override;

  void load(const rviz_common::Config & config) override;

public Q_SLOTS:
  void on_click_screen_capture();

  void on_click_video_capture();

  void on_prefix_change() {}

  void on_rate_change(const int rate);

  void on_buffer_size_change(const int buffer_size);

private:
  void create_timer();

  void callback(const Capture::Request::SharedPtr req, const Capture::Response::SharedPtr res);

  bool save_screen_shot(const std::string & file_name);

  bool start_buffering();

  bool start_recording();

  bool save_movie(const std::string & file_name);

  bool save_buffer(const std::string & file_name);

  bool stop_buffering();

  void on_timer();

  void save(const std::deque<cv::Mat> & images, const std::string & file_name);

  void update_buffer_size();

  QLabel * ros_time_label_;
  QPushButton * screen_capture_button_ptr_;
  QPushButton * capture_to_mp4_button_ptr_;
  QLineEdit * file_prefix_;
  QSpinBox * rate_;
  QSpinBox * buffer_size_;
  QMainWindow * main_window_{nullptr};

  cv::Size size_;

  std::deque<cv::Mat> movie_;

  std::deque<cv::Mat> buffer_;

  // Size of the frame buffer (number of frames to keep in memory)
  // At 10 Hz capture rate, 100 frames correspond to approximately 10 seconds of video
  size_t buffer_size_frames_{0};

  bool is_buffering_{false};
  bool is_recording_{false};

  rclcpp::Service<Capture>::SharedPtr srv_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rviz_plugins

#endif  // SCREEN_CAPTURE_PANEL_HPP_

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

#include "screen_capture_panel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>

namespace rviz_plugins
{

using std::placeholders::_1;
using std::placeholders::_2;

void setFormatDate(QLabel * line, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", localtime(&seconds));
  line->setText(QString("- ") + QString(buffer));
}

AutowareScreenCapturePanel::AutowareScreenCapturePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  std::filesystem::create_directory("capture");
  auto * v_layout = new QVBoxLayout;
  // screen capture
  auto * cap_layout = new QHBoxLayout;
  {
    ros_time_label_ = new QLabel;
    screen_capture_button_ptr_ = new QPushButton("Capture Screen Shot");
    connect(screen_capture_button_ptr_, SIGNAL(clicked()), this, SLOT(on_click_screen_capture()));
    file_prefix_ = new QLineEdit("cap");
    connect(file_prefix_, SIGNAL(editingFinished()), this, SLOT(on_prefix_change()));
    cap_layout->addWidget(screen_capture_button_ptr_);
    cap_layout->addWidget(file_prefix_);
    cap_layout->addWidget(ros_time_label_);
    // initialize file name system clock is better for identification.
    setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());
  }

  // video capture
  auto * video_cap_layout = new QHBoxLayout;
  {
    capture_to_mp4_button_ptr_ = new QPushButton("Capture Screen");
    connect(capture_to_mp4_button_ptr_, SIGNAL(clicked()), this, SLOT(on_click_video_capture()));
    rate_ = new QSpinBox();
    rate_->setRange(1, 10);
    rate_->setValue(10);
    rate_->setSingleStep(1);
    connect(rate_, SIGNAL(valueChanged(const int)), this, SLOT(on_rate_change(const int)));
    // video cap layout
    video_cap_layout->addWidget(capture_to_mp4_button_ptr_);
    video_cap_layout->addWidget(rate_);
    video_cap_layout->addWidget(new QLabel(" [Hz]"));
  }

  // buffer size setting
  auto * buffer_size_layout = new QHBoxLayout;
  {
    buffer_size_layout->addWidget(new QLabel("Buffer Size: "));

    buffer_size_ = new QSpinBox();
    buffer_size_->setRange(1, 300);  // Maximum 300 seconds buffer
    buffer_size_->setValue(10);      // Default 10 seconds buffer
    buffer_size_->setSingleStep(1);
    buffer_size_layout->addWidget(buffer_size_);
    buffer_size_layout->addWidget(new QLabel(" [sec]"));

    connect(
      buffer_size_, SIGNAL(valueChanged(const int)), this, SLOT(on_buffer_size_change(const int)));
  }

  // consider layout
  {
    v_layout->addLayout(cap_layout);
    v_layout->addLayout(video_cap_layout);
    v_layout->addLayout(buffer_size_layout);
    setLayout(v_layout);
  }

  // Initialize buffer sizes
  update_buffer_size();
}

void AutowareScreenCapturePanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  srv_ = raw_node_->create_service<Capture>(
    "/debug/capture", std::bind(&AutowareScreenCapturePanel::callback, this, _1, _2));

  create_timer();
}

void AutowareScreenCapturePanel::create_timer()
{
  const auto period = std::chrono::milliseconds(static_cast<int64_t>(1e3 / rate_->value()));
  timer_ = raw_node_->create_wall_timer(period, [&]() { on_timer(); });
}

void AutowareScreenCapturePanel::on_rate_change([[maybe_unused]] const int rate)
{
  timer_->cancel();
  create_timer();
  update_buffer_size();
  RCLCPP_INFO_STREAM(raw_node_->get_logger(), "RATE:" << rate_->value());
}

void AutowareScreenCapturePanel::on_buffer_size_change(const int buffer_size)
{
  update_buffer_size();
  RCLCPP_INFO_STREAM(
    raw_node_->get_logger(),
    "BUFFER SIZE: " << buffer_size << " [sec] with " << buffer_size_frames_ << " frames)");
}

void AutowareScreenCapturePanel::update_buffer_size()
{
  // Calculate buffer sizes in frames
  buffer_size_frames_ = buffer_size_->value() * rate_->value();
}

void AutowareScreenCapturePanel::on_timer()
{
  // update_buffer_size();
  setFormatDate(ros_time_label_, rclcpp::Clock().now().seconds());

  if (!main_window_) return;

  // this is deprecated but only way to capture nicely
  QScreen * screen = QGuiApplication::primaryScreen();
  QPixmap original_pixmap = screen->grabWindow(main_window_->winId());
  const auto q_image =
    original_pixmap.toImage().convertToFormat(QImage::Format_RGB888).rgbSwapped();
  const int h = q_image.height();
  const int w = q_image.width();
  cv::Size size = cv::Size(w, h);
  cv::Mat image(
    size, CV_8UC3, const_cast<uchar *>(q_image.bits()),
    static_cast<size_t>(q_image.bytesPerLine()));

  size_ = size;
  std::cerr << "buffer_size_frames_: " << buffer_size_frames_ << std::endl;
  std::cerr << "image size: " << image.size() << std::endl;
  std::cerr << "buffer size: " << buffer_.size() << std::endl;

  if (is_buffering_) {
    buffer_.push_back(image.clone());
    while (buffer_.size() > buffer_size_frames_) {
      buffer_.pop_front();
    }
  }

  if (is_recording_) {
    movie_.push_back(image.clone());
  }

  cv::waitKey(0);
}

void AutowareScreenCapturePanel::callback(
  const Capture::Request::SharedPtr req, const Capture::Response::SharedPtr res)
{
  if (req->action == Capture::Request::SCREEN_SHOT) {
    res->success = save_screen_shot(req->file_name);
    return;
  }

  if (req->action == Capture::Request::START_BUFFERING) {
    res->success = start_buffering();
    return;
  }

  if (req->action == Capture::Request::STOP_BUFFERING) {
    res->success = stop_buffering();
    return;
  }

  if (req->action == Capture::Request::RECORD) {
    res->success = start_recording();
    return;
  }

  if (req->action == Capture::Request::SAVE) {
    res->success = save_movie(req->file_name);
    return;
  }

  if (req->action == Capture::Request::SAVE_BUFFER) {
    res->success = save_buffer(req->file_name);
    return;
  }

  if (req->action == Capture::Request::SET_BUFFER) {
    if (req->buffer_seconds > 0) {
      buffer_size_->setValue(req->buffer_seconds);
      res->success = true;
    } else {
      res->success = false;
    }
    return;
  }

  res->success = false;
}

void AutowareScreenCapturePanel::on_click_screen_capture()
{
  save_screen_shot(file_prefix_->text().toStdString());
}

void AutowareScreenCapturePanel::on_click_video_capture()
{
  if (is_recording_) {
    save_movie(file_prefix_->text().toStdString());
  } else {
    start_recording();
  }
}

bool AutowareScreenCapturePanel::save_screen_shot(const std::string & file_name)
{
  const std::string time_text = "capture/" + file_name + ros_time_label_->text().toStdString();
  getDisplayContext()->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(
    time_text + ".png");

  return true;
}

bool AutowareScreenCapturePanel::start_recording()
{
  try {
    const QWidgetList top_level_widgets = QApplication::topLevelWidgets();
    for (QWidget * widget : top_level_widgets) {
      auto * main_window_candidate = qobject_cast<QMainWindow *>(widget);
      if (main_window_candidate) {
        main_window_ = main_window_candidate;
      }
    }
  } catch (...) {
    return false;
  }

  if (!main_window_) return false;

  RCLCPP_INFO_STREAM(raw_node_->get_logger(), "START RECORDING.");

  capture_to_mp4_button_ptr_->setText("capturing rviz screen");
  capture_to_mp4_button_ptr_->setStyleSheet("background-color: #FF0000;");

  is_recording_ = true;

  return true;
}

bool AutowareScreenCapturePanel::start_buffering()
{
  try {
    const QWidgetList top_level_widgets = QApplication::topLevelWidgets();
    for (QWidget * widget : top_level_widgets) {
      auto * main_window_candidate = qobject_cast<QMainWindow *>(widget);
      if (main_window_candidate) {
        main_window_ = main_window_candidate;
      }
    }
  } catch (...) {
    return false;
  }

  if (!main_window_) return false;

  RCLCPP_INFO_STREAM(raw_node_->get_logger(), "START BUFFERING.");

  buffer_.clear();
  is_buffering_ = true;

  return true;
}

bool AutowareScreenCapturePanel::stop_buffering()
{
  if (!is_buffering_) return false;

  buffer_.clear();
  is_buffering_ = false;

  return true;
}

bool AutowareScreenCapturePanel::save_movie(const std::string & file_name)
{
  if (!is_recording_) return false;

  RCLCPP_INFO_STREAM(raw_node_->get_logger(), "SAVE RECORDED MOVIE.");

  save(movie_, file_name);

  capture_to_mp4_button_ptr_->setText("waiting for capture");
  capture_to_mp4_button_ptr_->setStyleSheet("background-color: #00FF00;");

  is_recording_ = false;

  movie_.clear();

  return true;
}

bool AutowareScreenCapturePanel::save_buffer(const std::string & file_name)
{
  if (buffer_.empty()) return false;

  if (!is_buffering_) return false;

  RCLCPP_INFO_STREAM(raw_node_->get_logger(), "SAVE BUFFERED MOVIE.");

  save(buffer_, file_name + "_buffered");

  return true;
}

void AutowareScreenCapturePanel::save(
  const std::deque<cv::Mat> & images, const std::string & file_name)
{
  int fourcc = cv::VideoWriter::fourcc('h', '2', '6', '4');  // mp4

  cv::VideoWriter writer;

  writer.open(
    "capture/" + file_name + ros_time_label_->text().toStdString() + ".mp4", fourcc, rate_->value(),
    size_);

  for (const auto & frame : images) {
    cv::Mat resized_frame;
    cv::resize(frame, resized_frame, size_);
    writer.write(resized_frame);
  }

  writer.release();
}

void AutowareScreenCapturePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void AutowareScreenCapturePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

AutowareScreenCapturePanel::~AutowareScreenCapturePanel() = default;

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareScreenCapturePanel, rviz_common::Panel)

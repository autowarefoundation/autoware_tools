// Copyright 2024 TIER IV, Inc.
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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_RVIZ_PLUGIN_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_RVIZ_PLUGIN_HPP_

#include "driving_environment_analyzer/analyzer_core.hpp"
#include "driving_environment_analyzer/type_alias.hpp"

#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{

class DrivingEnvironmentAnalyzerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DrivingEnvironmentAnalyzerPanel(QWidget * parent = nullptr);
  ~DrivingEnvironmentAnalyzerPanel() override;
  void onInitialize() override;

public Q_SLOTS:
  void onBoxUpdate();
  void onSliderUpdate();
  void onClickSetTimeStamp();
  void onSelectBagFile();
  void onSelectDirectory();
  void onClickAnalyzeStaticODDFactor();
  void onClickAnalyzeDynamicODDFactor();

private:
  void onMap(const HADMapBin::ConstSharedPtr map_msg);

  std::shared_ptr<analyzer_core::AnalyzerCore> analyzer_;

  std::ofstream ofs_csv_file_;

  QSpinBox * bag_time_selector_;
  QSlider * bag_time_slider_;
  QLabel * bag_name_label_;
  QLabel * bag_time_line_;
  QPushButton * dir_button_ptr_;
  QPushButton * file_button_ptr_;
  QPushButton * analyze_static_odd_button_;
  QPushButton * analyze_dynamic_odd_button_;
  QPushButton * set_timestamp_btn_;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_static_;
};
}  // namespace driving_environment_analyzer

#endif  // DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_RVIZ_PLUGIN_HPP_

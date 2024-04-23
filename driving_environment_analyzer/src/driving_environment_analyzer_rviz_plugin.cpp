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

#include "driving_environment_analyzer/driving_environment_analyzer_rviz_plugin.hpp"

#include "driving_environment_analyzer/analyzer_core.hpp"

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{
namespace
{
void set_format_time(QLabel * label, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
  label->setText(QString(buffer) + QString::number((time - seconds), 'f', 3).rightRef(4));
}
}  // namespace

DrivingEnvironmentAnalyzerPanel::DrivingEnvironmentAnalyzerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * v_layout = new QVBoxLayout(this);
  {
    auto * layout = new QHBoxLayout(this);

    // add button to load bag file.
    file_button_ptr_ = new QPushButton("Select bag file");
    connect(file_button_ptr_, SIGNAL(clicked()), SLOT(onSelectBagFile()));
    layout->addWidget(file_button_ptr_);

    // add button to load directory.
    dir_button_ptr_ = new QPushButton("Select directory");
    connect(dir_button_ptr_, SIGNAL(clicked()), SLOT(onSelectDirectory()));
    layout->addWidget(dir_button_ptr_);

    v_layout->addLayout(layout);
  }

  {
    auto * layout = new QHBoxLayout(this);

    // add label to show target time.
    layout->addWidget(new QLabel("Bag Time:"));
    bag_time_line_ = new QLabel;
    layout->addWidget(bag_time_line_);

    // add spin box to select target time.
    bag_time_selector_ = new QSpinBox();
    bag_time_selector_->setValue(0);
    bag_time_selector_->setSingleStep(1);
    connect(bag_time_selector_, SIGNAL(valueChanged(int)), SLOT(onBoxUpdate()));
    layout->addWidget(bag_time_selector_);

    // add button to set target time.
    set_timestamp_btn_ = new QPushButton("Set time stamp");
    connect(set_timestamp_btn_, SIGNAL(clicked()), SLOT(onClickSetTimeStamp()));
    layout->addWidget(set_timestamp_btn_);

    // add button to start analyzing dynamic ODD.
    analyze_static_odd_button_ = new QPushButton("Analyze dynamic ODD factor");
    connect(analyze_static_odd_button_, SIGNAL(clicked()), SLOT(onClickAnalyzeDynamicODDFactor()));
    layout->addWidget(analyze_static_odd_button_);

    // add button to start analyzing static ODD.
    analyze_dynamic_odd_button_ = new QPushButton("Analyze static ODD factor");
    connect(analyze_dynamic_odd_button_, SIGNAL(clicked()), SLOT(onClickAnalyzeStaticODDFactor()));
    layout->addWidget(analyze_dynamic_odd_button_);

    v_layout->addLayout(layout);
  }

  {
    bag_time_slider_ = new QSlider();
    bag_time_slider_->setOrientation(Qt::Horizontal);
    bag_time_slider_->setValue(0);
    connect(bag_time_slider_, SIGNAL(valueChanged(int)), SLOT(onSliderUpdate()));
    v_layout->addWidget(bag_time_slider_);
  }

  {
    bag_name_label_ = new QLabel();
    bag_name_label_->setAlignment(Qt::AlignLeft);
    auto * layout = new QHBoxLayout(this);
    layout->addWidget(new QLabel("BagFile:"));
    layout->addWidget(bag_name_label_);
    v_layout->addLayout(layout);
  }

  setLayout(v_layout);
}

void DrivingEnvironmentAnalyzerPanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_map_ = raw_node_->create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DrivingEnvironmentAnalyzerPanel::onMap, this, _1));

  pub_odometry_ =
    raw_node_->create_publisher<Odometry>("/localization/kinematic_state", rclcpp::QoS(1));
  pub_objects_ = raw_node_->create_publisher<PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS(1));
  pub_tf_ = raw_node_->create_publisher<TFMessage>("/tf", rclcpp::QoS(1));
  pub_tf_static_ = raw_node_->create_publisher<TFMessage>("/tf_static", rclcpp::QoS(1));

  analyzer_ = std::make_shared<analyzer_core::AnalyzerCore>(*raw_node_);
}

void DrivingEnvironmentAnalyzerPanel::onMap(const HADMapBin::ConstSharedPtr msg)
{
  analyzer_->setMap(*msg);
}

void DrivingEnvironmentAnalyzerPanel::onBoxUpdate()
{
  set_format_time(bag_time_line_, bag_time_selector_->value());
  bag_time_slider_->setValue(bag_time_selector_->value());
}

void DrivingEnvironmentAnalyzerPanel::onSliderUpdate()
{
  set_format_time(bag_time_line_, bag_time_slider_->value());
  bag_time_selector_->setValue(bag_time_slider_->value());
}

void DrivingEnvironmentAnalyzerPanel::onSelectDirectory()
{
  QString file_name = QFileDialog::getExistingDirectory(this, tr("Open ROSBAG file"), "/tmp");
  if (file_name.count() == 0) {
    return;
  }

  analyzer_->setBagFile(file_name.toStdString());

  const auto [start_time, end_time] = analyzer_->getBagStartEndTime();
  bag_time_selector_->setRange(start_time, end_time);
  bag_time_slider_->setRange(start_time, end_time);
  bag_name_label_->setText(file_name);
}

void DrivingEnvironmentAnalyzerPanel::onSelectBagFile()
{
  QString file_name =
    QFileDialog::getOpenFileName(this, tr("Open ROSBAG file"), "/tmp", tr("ROSBAG (*.db3)"));

  if (file_name.count() == 0) {
    return;
  }

  analyzer_->setBagFile(file_name.toStdString());

  const auto [start_time, end_time] = analyzer_->getBagStartEndTime();
  bag_time_selector_->setRange(start_time, end_time);
  bag_time_slider_->setRange(start_time, end_time);
  bag_name_label_->setText(file_name);
}

void DrivingEnvironmentAnalyzerPanel::onClickSetTimeStamp()
{
  analyzer_->clearData();
  analyzer_->setTimeStamp(bag_time_selector_->value());

  if (!analyzer_->isDataReadyForDynamicODDAnalysis()) {
    return;
  }

  pub_odometry_->publish(analyzer_->getOdometry());
  pub_objects_->publish(analyzer_->getObjects());
  pub_tf_->publish(analyzer_->getTF());
  pub_tf_static_->publish(analyzer_->getTFStatic());
}

void DrivingEnvironmentAnalyzerPanel::onClickAnalyzeDynamicODDFactor()
{
  if (!analyzer_->isDataReadyForDynamicODDAnalysis()) {
    return;
  }

  analyzer_->analyzeDynamicODDFactor();
}

void DrivingEnvironmentAnalyzerPanel::onClickAnalyzeStaticODDFactor()
{
  if (!analyzer_->isDataReadyForStaticODDAnalysis()) {
    return;
  }

  analyzer_->analyzeStaticODDFactor();
}

DrivingEnvironmentAnalyzerPanel::~DrivingEnvironmentAnalyzerPanel() = default;
}  // namespace driving_environment_analyzer

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  driving_environment_analyzer::DrivingEnvironmentAnalyzerPanel, rviz_common::Panel)

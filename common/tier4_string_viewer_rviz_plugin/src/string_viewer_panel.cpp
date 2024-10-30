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

#include "string_viewer_panel.hpp"

#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>

#include <ctime>

namespace tier4_string_viewer_rviz_plugin
{

StringViewerPanel::StringViewerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  topic_list_ = new QComboBox();
  layout->addWidget(topic_list_);

  contents_ = new QLabel;
  layout->addWidget(contents_);

  setLayout(layout);

  connect(
    topic_list_, SIGNAL(currentIndexChanged(const QString &)), this,
    SLOT(on_topic_name(const QString &)));
}

void StringViewerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  using namespace std::literals::chrono_literals;
  timer_ = raw_node_->create_wall_timer(1000ms, [&]() { on_timer(); });
}

void StringViewerPanel::on_topic_name(const QString & topic)
{
  contents_->clear();
  sub_string_.reset();
  sub_string_ = raw_node_->create_subscription<StringStamped>(
    topic.toStdString(), rclcpp::QoS{1},
    std::bind(&StringViewerPanel::on_string, this, std::placeholders::_1));
}

void StringViewerPanel::on_string(const StringStamped::ConstSharedPtr msg)
{
  contents_->setText(msg->data.c_str());
  contents_->setWordWrap(true);
}

void StringViewerPanel::on_timer()
{
  const auto std_message_type = rosidl_generator_traits::name<StringStamped>();
  const auto published_topics = raw_node_->get_topic_names_and_types();

  if (published_topics.size() == topic_num_) return;

  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    for (const auto & type : topic.second) {
      if (type == std_message_type) {
        topic_list_->addItem(QString::fromStdString(topic.first));
      }
    }
  }

  topic_num_ = published_topics.size();
}
}  // namespace tier4_string_viewer_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_string_viewer_rviz_plugin::StringViewerPanel, rviz_common::Panel)

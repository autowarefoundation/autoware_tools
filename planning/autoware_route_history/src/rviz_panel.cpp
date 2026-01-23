// Copyright 2026 TIER IV, Inc.
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

#include "route_history/rviz_panel.hpp"

#include "route_history/node_logic.hpp"

#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include "std_msgs/msg/string.hpp"

#include <qboxlayout.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;

namespace autoware::route_history
{
RouteHistoryPanel::RouteHistoryPanel(QWidget * parent) : Panel(parent)
{
  const auto main_layout = new QVBoxLayout(this);
  main_layout->setAlignment(Qt::AlignTop);

  const auto title_layout = new QHBoxLayout(this);
  const auto main_title = new QLabel("Route history");
  const auto save_btn = new QPushButton("Save current route");
  const auto load_save_btn = new QPushButton("Load save file");

  QObject::connect(save_btn, &QPushButton::released, this, [this]() {
    node_abstract_->save_route();
    sync_read();
  });
  QObject::connect(
    load_save_btn, &QPushButton::released, this, [this]() { load_save_file_button_activated(); });

  title_layout->addWidget(main_title);
  title_layout->addWidget(load_save_btn);
  title_layout->addWidget(save_btn);

  dynamic_layout_ = new QVBoxLayout(this);
  dynamic_layout_->setAlignment(Qt::AlignTop);
  main_layout->addLayout(title_layout);
  main_layout->addLayout(dynamic_layout_);
}

RouteHistoryPanel::~RouteHistoryPanel() = default;

void RouteHistoryPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  pose_replay_node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  node_abstract_ = std::make_unique<autoware::route_history::NodeLogic>(pose_replay_node_ptr_);
  sync_read();
}

void RouteHistoryPanel::sync_read()
{
  clear_layout(dynamic_layout_);
  std::vector<autoware::route_history::UuidName> fetched_routes = node_abstract_->get_routes({});
  for (auto & r : fetched_routes) {
    RouteHistoryPanel::route_entry_factory(r.uuid, r.name);
  }
}

void RouteHistoryPanel::route_entry_factory(const std::string & uuid, const std::string & name)
{
  auto layout = new QHBoxLayout();
  auto label = new QLabel(QString::fromStdString(name));
  auto load_btn = new QPushButton("Load");
  QObject::connect(
    load_btn, &QPushButton::released, this, [this, uuid]() { node_abstract_->load_route(uuid); });
  auto rename_btn = new QPushButton("Rename");
  QObject::connect(rename_btn, &QPushButton::released, this, [this, name, uuid]() {
    rename_route_button_activated(name, uuid);
  });
  auto delete_btn = new QPushButton("Delete");
  QObject::connect(delete_btn, &QPushButton::released, this, [this, uuid]() {
    node_abstract_->delete_route(uuid);
    sync_read();
  });

  layout->addWidget(label);
  layout->addWidget(load_btn);
  layout->addWidget(rename_btn);
  layout->addWidget(delete_btn);
  dynamic_layout_->addLayout(layout);
}

void RouteHistoryPanel::load_save_file_button_activated()
{
  bool ok;
  std::string edit_title =
    "Current save file: " + node_abstract_->get_save_path() + "\n" + "Enter save file path:";
  QString new_path = QInputDialog::getText(
    this, tr("Load save"), tr(edit_title.c_str()), QLineEdit::Normal, "", &ok);
  if (ok && !new_path.isEmpty()) {
    node_abstract_->set_save_path(new_path.toStdString());
    sync_read();
  }
}

void RouteHistoryPanel::rename_route_button_activated(
  const std::string & name, const std::string & uuid)
{
  bool ok;
  QString currentName = QString::fromStdString(name);
  QString newName = QInputDialog::getText(
    this, tr("Rename"), tr("Enter new route name:"), QLineEdit::Normal, currentName, &ok);
  if (ok && !newName.isEmpty()) {
    node_abstract_->set_name(uuid, newName.toStdString());
    sync_read();
  }
}

void RouteHistoryPanel::clear_layout(QLayout * layout)
{
  if (!layout) return;
  QLayoutItem * item;
  while ((item = layout->takeAt(0)) != nullptr) {
    if (QWidget * widget = item->widget()) {
      widget->deleteLater();  // Schedule deletion of the widget
    } else if (QLayout * childLayout = item->layout()) {
      clear_layout(childLayout);  // Recursively clear nested layouts
    }
    delete item;  // Delete the layout item itself
  }
}

}  // namespace autoware::route_history
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::route_history::RouteHistoryPanel, rviz_common::Panel)

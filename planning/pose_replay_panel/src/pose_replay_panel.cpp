#include "pose_replay_panel/pose_replay_node_abstract.hpp"

#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <pose_replay_panel/pose_replay_panel.hpp>
#include <rviz_common/display_context.hpp>

#include "std_msgs/msg/string.hpp"
#include <qboxlayout.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::literals::chrono_literals;
 
namespace pose_replay_panel
{

PoseReplayPanel::PoseReplayPanel(QWidget * parent) : Panel(parent)
{
  const auto main_layout = new QVBoxLayout(this);
  main_layout->setAlignment(Qt::AlignTop);

  const auto title_layout = new QHBoxLayout(this);
  const auto main_title = new QLabel("Route history");
  const auto save_btn = new QPushButton("Save current route");
  title_layout->addWidget(main_title);
  title_layout->addWidget(save_btn);

  QObject::connect(save_btn, &QPushButton::released, this, [this]() {
    node_abstract_->save_route();
    sync_read();
  });

  dynamic_layout_ = new QVBoxLayout(this);
  dynamic_layout_->setAlignment(Qt::AlignTop);
  main_layout->addLayout(title_layout);
  main_layout->addLayout(dynamic_layout_);
}

PoseReplayPanel::~PoseReplayPanel() = default;

void PoseReplayPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  pose_replay_node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  node_abstract_ = std::make_unique<pose_replay::PoseReplayNode>(pose_replay_node_ptr_);
  sync_read();
}

void PoseReplayPanel::sync_read()
{
  clear_layout(dynamic_layout_);
  std::vector<pose_replay::UuidName> fetched_routes = node_abstract_->get_routes({});
  for (auto & r : fetched_routes) {
    PoseReplayPanel::route_entry_factory(r.uuid, r.name);
  }
}

void PoseReplayPanel::route_entry_factory(const std::string & uuid, const std::string & name)
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

void PoseReplayPanel::rename_route_button_activated(
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

void PoseReplayPanel::clear_layout(QLayout * layout)
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

}  // namespace pose_replay_panel
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pose_replay_panel::PoseReplayPanel, rviz_common::Panel)

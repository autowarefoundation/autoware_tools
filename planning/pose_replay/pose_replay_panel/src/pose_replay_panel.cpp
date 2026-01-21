#include "pose_replay_interfaces/srv/delete_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_name.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"
#include "pose_replay_panel/pose_replay_logic.hpp"

#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QVBoxLayout>
#include <pose_replay_panel/pose_replay_panel.hpp>
#include <rviz_common/display_context.hpp>

#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::literals::chrono_literals;

namespace pose_replay_panel
{
using std::string;

PoseReplayPanel::PoseReplayPanel(QWidget * parent) : Panel(parent)
{
  const auto main_layout = new QVBoxLayout(this);
  main_layout->setAlignment(Qt::AlignTop);
  const auto main_title = new QLabel("Route history");
  dynamic_layout_ = new QVBoxLayout(this);
  dynamic_layout_->setAlignment(Qt::AlignTop);
  main_layout->addWidget(main_title);
  main_layout->addLayout(dynamic_layout_);
}

PoseReplayPanel::~PoseReplayPanel() = default;

void PoseReplayPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  //// new
  pose_replay_node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  logic_ = std::make_unique<PoseReplay::PoseReplayLogic>(pose_replay_node_ptr_);
  ////

  // get_routes_client_ = node->create_client<pose_replay_interfaces::srv::GetUuidRoutes>(
  //   "/pose_replay/get_routes_service");
  // set_route_client_ =
  //   node->create_client<pose_replay_interfaces::srv::SetRoute>("/pose_replay/set_route_service");
  // delete_route_client_ = node->create_client<pose_replay_interfaces::srv::DeleteRoute>(
  //   "/pose_replay/delete_route_service");
  // set_name_client_ =
  //   node->create_client<pose_replay_interfaces::srv::SetName>("/pose_replay/set_name_service");
  // sync_notif_subscriber_ = node->create_subscription<std_msgs::msg::String>(
  //   "/pose_replay/update", 10,
  //   std::bind(&PoseReplayPanel::sync_notif_callback, this, std::placeholders::_1));

  sync_read();
}

void PoseReplayPanel::sync_read()
{
  clear_layout(dynamic_layout_);
  std::vector<PoseReplay::UuidName> fetched_routes = logic_->get_routes({});
  for (auto & r : fetched_routes) {
    PoseReplayPanel::route_entry_factory(r.uuid, r.name);
  }
}

void PoseReplayPanel::route_entry_factory(const string & uuid, const std::string & name)
{
  auto layout = new QHBoxLayout();
  auto label = new QLabel(QString::fromStdString(name));
  auto load_btn = new QPushButton("Load");
  QObject::connect(
    load_btn, &QPushButton::released, this, [this, uuid]() { set_route_button_activated(uuid); });
  auto rename_btn = new QPushButton("Rename");
  QObject::connect(rename_btn, &QPushButton::released, this, [this, name, uuid]() {
    rename_route_button_activated(name, uuid);
  });
  auto delete_btn = new QPushButton("Delete");
  QObject::connect(delete_btn, &QPushButton::released, this, [this, uuid]() {
    delete_route_button_activated(uuid);
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
    logic_->set_name(uuid, newName.toStdString());
    sync_read();
  }
}

void PoseReplayPanel::delete_route_button_activated(const std::string & uuid)
{
  logic_->delete_route(uuid);
  sync_read();
}

void PoseReplayPanel::set_route_button_activated(const std::string & uuid)
{
  logic_->load_route(uuid);
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

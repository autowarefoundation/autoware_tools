#include "pose_replay_interfaces/srv/delete_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_name.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"

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
  //  sync_button_ = new QPushButton("Sync");
  dynamic_layout_ = new QVBoxLayout(this);
  dynamic_layout_->setAlignment(Qt::AlignTop);
  //  main_layout->addWidget(sync_button_);
  main_layout->addWidget(main_title);
  main_layout->addLayout(dynamic_layout_);

  //  QObject::connect(syncbutton_, &QPushButton::released, this,
  //  &PoseReplayPanel::buttonActivated);
}

PoseReplayPanel::~PoseReplayPanel() = default;

void PoseReplayPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  get_routes_client_ = node->create_client<pose_replay_interfaces::srv::GetUuidRoutes>(
    "/pose_replay/get_routes_service");
  set_route_client_ =
    node->create_client<pose_replay_interfaces::srv::SetRoute>("/pose_replay/set_route_service");
  delete_route_client_ = node->create_client<pose_replay_interfaces::srv::DeleteRoute>(
    "/pose_replay/delete_route_service");
  set_name_client_ =
    node->create_client<pose_replay_interfaces::srv::SetName>("/pose_replay/set_name_service");
  sync_notif_subscriber_ = node->create_subscription<std_msgs::msg::String>(
    "/pose_replay/update", 10,
    std::bind(&PoseReplayPanel::sync_notif_callback, this, std::placeholders::_1));

  sync_read();
}

void PoseReplayPanel::sync_read()
{
  clear_layout(dynamic_layout_);

  auto request = std::make_shared<pose_replay_interfaces::srv::GetUuidRoutes::Request>();

  if (!get_routes_client_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("GetUuidRoutes"), "GetUuidRoutes service is not ready.");
    return;
  }

  get_routes_client_->async_send_request(
    request,
    [this](rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoutes>::SharedFuture future) {
      try {
        const auto & response = future.get();
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            auto routes = response->routes.routes;
            for (auto & r : routes) {
              PoseReplayPanel::route_entry_factory(r.uuid, r.name);
            }
          },
          Qt::QueuedConnection);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
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
    auto request = std::make_shared<pose_replay_interfaces::srv::SetName::Request>();
    request->uuid = uuid;
    request->name = newName.toStdString();

    if (!set_name_client_->service_is_ready()) {
      RCLCPP_WARN(rclcpp::get_logger("SetNameService"), "SetName service is not ready.");
      return;
    }

    set_name_client_->async_send_request(
      request, [this](rclcpp::Client<pose_replay_interfaces::srv::SetName>::SharedFuture future) {
        try {
          const auto & response = future.get();

          // Update the UI safely (in Qt thread)
          QMetaObject::invokeMethod(
            this,
            [this, response]() {
              RCLCPP_INFO(rclcpp::get_logger("PoseReplayPanel"), "%s", response->uuid.c_str());
            },
            Qt::QueuedConnection);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
        }
      });
  }
}

void PoseReplayPanel::delete_route_button_activated(const std::string & uuid)
{
  auto request = std::make_shared<pose_replay_interfaces::srv::DeleteRoute::Request>();
  request->uuid = uuid;

  if (!delete_route_client_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("DeleteRouteService"), "DeleteRoute service is not ready.");
    return;
  }

  delete_route_client_->async_send_request(
    request, [this](rclcpp::Client<pose_replay_interfaces::srv::DeleteRoute>::SharedFuture future) {
      try {
        const auto & response = future.get();

        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            RCLCPP_INFO(rclcpp::get_logger("PoseReplayPanel"), response->uuid.c_str());
          },
          Qt::QueuedConnection);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}

void PoseReplayPanel::set_route_button_activated(const std::string & uuid)
{
  auto request = std::make_shared<pose_replay_interfaces::srv::SetRoute::Request>();
  request->uuid = uuid;

  if (!set_route_client_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("SetRouteService"), "SetRoute service is not ready.");
    return;
  }

  set_route_client_->async_send_request(
    request, [this](rclcpp::Client<pose_replay_interfaces::srv::SetRoute>::SharedFuture future) {
      try {
        const auto & response = future.get();

        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            //            syncbutton_->setText(QString(response->start.position.x));
          },
          Qt::QueuedConnection);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}

void PoseReplayPanel::sync_notif_callback(std_msgs::msg::String msg)
{
  RCLCPP_INFO(rclcpp::get_logger("PoseReplayPanel"), "%s notification received", msg.data.c_str());
  sync_read();
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

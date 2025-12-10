#include <pose_replay_panel/pose_replay_panel.hpp>
#include <QVBoxLayout>
#include <QString>

#include <rviz_common/display_context.hpp>

#include "pose_replay_interfaces/srv/get_uuid_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"
#include "pose_replay_interfaces/srv/delete_route.hpp"

#include <chrono>
#include <string>
#include <functional>

using namespace std::literals::chrono_literals;


namespace pose_replay_panel
{
PoseReplayPanel::PoseReplayPanel(QWidget* parent) : Panel(parent)
{
  const auto mainlayout = new QVBoxLayout(this);
  syncbutton_ = new QPushButton("Sync");
  	
  dynamiclayout_ = new QVBoxLayout(this);

  mainlayout->addWidget(syncbutton_);
  mainlayout->addLayout(dynamiclayout_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the buttonActivated callback being called.
  QObject::connect(syncbutton_, &QPushButton::released, this, &PoseReplayPanel::buttonActivated);
}

PoseReplayPanel::~PoseReplayPanel() = default;

void PoseReplayPanel::onInitialize() 
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  // Create a String subscription and bind it to the topicCallback inside this class.
  client_ = node->create_client<pose_replay_interfaces::srv::GetUuidRoute>("get_route_service");
  
  setrouteclient_ = node->create_client<pose_replay_interfaces::srv::SetRoute>("set_route_service");
  
  deleterouteclient_ = node->create_client<pose_replay_interfaces::srv::DeleteRoute>("delete_route_service");

  // Get all Routes
  initclient_ = node->create_client<pose_replay_interfaces::srv::GetUuidRoutes>("get_routes_service");
  syncRead();
}

void PoseReplayPanel::clearLayout(QLayout *layout)
{
    if (!layout)
        return;

    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != nullptr) {
        if (QWidget *widget = item->widget()) {
            widget->deleteLater(); // Schedule deletion of the widget
        } else if (QLayout *childLayout = item->layout()) {
            clearLayout(childLayout); // Recursively clear nested layouts
        }
        delete item; // Delete the layout item itself
    }
}

void PoseReplayPanel::syncRead()
{
  clearLayout(dynamiclayout_);

  auto request = std::make_shared<pose_replay_interfaces::srv::GetUuidRoutes::Request>();
  
  if (!initclient_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("GetUuidRoutes"), "Service not ready");
    return;
  }

  initclient_->async_send_request(
    request,
    [this](rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoutes>::SharedFuture future) {
      try {
        auto response = future.get();
        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
		auto routes = response->routes.routes;
  	  	for(auto r : routes){
	  		PoseReplayPanel::routeEntryFactory(r.uuid);
    		} 
          },
          Qt::QueuedConnection);
      }
      catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}

void PoseReplayPanel::routeEntryFactory(const std::string& uuid)
{
	auto layout = new QHBoxLayout();
	auto label = new QLabel(QString::fromStdString(uuid));

	auto loadbtn = new QPushButton("load");
  	QObject::connect(
		loadbtn, 
		&QPushButton::released, 
		this, 
		std::bind(&PoseReplayPanel::setRouteButtonActivated, this, uuid)
	);

	auto deletebtn = new QPushButton("delete");
  	QObject::connect(
		deletebtn, 
		&QPushButton::released, 
		this, 
		std::bind(&PoseReplayPanel::deleteRouteButtonActivated, this, uuid)
	);

	layout->addWidget(label);
	layout->addWidget(loadbtn);
	layout->addWidget(deletebtn);
	dynamiclayout_->addLayout(layout);
}


void PoseReplayPanel::deleteRouteButtonActivated(std::string& uuid)
{
  auto request = std::make_shared<pose_replay_interfaces::srv::DeleteRoute::Request>();
  request->uuid = uuid;

  if (!deleterouteclient_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("PoseReplayPanel"), "Service not ready");
    return;
  }

  deleterouteclient_->async_send_request(
    request,
    [this](rclcpp::Client<pose_replay_interfaces::srv::DeleteRoute>::SharedFuture future) {
      try {
        auto response = future.get();

        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
		///
		RCLCPP_INFO(rclcpp::get_logger("PoseReplayPanel"), response->uuid.c_str());
		syncRead();
		///
          },
          Qt::QueuedConnection);
      }
      catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}

void PoseReplayPanel::setRouteButtonActivated(std::string& uuid)
{
  auto request = std::make_shared<pose_replay_interfaces::srv::SetRoute::Request>();
  request->uuid = uuid;

  if (!setrouteclient_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("PoseReplayPanel"), "Service not ready");
    return;
  }

  setrouteclient_->async_send_request(
    request,
    [this](rclcpp::Client<pose_replay_interfaces::srv::SetRoute>::SharedFuture future) {
      try {
        auto response = future.get();

        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
//            syncbutton_->setText(QString(response->start.position.x));
          },
          Qt::QueuedConnection);
      }
      catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}

void PoseReplayPanel::buttonActivated()
{
  auto request = std::make_shared<pose_replay_interfaces::srv::GetUuidRoute::Request>();
  request->uuid = "1c51ec25-f14c-4ce2-bd41-a411fa781917";

  if (!client_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("PoseReplayPanel"), "Service not ready");
    return;
  }

  client_->async_send_request(
    request,
    [this](rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoute>::SharedFuture future) {
      try {
        auto response = future.get();

        // Update the UI safely (in Qt thread)
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
//            syncbutton_->setText(QString(response->start.position.x));
          },
          Qt::QueuedConnection);
      }
      catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("PoseReplayPanel"), "Service call failed: %s", e.what());
      }
    });
}


}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pose_replay_panel::PoseReplayPanel, rviz_common::Panel)

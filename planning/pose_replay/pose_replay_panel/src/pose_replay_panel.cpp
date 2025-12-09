#include <pose_replay_panel/pose_replay_panel.hpp>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

#include "pose_replay_interfaces/srv/get_uuid_route.hpp"

#include <chrono>
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
  client_ = node->create_client<pose_replay_interfaces::srv::GetUuidRoute>("test_api_route");

  // Get all Uuids
  // Get all Routes
  // Populate dynamic layout with fetched data and buttons
  for(int i = 0; i<3; ++i){
	PoseReplayPanel::routeEntryFactory();
  } 
}

void PoseReplayPanel::routeEntryFactory()
{
	auto layout = new QHBoxLayout();
	auto label = new QLabel("label");
	auto loadbtn = new QPushButton("load");
	auto deletebtn = new QPushButton("delete");
	layout->addWidget(label);
	layout->addWidget(loadbtn);
	layout->addWidget(deletebtn);
	dynamiclayout_->addLayout(layout);
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

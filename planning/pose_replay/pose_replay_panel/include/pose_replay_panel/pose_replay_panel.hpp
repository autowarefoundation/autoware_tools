#ifndef POSE_REPLAY_PANEL_HPP 
#define POSE_REPLAY_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include "pose_replay_interfaces/srv/get_uuid_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"
#include "pose_replay_interfaces/srv/delete_route.hpp"
#include "std_msgs/msg/string.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <string>

namespace pose_replay_panel
{
class PoseReplayPanel
  : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PoseReplayPanel(QWidget * parent = 0);
  ~PoseReplayPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoute>::SharedPtr client_;
  rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoutes>::SharedPtr initclient_;
  rclcpp::Client<pose_replay_interfaces::srv::SetRoute>::SharedPtr setrouteclient_;
  rclcpp::Client<pose_replay_interfaces::srv::DeleteRoute>::SharedPtr deleterouteclient_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr syncnotifsubscriber_;
 
  // QLabel* label_;
  QPushButton* syncbutton_;
  QVBoxLayout* dynamiclayout_;

private Q_SLOTS:
  void buttonActivated();
  void routeEntryFactory(const std::string&);
  void setRouteButtonActivated(std::string&);
  void deleteRouteButtonActivated(std::string&);
  void syncRead();
  void clearLayout(QLayout*);
  void sync_notif_callback(std_msgs::msg::String);
};
}
#endif  

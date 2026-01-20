#ifndef POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_
#define POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_

#include "pose_replay_interfaces/srv/delete_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_name.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "std_msgs/msg/string.hpp"

#include <string>

namespace pose_replay_panel
{
class PoseReplayPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PoseReplayPanel(QWidget * parent = 0);
  ~PoseReplayPanel() override;
  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;

  // new
  rclcpp::Node::SharedPtr pose_replay_node_ptr_;
  //

  rclcpp::Client<pose_replay_interfaces::srv::GetUuidRoutes>::SharedPtr get_routes_client_;
  rclcpp::Client<pose_replay_interfaces::srv::SetRoute>::SharedPtr set_route_client_;
  rclcpp::Client<pose_replay_interfaces::srv::DeleteRoute>::SharedPtr delete_route_client_;
  rclcpp::Client<pose_replay_interfaces::srv::SetName>::SharedPtr set_name_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sync_notif_subscriber_;

  QVBoxLayout * dynamic_layout_;

private Q_SLOTS:
  void button_activated();
  void route_entry_factory(const std::string &, const std::string &);
  void set_route_button_activated(const std::string &);
  void delete_route_button_activated(const std::string &);
  void sync_read();
  void clear_layout(QLayout *);
  void sync_notif_callback(std_msgs::msg::String);
  void rename_route_button_activated(const std::string &, const std::string &);
};
}  // namespace pose_replay_panel
#endif  // POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_

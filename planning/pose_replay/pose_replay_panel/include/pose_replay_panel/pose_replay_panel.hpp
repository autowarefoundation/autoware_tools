#ifndef POSE_REPLAY_PANEL_HPP 
#define POSE_REPLAY_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include "pose_replay_interfaces/srv/get_uuid_route.hpp"
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

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

  // QLabel* label_;
  QPushButton* syncbutton_;
  QVBoxLayout* dynamiclayout_;

private Q_SLOTS:
  void buttonActivated();
  void routeEntryFactory();
};
}
#endif  

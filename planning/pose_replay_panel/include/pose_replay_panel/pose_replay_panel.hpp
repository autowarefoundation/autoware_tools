#ifndef POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_
#define POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_

#include "pose_replay_panel/pose_replay_node_abstract.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "std_msgs/msg/string.hpp"

#include <qpushbutton.h>

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
  rclcpp::Node::SharedPtr pose_replay_node_ptr_;
  std::unique_ptr<pose_replay::PoseReplayNode> node_abstract_;
  QVBoxLayout * dynamic_layout_;

private Q_SLOTS:
  void route_entry_factory(const std::string &, const std::string &);
  void sync_read();
  void clear_layout(QLayout *);
  void load_save_file_button_activated();
  void rename_route_button_activated(const std::string &, const std::string &);
};
}  // namespace pose_replay_panel
#endif  // POSE_REPLAY_PANEL__POSE_REPLAY_PANEL_HPP_

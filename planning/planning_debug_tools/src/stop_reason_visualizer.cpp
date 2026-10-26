// Copyright 2022 Tier IV, Inc.
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

#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace planning_debug_tools
{
using std::placeholders::_1;
using tier4_planning_msgs::msg::StopReason;
using tier4_planning_msgs::msg::StopReasonArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class StopReasonVisualizerNode : public rclcpp::Node
{
public:
  explicit StopReasonVisualizerNode(const rclcpp::NodeOptions & options)
  : Node("stop_reason_visualizer", options)
  {
    pub_stop_reasons_marker_ = create_publisher<MarkerArray>("~/debug/markers", 1);
    sub_stop_reasons_ = create_subscription<StopReasonArray>(
      "/planning/scenario_planning/status/stop_reasons", rclcpp::QoS{1},
      std::bind(&StopReasonVisualizerNode::onStopReasonArray, this, _1));
  }

private:
  void onStopReasonArray(const StopReasonArray::ConstSharedPtr msg)
  {
    using autoware_utils_visualization::append_marker_array;
    using autoware_utils_visualization::create_default_marker;
    using autoware_utils_visualization::create_marker_color;
    using autoware_utils_visualization::create_marker_scale;

    MarkerArray all_marker_array;
    const auto header = msg->header;
    const double offset_z = 1.0;
    for (auto stop_reason : msg->stop_reasons) {
      std::string reason = stop_reason.reason;
      if (reason.empty()) continue;
      const auto current_time = this->now();
      int id = 0;
      MarkerArray marker_array;
      for (auto stop_factor : stop_reason.stop_factors) {
        if (stop_factor.stop_factor_points.empty()) continue;
        std::string prefix = reason + "[" + std::to_string(id) + "]";
        const auto stop_factor_point = stop_factor.stop_factor_points.front();
        // base stop pose marker
        {
          auto stop_point_marker = create_default_marker(
            "map", current_time, prefix + ":stop_factor_point", id, Marker::SPHERE,
            create_marker_scale(0.25, 0.25, 0.25), create_marker_color(1.0, 0.0, 0.0, 0.999));
          stop_point_marker.pose.position = stop_factor_point;
          marker_array.markers.emplace_back(stop_point_marker);
        }
        // attention ! marker
        {
          auto attention_text_marker = create_default_marker(
            "map", current_time, prefix + ":attention text", id,
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
            create_marker_color(1.0, 1.0, 1.0, 0.999));
          attention_text_marker.pose.position = stop_factor_point;
          attention_text_marker.pose.position.z += offset_z;
          attention_text_marker.text = "!";
          marker_array.markers.emplace_back(attention_text_marker);
        }
        // point to pose
        {
          auto stop_to_pose_marker = create_default_marker(
            "map", current_time, prefix + ":stop_to_pose", id, Marker::LINE_STRIP,
            create_marker_scale(0.02, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.999));
          stop_to_pose_marker.points.emplace_back(stop_factor.stop_factor_points.front());
          stop_to_pose_marker.points.emplace_back(stop_factor.stop_pose.position);
          marker_array.markers.emplace_back(stop_to_pose_marker);
        }
        // point to pose
        {
          auto stop_pose_marker = create_default_marker(
            "map", current_time, prefix + ":stop_pose", id, Marker::ARROW,
            create_marker_scale(0.4, 0.2, 0.2), create_marker_color(1.0, 0.0, 0.0, 0.999));
          stop_pose_marker.pose = stop_factor.stop_pose;
          marker_array.markers.emplace_back(stop_pose_marker);
        }
        // add view distance text
        {
          auto reason_text_marker = create_default_marker(
            "map", current_time, prefix + ":reason", id, Marker::TEXT_VIEW_FACING,
            create_marker_scale(0.0, 0.0, 0.2), create_marker_color(1.0, 1.0, 1.0, 0.999));
          reason_text_marker.pose = stop_factor.stop_pose;
          reason_text_marker.text = prefix;
          marker_array.markers.emplace_back(reason_text_marker);
        }
        id++;
      }
      if (!marker_array.markers.empty())
        append_marker_array(marker_array, &all_marker_array, current_time);
    }
    pub_stop_reasons_marker_->publish(all_marker_array);
  }
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_stop_reasons_marker_;
  rclcpp::Subscription<StopReasonArray>::SharedPtr sub_stop_reasons_;
};
}  // namespace planning_debug_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_debug_tools::StopReasonVisualizerNode)

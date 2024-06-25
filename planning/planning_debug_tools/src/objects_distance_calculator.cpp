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

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/uuid_helper.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace planning_debug_tools
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using std::placeholders::_1;

bool arePointsClose(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

bool areSameDirection(
  const double yaw, const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & end_point)
{
  return tier4_autoware_utils::normalizeRadian(
           yaw - std::atan2(end_point.y - start_point.y, end_point.x - start_point.x)) < M_PI_2;
}

tier4_autoware_utils::Point2d createPoint2d(const geometry_msgs::msg::Point & point)
{
  return tier4_autoware_utils::Point2d{point.x, point.y};
}
class ObjectsDistanceCalculatorNode : public rclcpp::Node
{
public:
  explicit ObjectsDistanceCalculatorNode(const rclcpp::NodeOptions & options)
  : Node("objects_distance_calculator", options)
  {
    sub_trajectory_ = create_subscription<Trajectory>(
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/"
      "trajectory",
      rclcpp::QoS{1}, std::bind(&ObjectsDistanceCalculatorNode::onTrajectory, this, _1));
    sub_objects_ = create_subscription<PredictedObjects>(
      "/perception/object_recognition/objects", rclcpp::QoS{1},
      std::bind(&ObjectsDistanceCalculatorNode::onObjects, this, _1));

    vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  }

private:
  void onObjects(const PredictedObjects::ConstSharedPtr msg)
  {
    if (!trajectory_ptr_) {
      return;
    }

    std::cerr << "======================" << std::endl;
    for (const auto & object : msg->objects) {
      const auto obj_uuid = tier4_autoware_utils::toHexString(object.object_id);
      const auto & obj_type = object.classification.front().label;
      if (
        obj_type != ObjectClassification::CAR && obj_type != ObjectClassification::TRUCK &&
        obj_type != ObjectClassification::BUS && obj_type != ObjectClassification::TRAILER) {
        continue;
      }

      const auto obj_poly = tier4_autoware_utils::toPolygon2d(object);
      // const auto obj_string = createLinearRing2d(obj_poly);

      const auto trajectory_bounds = getTrajectoryBounds(*trajectory_ptr_);
      const double dist_to_left = boost::geometry::distance(obj_poly, trajectory_bounds.first);
      const double dist_to_right = boost::geometry::distance(obj_poly, trajectory_bounds.second);

      std::cerr << obj_uuid.c_str() << " " << std::min(dist_to_left, dist_to_right) << std::endl;
    }
  }

  void onTrajectory(const Trajectory::ConstSharedPtr msg) { trajectory_ptr_ = msg; }

  std::pair<tier4_autoware_utils::LineString2d, tier4_autoware_utils::LineString2d>
  getTrajectoryBounds(const Trajectory & trajectory)
  {
    const double max_ego_lon_offset = vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m;
    const double min_ego_lon_offset = -vehicle_info_.rear_overhang_m;
    const double max_ego_lat_offset =
      vehicle_info_.wheel_tread_m / 2.0 + vehicle_info_.left_overhang_m;
    const double ego_lat_offset = max_ego_lat_offset;

    std::vector<geometry_msgs::msg::Point> ego_left_bound;
    std::vector<geometry_msgs::msg::Point> ego_right_bound;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & trajectory_point = trajectory.points.at(i).pose;
      if (i == 0) {
        // Add the first bound point
        ego_left_bound.push_back(tier4_autoware_utils::calcOffsetPose(
                                   trajectory_point, min_ego_lon_offset, ego_lat_offset, 0.0)
                                   .position);
        ego_right_bound.push_back(tier4_autoware_utils::calcOffsetPose(
                                    trajectory_point, min_ego_lon_offset, -ego_lat_offset, 0.0)
                                    .position);
      }

      if (i == trajectory.points.size() - 1) {
        // Add the last bound point
        const auto ego_left_bound_last_point =
          tier4_autoware_utils::calcOffsetPose(
            trajectory_point, max_ego_lon_offset, ego_lat_offset, 0.0)
            .position;
        if (!arePointsClose(ego_left_bound.back(), ego_left_bound_last_point, 1e-6)) {
          ego_left_bound.push_back(ego_left_bound_last_point);
        }
        const auto ego_right_bound_last_point =
          tier4_autoware_utils::calcOffsetPose(
            trajectory_point, max_ego_lon_offset, -ego_lat_offset, 0.0)
            .position;
        if (!arePointsClose(ego_right_bound.back(), ego_right_bound_last_point, 1e-6)) {
          ego_right_bound.push_back(ego_right_bound_last_point);
        }
      } else {
        // Calculate new bound point depending on the orientation
        const auto & next_trajectory_point = trajectory.points.at(i + 1).pose;
        const double diff_yaw = tier4_autoware_utils::normalizeRadian(
          tf2::getYaw(next_trajectory_point.orientation) -
          tf2::getYaw(trajectory_point.orientation));
        const auto [ego_left_bound_new_point, ego_right_bound_new_point] = [&]() {
          if (0 < diff_yaw) {
            return std::make_pair(
              tier4_autoware_utils::calcOffsetPose(trajectory_point, 0.0, ego_lat_offset, 0.0)
                .position,
              tier4_autoware_utils::calcOffsetPose(
                trajectory_point, max_ego_lon_offset, -ego_lat_offset, 0.0)
                .position);
          }
          return std::make_pair(
            tier4_autoware_utils::calcOffsetPose(
              trajectory_point, max_ego_lon_offset, ego_lat_offset, 0.0)
              .position,
            tier4_autoware_utils::calcOffsetPose(trajectory_point, 0.0, -ego_lat_offset, 0.0)
              .position);
        }();

        // Check if the bound will be longitudinally monotonic.
        if (areSameDirection(
              tf2::getYaw(trajectory_point.orientation), ego_left_bound.back(),
              ego_left_bound_new_point)) {
          ego_left_bound.push_back(ego_left_bound_new_point);
        }
        if (areSameDirection(
              tf2::getYaw(trajectory_point.orientation), ego_right_bound.back(),
              ego_right_bound_new_point)) {
          ego_right_bound.push_back(ego_right_bound_new_point);
        }
      }
    }

    tier4_autoware_utils::LineString2d trajectory_left_bound;
    for (const auto & point : ego_left_bound) {
      trajectory_left_bound.push_back(createPoint2d(point));
    }
    tier4_autoware_utils::LineString2d trajectory_right_bound;
    for (const auto & point : ego_right_bound) {
      trajectory_right_bound.push_back(createPoint2d(point));
    }

    return {trajectory_left_bound, trajectory_right_bound};
  }

  Trajectory::ConstSharedPtr trajectory_ptr_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  vehicle_info_util::VehicleInfo vehicle_info_;
};
}  // namespace planning_debug_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_debug_tools::ObjectsDistanceCalculatorNode)

// Copyright 2024 Autoware Foundation
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

#ifndef POINTCLOUD_DIVIDER_NODE_HPP_
#define POINTCLOUD_DIVIDER_NODE_HPP_

#include <string>

#define PCL_NO_RECOMPILE
#include <rclcpp/rclcpp.hpp>

namespace autoware::pointcloud_divider
{

class PointCloudDivider : public rclcpp::Node
{
public:
  explicit PointCloudDivider(const rclcpp::NodeOptions & node_options);
};

}  // namespace autoware::pointcloud_divider

#endif  // POINTCLOUD_DIVIDER_NODE_HPP_

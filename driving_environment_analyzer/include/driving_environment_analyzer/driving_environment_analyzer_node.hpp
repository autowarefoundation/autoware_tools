// Copyright 2024 TIER IV, Inc.
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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_NODE_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_NODE_HPP_

#include "driving_environment_analyzer/analyzer_core.hpp"
#include "driving_environment_analyzer/type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{

class DrivingEnvironmentAnalyzerNode : public rclcpp::Node
{
public:
  explicit DrivingEnvironmentAnalyzerNode(const rclcpp::NodeOptions & node_options);

private:
  void onMap(const LaneletMapBin::ConstSharedPtr map_msg);
  void analyze();

  std::shared_ptr<analyzer_core::AnalyzerCore> analyzer_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
  rosbag2_cpp::Reader reader_;
};
}  // namespace driving_environment_analyzer

#endif  // DRIVING_ENVIRONMENT_ANALYZER__DRIVING_ENVIRONMENT_ANALYZER_NODE_HPP_

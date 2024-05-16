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

#include "driving_environment_analyzer/driving_environment_analyzer_node.hpp"

#include "driving_environment_analyzer/analyzer_core.hpp"

#include <memory>
#include <string>
#include <vector>

namespace driving_environment_analyzer
{

DrivingEnvironmentAnalyzerNode::DrivingEnvironmentAnalyzerNode(
  const rclcpp::NodeOptions & node_options)
: Node("driving_environment_analyzer", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DrivingEnvironmentAnalyzerNode::analyze, this));

  sub_map_ = create_subscription<HADMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DrivingEnvironmentAnalyzerNode::onMap, this, _1));

  analyzer_ = std::make_shared<analyzer_core::AnalyzerCore>(*this);

  analyzer_->setBagFile(declare_parameter<std::string>("bag_path"));
}

void DrivingEnvironmentAnalyzerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  analyzer_->setMap(*msg);
}

void DrivingEnvironmentAnalyzerNode::analyze()
{
  if (!analyzer_->isDataReadyForStaticODDAnalysis()) {
    return;
  }

  analyzer_->analyzeStaticODDFactor();
  rclcpp::shutdown();
}
}  // namespace driving_environment_analyzer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(driving_environment_analyzer::DrivingEnvironmentAnalyzerNode)

// Copyright 2023 Autoware Foundation
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

#include "autoware_lanelet2_map_validator/validation.hpp"
#include "lanelet2_validation/Validation.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("autoware_lanelet2_map_validator");

  auto config = lanelet::autoware::validation::parseCommandLine(
    argc, const_cast<const char **>(argv));  // NOLINT

  auto command_line_config = config.command_line_config;
  if (command_line_config.help) {
    return 0;
  }
  if (command_line_config.print) {
    auto checks =
      lanelet::validation::availabeChecks(command_line_config.validationConfig.checksFilter);
    if (checks.empty()) {
      std::cout << "No checks found matching '" << command_line_config.validationConfig.checksFilter
                << "'\n";
    } else {
      std::cout << "Will use following checks:\n";
      for (auto & check : checks) {
        std::cout << check << '\n';
      }
    }
    return 0;
  }
  if (command_line_config.mapFile.empty()) {
    std::cout << "No map file specified" << std::endl;
    return 1;
  }

  auto issues = lanelet::autoware::validation::validateMap(config);
  lanelet::validation::printAllIssues(issues);
  return static_cast<int>(!issues.empty());
}

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

#include "lanelet2_map_validator/cli.hpp"
#include "lanelet2_map_validator/map_loader.hpp"
#include "lanelet2_map_validator/utils.hpp"
#include "lanelet2_map_validator/validation.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>

int main(int argc, char * argv[])
{
  lanelet::autoware::validation::MetaConfig meta_config =
    lanelet::autoware::validation::parseCommandLine(
      argc, const_cast<const char **>(argv));  // NOLINT

  // Print help (Already done in parseCommandLine)
  if (meta_config.command_line_config.help) {
    return 0;
  }

  // Print available validators
  if (meta_config.command_line_config.print) {
    auto checks = lanelet::validation::availabeChecks(  // cspell:disable-line
      meta_config.command_line_config.validationConfig.checksFilter);
    if (checks.empty()) {
      std::cout << "No checks found matching to '"
                << meta_config.command_line_config.validationConfig.checksFilter << "'"
                << std::endl;
    } else {
      std::cout << "The following checks are available:" << std::endl;
      for (auto & check : checks) {
        std::cout << check << std::endl;
      }
    }
    return 0;
  }

  // Validation start
  if (meta_config.command_line_config.mapFile.empty()) {
    throw std::invalid_argument("No map file specified!");
  }
  if (!std::filesystem::is_regular_file(meta_config.command_line_config.mapFile)) {
    throw std::invalid_argument("Map file doesn't exist or is not a file!");
  }
  const auto [lanelet_map_ptr, map_issue] = lanelet::autoware::validation::loadAndValidateMapLoad(
    meta_config.projector_type, meta_config.command_line_config.mapFile,
    meta_config.command_line_config.validationConfig);

  if (!lanelet_map_ptr) {
    lanelet::validation::printAllIssues(map_issue);
  } else if (!meta_config.requirements_file.empty()) {
    if (!std::filesystem::is_regular_file(meta_config.requirements_file)) {
      throw std::invalid_argument("Input file doesn't exist or is not a file!");
    }
    std::ifstream input_file(meta_config.requirements_file);
    json json_data;
    input_file >> json_data;
    lanelet::autoware::validation::process_requirements(json_data, meta_config, *lanelet_map_ptr);
  } else {
    auto issues = lanelet::autoware::validation::apply_validation(
      *lanelet_map_ptr, meta_config.command_line_config.validationConfig);
    lanelet::validation::printAllIssues(issues);
  }

  return 0;
}

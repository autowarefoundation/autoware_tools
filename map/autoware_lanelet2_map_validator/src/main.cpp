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

#include "lanelet2_validation/Validation.h"

#include <autoware_lanelet2_map_validator/cli.hpp>
#include <autoware_lanelet2_map_validator/utils.hpp>
#include <autoware_lanelet2_map_validator/validation.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>

// ANSI color codes for console output
#define BOLD_ONLY "\033[1m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"
#define BOLD_RED "\033[1;31m"
#define NORMAL_GREEN "\033[32m"
#define NORMAL_RED "\033[31m"
#define FONT_RESET "\033[0m"

int process_requirements(
  YAML::Node yaml_config, const lanelet::autoware::validation::MetaConfig & validator_config)
{
  uint64_t warning_count = 0;
  uint64_t error_count = 0;
  lanelet::autoware::validation::MetaConfig temp_validator_config = validator_config;

  for (YAML::Node requirement : yaml_config["requirements"]) {
    std::string id = requirement["id"].as<std::string>();
    bool requirement_passed = true;

    std::vector<lanelet::validation::DetectedIssues> issues;
    std::map<std::string, bool> temp_validation_results;

    for (YAML::Node validator : requirement["validators"]) {
      std::string validator_name = validator["name"].as<std::string>();
      temp_validator_config.command_line_config.validationConfig.checksFilter = validator_name;

      std::vector<lanelet::validation::DetectedIssues> temp_issues =
        lanelet::autoware::validation::validateMap(temp_validator_config);

      if (temp_issues.empty()) {
        // Validator passed
        temp_validation_results[validator_name] = true;
        validator["passed"] = true;
      } else {
        // Validator failed
        requirement_passed = false;
        warning_count += temp_issues[0].warnings().size();
        error_count += temp_issues[0].errors().size();
        temp_validation_results[validator_name] = false;
        validator["passed"] = false;
        YAML::Node issues_node = YAML::Node(YAML::NodeType::Sequence);
        for (lanelet::validation::Issue issue : temp_issues[0].issues) {
          YAML::Node issue_node;
          issue_node["severity"] = lanelet::validation::toString(issue.severity);
          issue_node["primitive"] = lanelet::validation::toString(issue.primitive);
          issue_node["id"] = issue.id;
          issue_node["message"] = issue.message;
          issues_node.push_back(issue_node);
        }
        validator["issues"] = issues_node;
      }

      lanelet::autoware::validation::appendIssues(issues, std::move(temp_issues));
    }

    std::cout << BOLD_ONLY << "[" << id << "] ";

    if (requirement_passed) {
      requirement["passed"] = true;
      std::cout << BOLD_GREEN << "Passed" << FONT_RESET << std::endl;
    } else {
      requirement["passed"] = false;
      std::cout << BOLD_RED << "Failed" << FONT_RESET << std::endl;
    }

    // In order to make "passed" field above then the "validators" field in the output file.
    YAML::Node temp_validators = requirement["validators"];
    requirement.remove("validators");
    requirement["validators"] = temp_validators;

    for (const auto & result : temp_validation_results) {
      if (result.second) {
        std::cout << "  - " << result.first << ": " << NORMAL_GREEN << "Passed" << FONT_RESET
                  << std::endl;
      } else {
        std::cout << "  - " << result.first << ": " << NORMAL_RED << "Failed" << FONT_RESET
                  << std::endl;
      }
    }
    lanelet::validation::printAllIssues(issues);
    std::cout << std::endl;
  }

  if (warning_count + error_count == 0) {
    std::cout << BOLD_GREEN << "No issues were found from " << FONT_RESET
              << validator_config.command_line_config.mapFile << std::endl;
  } else {
    if (warning_count > 0) {
      std::cout << BOLD_YELLOW << "Total of " << warning_count << " warnings were found from "
                << FONT_RESET << validator_config.command_line_config.mapFile << std::endl;
    }
    if (error_count > 0) {
      std::cout << BOLD_RED << "Total of " << error_count << " errors were found from "
                << FONT_RESET << validator_config.command_line_config.mapFile << std::endl;
    }
  }

  if (!validator_config.output_file_path.empty()) {
    std::string file_name = validator_config.output_file_path + "/lanelet2_validation_results.yaml";
    std::ofstream output_file(file_name);
    output_file << yaml_config;
    std::cout << "Results are output to " << file_name << std::endl;
  }

  return (warning_count + error_count == 0) ? 0 : 1;
}

int main(int argc, char * argv[])
{
  lanelet::autoware::validation::MetaConfig config =
    lanelet::autoware::validation::parseCommandLine(
      argc, const_cast<const char **>(argv));  // NOLINT

  // Print help (Already done in parseCommandLine)
  if (config.command_line_config.help) {
    return 0;
  }

  // Print available validators
  if (config.command_line_config.print) {
    auto checks =
      lanelet::validation::availabeChecks(config.command_line_config.validationConfig.checksFilter); // cspell:disable-line
    if (checks.empty()) {
      std::cout << "No checks found matching '"
                << config.command_line_config.validationConfig.checksFilter << "'\n";
    } else {
      std::cout << "The following checks are available:\n";
      for (auto & check : checks) {
        std::cout << check << '\n';
      }
    }
    return 0;
  }

  // Check whether the map_file is specified
  if (config.command_line_config.mapFile.empty()) {
    std::cout << "No map file specified" << std::endl;
    return 1;
  }

  // Validation start
  if (!config.requirements_file.empty()) {
    YAML::Node yaml_config = YAML::LoadFile(config.requirements_file);
    return process_requirements(yaml_config, config);
  } else {
    auto issues = lanelet::autoware::validation::validateMap(config);
    lanelet::validation::printAllIssues(issues);
    return static_cast<int>(!issues.empty());
  }
}

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
#include "lanelet2_map_validator/utils.hpp"
#include "lanelet2_map_validator/validation.hpp"

#include <nlohmann/json.hpp>

#include <lanelet2_validation/Validation.h>

#include <filesystem>
#include <fstream>
#include <iomanip>

// Use nlohmann::json for JSON handling
using json = nlohmann::json;

// ANSI color codes for console output
#define BOLD_ONLY "\033[1m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"
#define BOLD_RED "\033[1;31m"
#define NORMAL_GREEN "\033[32m"
#define NORMAL_RED "\033[31m"
#define FONT_RESET "\033[0m"

void process_requirements(
  json json_config, const lanelet::autoware::validation::MetaConfig & validator_config)
{
  uint64_t warning_count = 0;
  uint64_t error_count = 0;
  lanelet::autoware::validation::MetaConfig temp_validator_config = validator_config;

  for (auto & requirement : json_config["requirements"]) {
    std::string id = requirement["id"];
    bool requirement_passed = true;

    std::vector<lanelet::validation::DetectedIssues> issues;
    std::map<std::string, bool> temp_validation_results;

    for (auto & validator : requirement["validators"]) {
      std::string validator_name = validator["name"];
      temp_validator_config.command_line_config.validationConfig.checksFilter = validator_name;

      std::vector<lanelet::validation::DetectedIssues> temp_issues =
        lanelet::autoware::validation::validateMap(
          validator_config.projector_type, validator_config.command_line_config.mapFile,
          validator_config.command_line_config.validationConfig);

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

        json issues_json;
        for (const auto & issue : temp_issues[0].issues) {
          json issue_json;
          issue_json["severity"] = lanelet::validation::toString(issue.severity);
          issue_json["primitive"] = lanelet::validation::toString(issue.primitive);
          issue_json["id"] = issue.id;
          issue_json["message"] = issue.message;
          issues_json.push_back(issue_json);
        }
        validator["issues"] = issues_json;
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
    if (!std::filesystem::is_directory(validator_config.output_file_path)) {
      throw std::invalid_argument("Output path doesn't exist or is not a directory!");
    }
    std::filesystem::path file_directory = validator_config.output_file_path;
    std::filesystem::path file_path = file_directory / "lanelet2_validation_results.json";
    std::ofstream output_file(file_path);
    output_file << std::setw(4) << json_config;
    std::cout << "Results are output to " << file_path << std::endl;
  }
}

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
  } else if (!std::filesystem::is_regular_file(meta_config.command_line_config.mapFile)) {
    throw std::invalid_argument("Map file doesn't exist or is not a file!");
  }

  if (!meta_config.requirements_file.empty()) {
    if (!std::filesystem::is_regular_file(meta_config.requirements_file)) {
      throw std::invalid_argument("Input file doesn't exist or is not a file!");
    }
    std::ifstream input_file(meta_config.requirements_file);
    json json_config;
    input_file >> json_config;
    process_requirements(json_config, meta_config);
  } else {
    auto issues = lanelet::autoware::validation::validateMap(
      meta_config.projector_type, meta_config.command_line_config.mapFile,
      meta_config.command_line_config.validationConfig);
    lanelet::validation::printAllIssues(issues);
  }

  return 0;
}

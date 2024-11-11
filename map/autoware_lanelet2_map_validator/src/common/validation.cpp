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

#include "lanelet2_map_validator/validation.hpp"

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

// ANSI color codes for console output
#define BOLD_ONLY "\033[1m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"
#define BOLD_RED "\033[1;31m"
#define NORMAL_GREEN "\033[32m"
#define NORMAL_RED "\033[31m"
#define FONT_RESET "\033[0m"

// cSpell:words indegree Indegree

namespace lanelet
{
namespace autoware
{
namespace validation
{

std::unique_ptr<lanelet::Projector> getProjector(
  const std::string & projector_type, const lanelet::GPSPoint & origin)

{
  if (projector_type == projector_names::mgrs) {
    return std::make_unique<lanelet::projection::MGRSProjector>();
  }
  if (projector_type == projector_names::transverse_mercator) {
    return std::make_unique<lanelet::projection::TransverseMercatorProjector>(
      lanelet::Origin{origin});
  }
  if (projector_type == projector_names::utm) {
    return std::make_unique<lanelet::projection::UtmProjector>(lanelet::Origin{origin});
  }
  return nullptr;
}

std::vector<lanelet::validation::DetectedIssues> validateMap(
  const std::string & projector_type, const std::string & map_file,
  const lanelet::validation::ValidationConfig & val_config)
{
  std::vector<lanelet::validation::DetectedIssues> issues;
  lanelet::LaneletMapPtr map{nullptr};
  lanelet::validation::Strings errors;
  try {
    const auto projector = getProjector(projector_type, val_config.origin);
    if (!projector) {
      errors.push_back("No valid map projection type specified!");
    } else {
      map = lanelet::load(map_file, *projector, &errors);
    }
    if (map) {
      appendIssues(issues, lanelet::validation::validateMap(*map, val_config));
    } else {
      errors.push_back("Failed to load map!");
    }
    if (!errors.empty()) {
      issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                            return lanelet::validation::Issue(
                              lanelet::validation::Severity::Error, error);
                          }));
    }
  } catch (lanelet::LaneletError & err) {
    issues.emplace_back("general", utils::transform(errors, [](auto & error) {
                          return lanelet::validation::Issue(
                            lanelet::validation::Severity::Error, error);
                        }));
  }

  appendIssues(issues, lanelet::validation::validateMap(*map, val_config));
  return issues;
}

Validators parse_validators(const json & json_data)
{
  Validators validators;

  for (const auto & requirement : json_data["requirements"]) {
    for (const auto & validator : requirement["validators"]) {
      ValidatorInfo info;

      if (validator.contains("prerequisites")) {
        for (const auto & prereq : validator["prerequisites"]) {
          info.prereq_with_forgive_warnings[prereq["name"]] = false;
          if (prereq.contains("forgive_warnings")) {
            info.prereq_with_forgive_warnings[prereq["name"]] = prereq["forgive_warnings"];
          }
        }
      }
      validators[validator["name"]] = info;
    }
  }
  return validators;
}

std::tuple<std::queue<std::string>, Validators> create_validation_queue(
  const Validators & validators)
{
  std::unordered_map<std::string, std::vector<std::string>> graph;  // Graph of dependencies
  std::unordered_map<std::string, int> indegree;  // Indegree (number of prerequisites)
  std::queue<std::string> validation_queue;
  Validators remaining_validators;  // Validators left unprocessed

  // Build the graph and initialize indegree
  for (const auto & [name, info] : validators) {
    indegree[name] = 0;                 // Initialize indegree
    remaining_validators[name] = info;  // Throw all validators to remaining_validators first

    for (const auto & [prereq, forgive_warnings] : info.prereq_with_forgive_warnings) {
      graph[prereq].push_back(name);  // prereq -> validator (prereq points to validator)
      indegree[name]++;               // Increment the indegree of the validator
    }
  }

  // Use a queue to store validators with no prerequisites (indegree == 0)
  std::queue<std::string> q;
  for (const auto & [name, count] : indegree) {
    if (count == 0) {
      q.push(name);
      remaining_validators.erase(name);
    }
  }

  // Perform topological sort to derive execution order
  while (!q.empty()) {
    std::string current_validator_name = q.front();
    q.pop();

    // Add the current validator to the execution queue
    validation_queue.push(current_validator_name);

    // For each dependent validator, reduce indegree and add to the queue if indegree becomes 0
    for (const auto & neighbor : graph[current_validator_name]) {
      indegree[neighbor]--;
      if (indegree[neighbor] == 0) {
        q.push(neighbor);
        remaining_validators.erase(neighbor);
      }
    }
  }

  for (auto & [name, info] : remaining_validators) {
    info.max_severity = ValidatorInfo::Severity::ERROR;
  }

  return {validation_queue, remaining_validators};
}

// Function to find a validator block by name
json & find_validator_block(json & json_data, const std::string & validator_name)
{
  for (auto & requirement : json_data["requirements"]) {
    for (auto & validator : requirement["validators"]) {
      if (validator["name"] == validator_name) {
        return validator;  // Return a reference to the found validator
      }
    }
  }

  // Handle case where validator is not found
  std::string msg = "Validator block " + validator_name + " not found";
  throw std::runtime_error(msg);
}

std::vector<lanelet::validation::DetectedIssues> describe_unused_validators_to_json(
  json & json_data, const Validators & unused_validators)
{
  lanelet::validation::Issues issues;
  std::vector<lanelet::validation::DetectedIssues> detected_issues;

  for (const auto & [name, validator] : unused_validators) {
    json & validator_json = find_validator_block(json_data, name);
    validator_json["passed"] = false;
    json issues_json;
    json issue_json;
    issue_json["severity"] = lanelet::validation::toString(lanelet::validation::Severity::Error);
    issue_json["primitive"] =
      lanelet::validation::toString(lanelet::validation::Primitive::Primitive);
    issue_json["id"] = 0;
    issue_json["message"] = "Prerequisites don't exist OR they are making a loop.";
    issues_json.push_back(issue_json);
    validator_json["issues"] = issues_json;

    lanelet::validation::Issue issue;
    issue.severity = lanelet::validation::Severity::Error;
    issue.primitive = lanelet::validation::Primitive::Primitive;
    issue.id = lanelet::InvalId;
    issue.message = "Prerequisites don't exist OR they are making a loop.";
    issues.push_back(issue);
  }

  if (issues.size() > 0) {
    detected_issues.push_back({"invalid_prerequisites", issues});
  }
  return detected_issues;
}

std::vector<lanelet::validation::DetectedIssues> check_prerequisite_completion(
  const Validators & validators, const std::string target_validator_name)
{
  lanelet::validation::Issues issues;
  std::vector<lanelet::validation::DetectedIssues> detected_issues;

  ValidatorInfo current_validator_info = validators.at(target_validator_name);

  bool prerequisite_complete = true;
  for (const auto & [prereq, forgive_warnings] :
       current_validator_info.prereq_with_forgive_warnings) {
    if (
      validators.at(prereq).max_severity == ValidatorInfo::Severity::ERROR ||
      (validators.at(prereq).max_severity == ValidatorInfo::Severity::WARNING &&
       !forgive_warnings)) {
      prerequisite_complete = false;
      break;
    }
  }

  if (!prerequisite_complete) {
    lanelet::validation::Issue issue;
    issue.severity = lanelet::validation::Severity::Error;
    issue.primitive = lanelet::validation::Primitive::Primitive;
    issue.id = lanelet::InvalId;
    issue.message = "Prerequisites didn't pass";
    issues.push_back(issue);
  }

  if (issues.size() > 0) {
    detected_issues.push_back({target_validator_name, issues});
  }

  return detected_issues;
}

void summarize_validator_results(json & json_data)
{
  uint64_t warning_count = 0;
  uint64_t error_count = 0;

  for (auto & requirement : json_data["requirements"]) {
    std::string id = requirement["id"];
    bool is_requirement_passed = true;
    std::map<std::string, bool> validator_results;

    for (const auto & validator : requirement["validators"]) {
      bool validator_passed = validator["passed"];
      validator_results[validator["name"]] = validator_passed;
      is_requirement_passed &= validator_passed;

      if (!validator.contains("issues")) {
        continue;
      }
      for (const auto & issue : validator["issues"]) {
        if (
          issue["severity"] ==
          lanelet::validation::toString(lanelet::validation::Severity::Warning)) {
          warning_count++;
        } else if (
          issue["severity"] ==
          lanelet::validation::toString(lanelet::validation::Severity::Error)) {
          error_count++;
        }
      }
    }

    std::cout << BOLD_ONLY << "[" << id << "] ";

    if (is_requirement_passed) {
      requirement["passed"] = true;
      std::cout << BOLD_GREEN << "Passed" << FONT_RESET << std::endl;
    } else {
      requirement["passed"] = false;
      std::cout << BOLD_RED << "Failed" << FONT_RESET << std::endl;
    }

    for (const auto & [name, result] : validator_results) {
      if (result) {
        std::cout << "  - " << name << ": " << NORMAL_GREEN << "Passed" << FONT_RESET << std::endl;
      } else {
        std::cout << "  - " << name << ": " << NORMAL_RED << "Failed" << FONT_RESET << std::endl;
      }
    }
  }

  if (warning_count + error_count == 0) {
    std::cout << BOLD_GREEN << "No errors nor warnings were found" << FONT_RESET << std::endl;
  } else {
    if (warning_count > 0) {
      std::cout << BOLD_YELLOW << "Total of " << warning_count << " warnings were found"
                << FONT_RESET << std::endl;
    }
    if (error_count > 0) {
      std::cout << BOLD_RED << "Total of " << error_count << " errors were found" << FONT_RESET
                << std::endl;
    }
  }
}

lanelet::validation::ValidationConfig replace_validator(
  const lanelet::validation::ValidationConfig & input, const std::string & validator_name)
{
  auto temp = input;
  temp.checksFilter = validator_name;
  return temp;
}

void process_requirements(json json_data, const MetaConfig & validator_config)
{
  std::vector<lanelet::validation::DetectedIssues> issues;

  // List up validators in order
  Validators validators = parse_validators(json_data);
  auto [validation_queue, remaining_validators] = create_validation_queue(validators);

  // Note validators that cannot be run from the start
  if (auto unused_validator_issues =
        describe_unused_validators_to_json(json_data, remaining_validators);
      !unused_validator_issues.empty()) {
    appendIssues(issues, std::move(unused_validator_issues));
  }

  // Main validation process
  while (!validation_queue.empty()) {
    std::string validator_name = validation_queue.front();
    validation_queue.pop();

    std::vector<lanelet::validation::DetectedIssues> temp_issues;

    // Check prerequisites are OK
    appendIssues(temp_issues, check_prerequisite_completion(validators, validator_name));

    if (temp_issues.size() == 0) {
      // Validate map
      appendIssues(
        temp_issues,
        validateMap(
          validator_config.projector_type, validator_config.command_line_config.mapFile,
          replace_validator(
            validator_config.command_line_config.validationConfig, validator_name)));
    }

    // Add validation results to the json data
    json & validator_json = find_validator_block(json_data, validator_name);
    if (temp_issues.size() == 0) {
      validator_json["passed"] = true;
      continue;
    }

    if (temp_issues[0].warnings().size() + temp_issues[0].errors().size() == 0) {
      validator_json["passed"] = true;
    } else {
      validator_json["passed"] = false;
    }
    if (temp_issues[0].issues.size() > 0) {
      json issues_json;
      for (const auto & issue : temp_issues[0].issues) {
        json issue_json;
        issue_json["severity"] = lanelet::validation::toString(issue.severity);
        issue_json["primitive"] = lanelet::validation::toString(issue.primitive);
        issue_json["id"] = issue.id;
        issue_json["message"] = issue.message;
        issues_json.push_back(issue_json);

        if (
          static_cast<int>(issue.severity) <
          static_cast<int>(validators[validator_name].max_severity)) {
          validators[validator_name].max_severity =
            static_cast<ValidatorInfo::Severity>(static_cast<int>(issue.severity));
        }
      }
      validator_json["issues"] = issues_json;
    }
    appendIssues(issues, std::move(temp_issues));
  }

  // Show results
  summarize_validator_results(json_data);
  lanelet::validation::printAllIssues(issues);

  // Save results
  if (!validator_config.output_file_path.empty()) {
    if (!std::filesystem::is_directory(validator_config.output_file_path)) {
      throw std::invalid_argument("Output path doesn't exist or is not a directory!");
    }
    std::filesystem::path file_directory = validator_config.output_file_path;
    std::filesystem::path file_path = file_directory / "lanelet2_validation_results.json";
    std::ofstream output_file(file_path);
    output_file << std::setw(4) << json_data;
    std::cout << "Results are output to " << file_path << std::endl;
  }
}

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet

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

#ifndef LANELET2_MAP_VALIDATOR__VALIDATION_HPP_
#define LANELET2_MAP_VALIDATOR__VALIDATION_HPP_

#include "lanelet2_map_validator/cli.hpp"
#include "lanelet2_map_validator/utils.hpp"

#include <nlohmann/json.hpp>

#include <lanelet2_validation/Cli.h>
#include <lanelet2_validation/Validation.h>

#include <queue>
#include <regex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace lanelet::autoware::validation
{
struct ValidatorInfo
{
  enum class Severity { ERROR, WARNING, INFO, NONE };

  std::unordered_map<std::string, bool> prereq_with_forgive_warnings;
  Severity max_severity = Severity::NONE;
};

using Validators = std::unordered_map<std::string, ValidatorInfo>;

std::vector<lanelet::validation::DetectedIssues> apply_validation(
  const lanelet::LaneletMap & lanelet_map,
  const lanelet::validation::ValidationConfig & val_config);

Validators parse_validators(const json & json_data);

std::tuple<std::queue<std::string>, Validators> create_validation_queue(
  const Validators & validators);

// Function to find a validator block by name
json & find_validator_block(json & json_data, const std::string & validator_name);

std::vector<lanelet::validation::DetectedIssues> describe_unused_validators_to_json(
  json & json_data, const Validators & unused_validators);

std::vector<lanelet::validation::DetectedIssues> check_prerequisite_completion(
  const Validators & validators, const std::string target_validator_name);

void summarize_validator_results(json & json_data);

lanelet::validation::ValidationConfig replace_validator(
  const lanelet::validation::ValidationConfig & input, const std::string & validator_name);

void process_requirements(
  json json_data, const lanelet::autoware::validation::MetaConfig & validator_config,
  const lanelet::LaneletMap & lanelet_map);
}  // namespace lanelet::autoware::validation

#endif  // LANELET2_MAP_VALIDATOR__VALIDATION_HPP_

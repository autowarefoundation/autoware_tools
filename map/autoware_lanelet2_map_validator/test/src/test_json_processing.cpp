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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <gtest/gtest.h>
#include <lanelet2_validation/Validation.h>

#include <fstream>
#include <string>

using json = nlohmann::json;

namespace lanelet::autoware::validation
{

class JsonProcessingTest : public ::testing::Test
{
protected:
  json load_json_file(std::string file_name)
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");
    std::ifstream file(package_share_directory + "/data/json/" + file_name);
    EXPECT_TRUE(file.is_open()) << "Failed to open test JSON file.";

    json json_data;
    file >> json_data;
    return json_data;
  }
};

TEST_F(JsonProcessingTest, ParseValidatorsWithValidInput)
{
  json sample_input_data = load_json_file("test_input.json");
  Validators validators = parse_validators(sample_input_data);
  ASSERT_EQ(validators.size(), 3);
  ASSERT_TRUE(validators.find("parsing.valid_input.no_prerequisites1") != validators.end());
  ASSERT_TRUE(validators.find("parsing.valid_input.no_prerequisites2") != validators.end());
  ASSERT_TRUE(validators.find("parsing.valid_input.with_prerequisites1") != validators.end());
  EXPECT_EQ(
    validators["parsing.valid_input.no_prerequisites1"].prereq_with_forgive_warnings.size(), 0);
  EXPECT_EQ(
    validators["parsing.valid_input.no_prerequisites2"].prereq_with_forgive_warnings.size(), 0);
  EXPECT_EQ(
    validators["parsing.valid_input.with_prerequisites1"].prereq_with_forgive_warnings.size(), 2);
  EXPECT_TRUE(validators["parsing.valid_input.with_prerequisites1"]
                .prereq_with_forgive_warnings["parsing.valid_input.no_prerequisites1"]);
  EXPECT_FALSE(validators["parsing.valid_input.with_prerequisites1"]
                 .prereq_with_forgive_warnings["parsing.valid_input.no_prerequisites2"]);
}

TEST_F(JsonProcessingTest, CreateValidationQueueNoCycles)
{
  Validators validators = {
    {"validator1", {{}, ValidatorInfo::Severity::NONE}},
    {"validator2", {{{"validator1", true}}, ValidatorInfo::Severity::NONE}},
    {"validator3", {{{"validator2", true}}, ValidatorInfo::Severity::NONE}}};
  auto [queue, remaining] = create_validation_queue(validators);
  EXPECT_EQ(queue.size(), 3);
  EXPECT_TRUE(remaining.empty());
}

TEST_F(JsonProcessingTest, CreateValidationQueueWithCycles)
{
  Validators validators = {
    {"validator1", {{{"validator3", true}}, ValidatorInfo::Severity::NONE}},
    {"validator2", {{{"validator1", true}}, ValidatorInfo::Severity::NONE}},
    {"validator3", {{{"validator2", true}}, ValidatorInfo::Severity::NONE}},
    {"validator4", {{}, ValidatorInfo::Severity::NONE}}};

  auto [queue, remaining] = create_validation_queue(validators);
  EXPECT_EQ(queue.size(), 1);  // Expect empty queue because of a cycle
  EXPECT_EQ(remaining.size(), 3);
  EXPECT_EQ(remaining["validator1"].max_severity, ValidatorInfo::Severity::ERROR);
  EXPECT_EQ(remaining["validator2"].max_severity, ValidatorInfo::Severity::ERROR);
  EXPECT_EQ(remaining["validator3"].max_severity, ValidatorInfo::Severity::ERROR);
  EXPECT_EQ(remaining["validator4"].max_severity, ValidatorInfo::Severity::NONE);
}

TEST_F(JsonProcessingTest, CheckPrerequisiteCompletionSuccess)
{
  Validators validators = {
    {"validator1", {{}, ValidatorInfo::Severity::INFO}},
    {"validator2", {{{"validator1", true}}, ValidatorInfo::Severity::NONE}}};

  auto issues = check_prerequisite_completion(validators, "validator2");
  EXPECT_EQ(issues.size(), 0);
}

TEST_F(JsonProcessingTest, CheckPrerequisiteCompletionFailure)
{
  Validators validators = {
    {"validator1", {{}, ValidatorInfo::Severity::ERROR}},
    {"validator2", {{{"validator1", true}}, ValidatorInfo::Severity::NONE}}};

  auto issues = check_prerequisite_completion(validators, "validator2");
  EXPECT_EQ(issues.size(), 1);
}

TEST_F(JsonProcessingTest, DescribeUnusedValidatorsToJson)
{
  Validators error_validators = {
    {"validator1", {{{"validator3", true}}, ValidatorInfo::Severity::ERROR}},
    {"validator2", {{{"validator1", true}}, ValidatorInfo::Severity::ERROR}},
    {"validator3", {{{"validator2", true}}, ValidatorInfo::Severity::ERROR}}};

  json sample_input_data = load_json_file("test_describe_unused_validators_input.json");
  json answer_output_data = load_json_file("test_describe_unused_validators_output.json");

  // Check issues
  auto detected_issues = describe_unused_validators_to_json(sample_input_data, error_validators);
  EXPECT_EQ(detected_issues.size(), 1);
  EXPECT_EQ(detected_issues[0].issues.size(), 3);
  EXPECT_EQ(detected_issues[0].checkName, "general.invalid_prerequisites");
  EXPECT_EQ(detected_issues[0].issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(detected_issues[0].issues[1].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(detected_issues[0].issues[2].severity, lanelet::validation::Severity::Error);

  // Check json_data
  EXPECT_EQ(sample_input_data, answer_output_data);
}

TEST_F(JsonProcessingTest, SummarizeValidatorResults)
{
  json sample_input_data = load_json_file("test_summarize_validator_results_input.json");
  json answer_output_data = load_json_file("test_summarize_validator_results_output.json");

  testing::internal::CaptureStdout();
  summarize_validator_results(sample_input_data);
  EXPECT_EQ(sample_input_data, answer_output_data);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("[success-requirement] \033[1;32mPassed"), std::string::npos);
  EXPECT_NE(output.find("[failure-requirement] \033[1;31mFailed"), std::string::npos);
  EXPECT_NE(output.find("Total of 1 errors were found"), std::string::npos);
  EXPECT_EQ(output.find("warnings were found"), std::string::npos);
}
}  // namespace lanelet::autoware::validation

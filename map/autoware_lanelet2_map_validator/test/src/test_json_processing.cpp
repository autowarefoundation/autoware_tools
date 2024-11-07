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

namespace lanelet
{
namespace autoware
{
namespace validation
{

class JsonProcessingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");
    std::ifstream file(package_share_directory + "/data/test_input.json");
    ASSERT_TRUE(file.is_open()) << "Failed to open test JSON file.";

    file >> sample_input_data;
  }
  json sample_input_data;
  json sample_output_data;
};

TEST_F(JsonProcessingTest, ParseValidatorsWithValidInput)
{
  Validators validators = parse_validators(sample_input_data);
  ASSERT_EQ(validators.size(), 3);
  ASSERT_TRUE(validators.find("parsing.valid_input.no_prerequisites1") != validators.end());
  ASSERT_TRUE(validators.find("parsing.valid_input.no_prerequisites2") != validators.end());
  ASSERT_TRUE(validators.find("parsing.valid_input.with_prerequisites1") != validators.end());
  EXPECT_EQ(validators["parsing.valid_input.no_prerequisites1"].prerequisites.size(), 0);
  EXPECT_EQ(validators["parsing.valid_input.no_prerequisites2"].prerequisites.size(), 0);
  EXPECT_EQ(validators["parsing.valid_input.with_prerequisites1"].prerequisites.size(), 2);
  EXPECT_TRUE(validators["parsing.valid_input.with_prerequisites1"]
                .forgive_warnings["parsing.valid_input.no_prerequisites1"]);
  EXPECT_FALSE(validators["parsing.valid_input.with_prerequisites1"]
                 .forgive_warnings["parsing.valid_input.no_prerequisites2"]);
}
/*
TEST_F(JsonProcessingTest, CreateValidationQueueNoCycles)
{
  Validators validators = {
    {"validator1", ValidatorInfo{.prerequisites = {}}},
    {"validator2", ValidatorInfo{.prerequisites = {"validator1"}}},
  };

  auto [queue, remaining] = create_validation_queue(validators);
  EXPECT_EQ(queue.size(), 2);
  EXPECT_TRUE(remaining.empty());
}

TEST_F(JsonProcessingTest, CreateValidationQueueWithCycles)
{
  Validators validators = {
    {"validator1", ValidatorInfo{.prerequisites = {"validator2"}}},
    {"validator2", ValidatorInfo{.prerequisites = {"validator1"}}},
  };

  auto [queue, remaining] = create_validation_queue(validators);
  EXPECT_EQ(queue.size(), 0);  // Expect empty queue because of a cycle
  EXPECT_EQ(remaining.size(), 2);
  EXPECT_EQ(remaining["validator1"].max_severity, ValidatorInfo::Severity::ERROR);
}

TEST_F(JsonProcessingTest, CheckPrerequisiteCompletionSuccess)
{
  Validators validators = {
    {"validator1",
     ValidatorInfo{.prerequisites = {}, .max_severity = ValidatorInfo::Severity::INFO}},
    {"validator2", ValidatorInfo{.prerequisites = {"validator1"}}},
  };

  sample_input_data = R"({
        "requirements": [{
            "id": "1",
            "validators": [{"name": "validator2"}]
        }]
    })"_json;

  auto issues = check_prerequisite_completion(sample_input_data, validators, "validator2");
  EXPECT_TRUE(issues.empty());
}

TEST_F(JsonProcessingTest, CheckPrerequisiteCompletionFailure)
{
  Validators validators = {
    {"validator1",
     ValidatorInfo{.prerequisites = {}, .max_severity = ValidatorInfo::Severity::ERROR}},
    {"validator2", ValidatorInfo{.prerequisites = {"validator1"}}},
  };

  sample_input_data = R"({
        "requirements": [{
            "id": "1",
            "validators": [{"name": "validator2"}]
        }]
    })"_json;

  auto issues = check_prerequisite_completion(sample_input_data, validators, "validator2");
  EXPECT_FALSE(issues.empty());
}

TEST_F(JsonProcessingTest, DescriptUnusedValidatorsToJson)
{
  Validators unused_validators = {
    {"validator1",
     ValidatorInfo{.prerequisites = {}, .max_severity = ValidatorInfo::Severity::ERROR}},
  };

  sample_input_data = R"({
        "requirements": [{
            "id": "1",
            "validators": [{"name": "validator1"}]
        }]
    })"_json;

  auto detected_issues = descript_unused_validators_to_json(sample_input_data, unused_validators);
  EXPECT_EQ(detected_issues.size(), 1);
  EXPECT_EQ(detected_issues[0].issues.size(), 1);
  EXPECT_EQ(detected_issues[0].issues[0].severity, lanelet::validation::Severity::Error);

  // Check json_data too,
}

TEST_F(JsonProcessingTest, SummarizeValidatorResultsAllPassed)
{
  sample_input_data = R"({
        "requirements": [{
            "id": "1",
            "validators": [{"name": "validator1", "passed": true}]
        }]
    })"_json;

  testing::internal::CaptureStdout();
  summarize_validator_results(sample_input_data);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("Passed"), std::string::npos);
}

TEST_F(JsonProcessingTest, SummarizeValidatorResultsWithWarningsAndErrors)
{
  sample_input_data = R"({
        "requirements": [{
            "id": "1",
            "validators": [{
                "name": "validator1",
                "passed": false,
                "issues": [
                    {"severity": "Error", "message": "Test error"},
                    {"severity": "Warning", "message": "Test warning"}
                ]
            }]
        }]
    })"_json;

  testing::internal::CaptureStdout();
  summarize_validator_results(sample_input_data);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_NE(output.find("Failed"), std::string::npos);
  EXPECT_NE(output.find("errors were found"), std::string::npos);
  EXPECT_NE(output.find("warnings were found"), std::string::npos);
}
*/
}  // namespace validation
}  // namespace autoware
}  // namespace lanelet

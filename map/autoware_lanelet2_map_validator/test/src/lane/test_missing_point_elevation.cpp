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

#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/ValidatorFactory.h>

#include <string>

class TestMissingPointElevation : public MapValidationTester
{
private:
};

TEST_F(TestMissingPointElevation, CompleteLanelet)  // NOLINT for gtest
{
  load_target_map("lane/single_lanelet.osm");

  EXPECT_EQ(loading_issues_[0].issues.size(), 0);
}

TEST_F(TestMissingPointElevation, PartiallyMissingLanelet)  // NOLINT for gtest
{
  load_target_map("lane/single_lanelet_one_point_missing_ele.osm");

  bool found_identical_issue = false;
  const int target_primitive_id = 13;

  const lanelet::validation::Severity target_severity = lanelet::validation::Severity::Error;
  const lanelet::validation::Primitive target_primitive = lanelet::validation::Primitive::Primitive;
  const lanelet::Id target_issue_id = lanelet::InvalId;
  const std::string target_message = "Error reading primitive with id " +
                                     std::to_string(target_primitive_id) +
                                     " from file: Elevation tag is not defined for the given node.";

  for (const auto & issue : loading_issues_[0].issues) {
    if (
      issue.severity == target_severity && issue.primitive == target_primitive &&
      issue.id == target_issue_id && issue.message == target_message) {
      found_identical_issue = true;
    }
  }

  EXPECT_TRUE(found_identical_issue);
}

TEST_F(TestMissingPointElevation, PartiallyUndefinedLanelet)  // NOLINT for gtest
{
  load_target_map("lane/single_lanelet_one_point_having_non_value_ele.osm");

  bool found_identical_issue = false;
  const int target_primitive_id = 13;

  const lanelet::validation::Severity target_severity = lanelet::validation::Severity::Error;
  const lanelet::validation::Primitive target_primitive = lanelet::validation::Primitive::Primitive;
  const lanelet::Id target_issue_id = lanelet::InvalId;
  const std::string target_message = "Error reading primitive with id " +
                                     std::to_string(target_primitive_id) +
                                     " from file: Elevation tag exists but does not have a value.";

  for (const auto & issue : loading_issues_[0].issues) {
    if (
      issue.severity == target_severity && issue.primitive == target_primitive &&
      issue.id == target_issue_id && issue.message == target_message) {
      found_identical_issue = true;
    }
  }

  EXPECT_TRUE(found_identical_issue);
}

TEST_F(TestMissingPointElevation, EntirelyMissingLanelet)  // NOLINT for gtest
{
  load_target_map("lane/single_lanelet_without_ele.osm");

  int ele_issue_count = 0;

  lanelet::validation::Severity target_severity = lanelet::validation::Severity::Error;
  lanelet::validation::Primitive target_primitive = lanelet::validation::Primitive::Primitive;
  lanelet::Id target_issue_id = lanelet::InvalId;
  std::string target_partial_message = "Elevation tag is not defined for the given node.";

  for (const auto & issue : loading_issues_[0].issues) {
    if (
      issue.severity == target_severity && issue.primitive == target_primitive &&
      issue.id == target_issue_id &&
      issue.message.find(target_partial_message) != std::string::npos) {
      ele_issue_count++;
    }
  }

  EXPECT_EQ(ele_issue_count, 14);  // All fourteen points
}

TEST_F(TestMissingPointElevation, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");
}

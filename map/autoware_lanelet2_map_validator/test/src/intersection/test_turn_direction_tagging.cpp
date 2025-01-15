// Copyright 2025 Autoware Foundation
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

#include "lanelet2_map_validator/validators/intersection/turn_direction_tagging.hpp"
#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <string>

class TestIntersectionTurnDirectionTagging : public MapValidationTester
{
private:
};

TEST_F(TestIntersectionTurnDirectionTagging, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name =
    lanelet::autoware::validation::IntersectionTurnDirectionTaggingValidator::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestIntersectionTurnDirectionTagging, MissingTurnDirectionTag)  // NOLINT for gtest
{
  load_target_map("intersection/intersection_area_without_straight_tag.osm");

  lanelet::autoware::validation::IntersectionTurnDirectionTaggingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 58);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::Lanelet);
  EXPECT_EQ(
    issues[0].message,
    "[Intersection.TurnDirectionTagging-001] This lanelet is missing a turn_direction tag.");
}

TEST_F(TestIntersectionTurnDirectionTagging, WrongTurnDirectionTag)  // NOLINT for gtest
{
  load_target_map("intersection/intersection_area_with_invalid_turn_direction_tag.osm");

  lanelet::autoware::validation::IntersectionTurnDirectionTaggingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 53);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::Lanelet);
  EXPECT_EQ(
    issues[0].message,
    "[Intersection.TurnDirectionTagging-002] "
    "Invalid turn_direction tag \"leftt\" is found.");  // cspell:disable-line
}

TEST_F(TestIntersectionTurnDirectionTagging, CorrectIntersection)  // NOLINT for gtest
{
  load_target_map("intersection/basic_intersection_area.osm");

  lanelet::autoware::validation::IntersectionTurnDirectionTaggingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

TEST_F(TestIntersectionTurnDirectionTagging, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::autoware::validation::IntersectionTurnDirectionTaggingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

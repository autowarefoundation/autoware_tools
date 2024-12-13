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

#include "lanelet2_map_validator/validators/traffic_light/traffic_light_facing.hpp"
#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <set>

class TestTrafficLightFacing : public MapValidationTester
{
private:
};

TEST_F(TestTrafficLightFacing, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name =
    lanelet::autoware::validation::TrafficLightFacingValidator::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestTrafficLightFacing, WrongReferrerLanelet)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_with_wrong_referrer_lanelet.osm");

  lanelet::autoware::validation::TrafficLightFacingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 416);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Info);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::LineString);
  EXPECT_EQ(
    issues[0].message,
    "[TrafficLight.CorrectFacing-001] "
    "Lanelets referring this traffic_light have several divergent starting lines");
}

TEST_F(TestTrafficLightFacing, WrongTrafficLightFacing)  // NOLINT for gtest
{
  load_target_map("traffic_light/crosswalk_with_wrong_traffic_light_facing.osm");

  lanelet::autoware::validation::TrafficLightFacingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 48);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::LineString);
  EXPECT_EQ(
    issues[0].message,
    "[TrafficLight.CorrectFacing-002] The linestring direction seems to be wrong.");
}

TEST_F(TestTrafficLightFacing, UncertainTrafficLightFacing)  // NOLINT for gtest
{
  load_target_map("traffic_light/crosswalk_with_entirely_synched_traffic_lights.osm");

  lanelet::autoware::validation::TrafficLightFacingValidator checker;
  const auto & issues = checker(*map_);

  std::set<int> expected_ids = {48, 62, 74, 76};
  EXPECT_EQ(issues.size(), expected_ids.size());

  for (const auto & issue : issues) {
    EXPECT_NE(expected_ids.find(issue.id), expected_ids.end());
    EXPECT_EQ(issue.severity, lanelet::validation::Severity::Warning);
    EXPECT_EQ(issue.primitive, lanelet::validation::Primitive::LineString);
    EXPECT_EQ(
      issue.message,
      "[TrafficLight.CorrectFacing-003] The linestring direction has been judged as both correct "
      "and wrong.");
  }
}

TEST_F(TestTrafficLightFacing, CorrectFacing)  // NOLINT for gtest
{
  load_target_map("traffic_light/crosswalk_with_traffic_lights.osm");

  lanelet::autoware::validation::TrafficLightFacingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

TEST_F(TestTrafficLightFacing, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::autoware::validation::TrafficLightFacingValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);  // Four INFOs should appear
}

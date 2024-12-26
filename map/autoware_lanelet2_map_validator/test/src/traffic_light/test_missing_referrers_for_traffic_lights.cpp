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

#include "lanelet2_map_validator/validators/traffic_light/missing_referrers_for_traffic_lights.hpp"
#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <string>

class TestMissingReferrersForTrafficLights : public MapValidationTester
{
private:
};

TEST_F(TestMissingReferrersForTrafficLights, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name =
    lanelet::autoware::validation::MissingReferrersForTrafficLightsValidator::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestMissingReferrersForTrafficLights, MissingReferrers)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_without_referrer.osm");

  lanelet::autoware::validation::MissingReferrersForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 1025);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::RegulatoryElement);
  EXPECT_EQ(
    issues[0].message,
    "[TrafficLight.MissingReferrers-001] Regulatory element of traffic light must be referred by "
    "at least one lanelet.");
}

TEST_F(TestMissingReferrersForTrafficLights, ReferrerExists)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_with_regulatory_element.osm");

  lanelet::autoware::validation::MissingReferrersForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

TEST_F(TestMissingReferrersForTrafficLights, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::autoware::validation::MissingReferrersForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);  // Four INFOs should appear
}

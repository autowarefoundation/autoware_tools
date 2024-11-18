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

#include "lanelet2_map_validator/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp"
#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

class TestRegulatoryElementDetailsForTrafficLights : public MapValidationTester
{
private:
};

TEST_F(TestRegulatoryElementDetailsForTrafficLights, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name =
    lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, WrongRefersType)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_with_wrong_refers_type.osm");

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 416);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::LineString);
  EXPECT_EQ(
    issues[0].message,
    "Refers of traffic light regulatory element must have type of traffic_light.");
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, WrongRefLineType)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_with_wrong_ref_line_type.osm");

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 414);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::LineString);
  EXPECT_EQ(
    issues[0].message, "ref_line of traffic light regulatory element must have type of stop_line.");
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, MissingRefers)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_regulatory_element_without_refers.osm");

  // traffic_light-type regulatory elements that has no refers will not be loaded from the start
  // and should be mentioned in the loading_errors

  bool found_error_on_loading = false;
  int target_primitive_id = 1025;
  std::string target_message =
    "Error parsing primitive " + std::to_string(target_primitive_id) +
    ": Creating a regulatory element of type traffic_light failed: No traffic light defined!";

  for (const auto & error : loading_errors_) {
    if (error.find(target_message) != std::string::npos) {
      found_error_on_loading = true;
      break;
    }
  }

  EXPECT_TRUE(found_error_on_loading);
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, MissingRefLine)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_regulatory_element_without_ref_line.osm");

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 1025);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::RegulatoryElement);
  EXPECT_EQ(
    issues[0].message, "Regulatory element of traffic light must have a stop line(ref_line).");
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, CorrectDetails)  // NOLINT for gtest
{
  load_target_map("traffic_light/traffic_light_with_regulatory_element.osm");

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

TEST_F(TestRegulatoryElementDetailsForTrafficLights, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

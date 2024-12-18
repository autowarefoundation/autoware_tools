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
#include "validator_template.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

class TestValidatorTemplate : public MapValidationTester
{
private:
};

TEST_F(TestValidatorTemplate, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name = lanelet::autoware::validation::ValidatorTemplate::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestValidatorTemplate, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::autoware::validation::ValidatorTemplate checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

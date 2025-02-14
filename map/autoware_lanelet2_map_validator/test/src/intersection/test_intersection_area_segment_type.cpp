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

#include "lanelet2_map_validator/validators/intersection/intersection_area_segment_type.hpp"
#include "map_validation_tester.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <string>

class TestIntersectionAreaSegmentType : public MapValidationTester
{
private:
};

TEST_F(TestIntersectionAreaSegmentType, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validator_name =
    lanelet::autoware::validation::IntersectionAreaSegmentTypeValidator::name();

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validator_name);  // cspell:disable-line

  const uint32_t expected_validator_num = 1;
  EXPECT_EQ(expected_validator_num, validators.size());
  EXPECT_EQ(expected_validator_name, validators[0]);
}

TEST_F(TestIntersectionAreaSegmentType, CheckIrrelativePoint)  // NOLINT for gtest
{
  load_target_map("intersection/intersection_area_with_irrelative_point.osm");

  lanelet::autoware::validation::IntersectionAreaSegmentTypeValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 10803);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::Polygon);
  EXPECT_EQ(
    issues[0].message,
    "[Intersection.IntersectionAreaSegmentType-001] This intersection area is not made by points "
    "from road_border linestrings or lanelet edges. (Point ID: (10804))");
}

TEST_F(TestIntersectionAreaSegmentType, CheckWrongLinestringType)  // NOLINT for gtest
{
  load_target_map("intersection/intersection_area_with_wrong_linestring_type.osm");

  lanelet::autoware::validation::IntersectionAreaSegmentTypeValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 1);
  EXPECT_EQ(issues[0].id, 10803);
  EXPECT_EQ(issues[0].severity, lanelet::validation::Severity::Error);
  EXPECT_EQ(issues[0].primitive, lanelet::validation::Primitive::Polygon);
  EXPECT_EQ(
    issues[0].message,
    "[Intersection.IntersectionAreaSegmentType-001] This intersection area is not made by points "
    "from road_border linestrings or lanelet edges. (Point ID: (10756, 10757, 10758, 10759, 10760, "
    "10761, 10762, 10763, 10764, 10765, 10766, 10767, 10768, 10769, 10770, 10771, 10772, 10773, "
    "10774, 10775, 10776))");
}

TEST_F(TestIntersectionAreaSegmentType, ValidIntersectionArea)  // NOLINT for gtest
{
  load_target_map("intersection/basic_intersection_area.osm");

  lanelet::autoware::validation::IntersectionAreaSegmentTypeValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

TEST_F(TestIntersectionAreaSegmentType, SampleMap)  // NOLINT for gtest
{
  load_target_map("sample_map.osm");

  lanelet::autoware::validation::IntersectionAreaSegmentTypeValidator checker;
  const auto & issues = checker(*map_);

  EXPECT_EQ(issues.size(), 0);
}

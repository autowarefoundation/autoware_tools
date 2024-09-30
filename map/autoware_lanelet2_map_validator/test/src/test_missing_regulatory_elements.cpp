// Copyright 2023 Autoware Foundation
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

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_map_validator/validators/crosswalk/missing_regulatory_elements_for_crosswalks.hpp>
#include <autoware_lanelet2_map_validator/validators/stop_line/missing_regulatory_elements_for_stop_lines.hpp>
#include <autoware_lanelet2_map_validator/validators/traffic_light/missing_regulatory_elements_for_traffic_lights.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

using lanelet::AttributeMap;
using lanelet::AttributeName;
using lanelet::AttributeValueString;
using lanelet::Lanelet;
using lanelet::LaneletMapPtr;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::Polygon3d;
using lanelet::RegulatoryElementPtr;
using lanelet::TrafficLight;
using lanelet::autoware::Crosswalk;
using lanelet::utils::getId;
class TestSuite : public ::testing::Test
{
public:
  TestSuite() { initializeAttributes(); }

  ~TestSuite() override = default;

  void initializeAttributes()
  {
    // Stop Line
    sl_attr_[AttributeName::Type] = AttributeValueString::StopLine;

    // Traffic Light
    tl_attr_[AttributeName::Type] = AttributeValueString::TrafficLight;
    tl_attr_[AttributeName::Subtype] = "red_yellow_green";
    tl_attr_["height"] = "0.5";

    // Crosswalk polygon
    cw_poly_attr_[AttributeName::Type] =
      lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon;

    // Crosswalk
    cw_attr_[AttributeName::Type] = AttributeValueString::Lanelet;
    cw_attr_[AttributeName::Subtype] = AttributeValueString::Crosswalk;
    cw_attr_[AttributeName::SpeedLimit] = "10";
    cw_attr_[AttributeName::OneWay] = "no";
    cw_attr_[AttributeName::Location] = AttributeValueString::Urban;
    cw_attr_[AttributeName::ParticipantPedestrian] = "yes";

    // Regulatory element of traffic light
    AttributeMap tl_re_attr_ = AttributeMap();
    tl_re_attr_[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    tl_re_attr_[AttributeName::Subtype] = AttributeValueString::TrafficLight;

    // Regulatory element of crosswalk
    AttributeMap cw_re_attr_ = AttributeMap();
    cw_re_attr_[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    cw_re_attr_[AttributeName::Subtype] = AttributeValueString::Crosswalk;
  }

  void addTestMap(LaneletMapPtr in_map_ptr)
  {
    // Create test map

    // Points for stop line
    Point3d p0 = Point3d(getId(), 0.0, 0.0, 0.1);
    Point3d p1 = Point3d(getId(), 0.0, 1.0, 0.1);
    Point3d p2 = Point3d(getId(), 0.0, 2.0, 0.1);
    // Points for traffic light
    Point3d p3 = Point3d(getId(), 0.0, 0.0, 5.0);
    Point3d p4 = Point3d(getId(), 0.0, 1.0, 5.0);
    Point3d p5 = Point3d(getId(), 0.0, 2.0, 5.0);
    // Points for crosswalk
    Point3d p6 = Point3d(getId(), 1.0, 0.0, 0.1);
    Point3d p7 = Point3d(getId(), 1.0, 1.0, 0.1);
    Point3d p8 = Point3d(getId(), 1.0, 2.0, 0.1);
    Point3d p9 = Point3d(getId(), 2.0, 0.0, 0.1);
    Point3d p10 = Point3d(getId(), 2.0, 1.0, 0.1);
    Point3d p11 = Point3d(getId(), 2.0, 2.0, 0.1);

    // Stop line
    LineString3d sl1 = LineString3d(getId(), {p0, p1}, sl_attr_);
    LineString3d sl2 = LineString3d(getId(), {p1, p2}, sl_attr_);

    LineString3d tl1 = LineString3d(getId(), {p3, p4}, tl_attr_);
    LineString3d tl2 = LineString3d(getId(), {p4, p5}, tl_attr_);

    // LineStrings for crosswalk
    LineString3d cw_ls1 = LineString3d(getId(), {p6, p7});
    LineString3d cw_ls2 = LineString3d(getId(), {p7, p8});
    LineString3d cw_ls3 = LineString3d(getId(), {p9, p10});
    LineString3d cw_ls4 = LineString3d(getId(), {p10, p11});

    Polygon3d cw_poly1 = Polygon3d(getId(), {p7, p6, p9, p10, p7}, cw_poly_attr_);
    Polygon3d cw_poly2 = Polygon3d(getId(), {p8, p7, p10, p11, p8}, cw_poly_attr_);
    // Lanelets for crosswalk
    Lanelet cw1 = Lanelet(getId(), cw_ls1, cw_ls3, cw_attr_);
    Lanelet cw2 = Lanelet(getId(), cw_ls2, cw_ls4, cw_attr_);

    // Traffic light regulatory element
    RegulatoryElementPtr tl_reg_elem1, tl_reg_elem2;
    tl_reg_elem1 = TrafficLight::make(getId(), tl_re_attr_, {tl1}, {sl1});
    tl_reg_elem2 = TrafficLight::make(getId(), tl_re_attr_, {tl2}, {sl2});

    // Crosswalk regulatory element
    RegulatoryElementPtr cw_reg_elem1, cw_reg_elem2;
    cw_reg_elem1 = Crosswalk::make(getId(), cw_re_attr_, cw1, cw_poly1, {sl1});
    cw_reg_elem2 = Crosswalk::make(getId(), cw_re_attr_, cw2, cw_poly2, {sl2});

    cw1.addRegulatoryElement(cw_reg_elem1);
    cw2.addRegulatoryElement(cw_reg_elem2);

    // Add elements to map
    for (const auto & re : {tl_reg_elem1, tl_reg_elem2}) {
      in_map_ptr->add(re);
    }
    for (const auto & cw : {cw1, cw2}) {
      in_map_ptr->add(cw);
    }
  }
  AttributeMap sl_attr_, tl_attr_, cw_attr_, cw_poly_attr_, tl_re_attr_, cw_re_attr_;

private:
};

TEST_F(TestSuite, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validators_concat =
    "mapping.crosswalk.missing_regulatory_elements,"
    "mapping.stop_line.missing_regulatory_elements,"
    "mapping.traffic_light.missing_regulatory_elements";

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validators_concat); // cspell:disable-line
  uint8_t expected_num_validators = 3;
  std::cout << "size: " << validators.size() << std::endl;
  EXPECT_EQ(expected_num_validators, validators.size());

  std::set<std::string> expected_validators_set = {
    "mapping.crosswalk.missing_regulatory_elements",
    "mapping.stop_line.missing_regulatory_elements",
    "mapping.traffic_light.missing_regulatory_elements"};

  std::set<std::string> testing_validators_set = {
    lanelet::validation::MissingRegulatoryElementsForCrosswalksValidator::name(),
    lanelet::validation::MissingRegulatoryElementsForStopLinesValidator::name(),
    lanelet::validation::MissingRegulatoryElementsForTrafficLightsValidator::name()};

  for (const auto & name : testing_validators_set) {
    std::cout << name << std::endl;
    EXPECT_TRUE(expected_validators_set.find(name) != expected_validators_set.end())
      << "Unexpected validator found: " << name;
  }
}

TEST_F(TestSuite, MissingRegulatoryElementOfTrafficLight)  // NOLINT for gtest
{
  // Check missing regulatory element of traffic light

  const auto & tl_no_reg_elem = LineString3d(
    99999, {Point3d(getId(), 0.0, 3.0, 5.0), Point3d(getId(), 0.0, 4.0, 5.0)}, tl_attr_);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({tl_no_reg_elem});
  addTestMap(test_map_ptr);

  lanelet::validation::MissingRegulatoryElementsForTrafficLightsValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 1;
  static constexpr const char * expected_message =
    "No regulatory element refers to this traffic light.";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    EXPECT_EQ(99999, issue.id);
    EXPECT_EQ(expected_message, issue.message);
    EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
    EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
  }
}

TEST_F(TestSuite, MissingRegulatoryElementOfCrosswalk)  // NOLINT for gtest
{
  // Check missing regulatory element of crosswalk

  const auto & cw_no_reg_elem = Lanelet(
    99999,
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr_);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_reg_elem});
  addTestMap(test_map_ptr);

  lanelet::validation::MissingRegulatoryElementsForCrosswalksValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 1;
  static constexpr const char * expected_message =
    "No regulatory element refers to this crosswalk.";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    EXPECT_EQ(99999, issue.id);
    EXPECT_EQ(expected_message, issue.message);
    EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
    EXPECT_EQ(lanelet::validation::Primitive::Lanelet, issue.primitive);
  }
}

TEST_F(TestSuite, MissingRegulatoryElementOfStopLine)  // NOLINT for gtest
{
  // Check missing regulatory element of stop line

  const auto & sl_no_reg_elem = LineString3d(
    99999, {Point3d(getId(), 0.0, 3.0, 5.0), Point3d(getId(), 0.0, 4.0, 5.0)}, sl_attr_);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({sl_no_reg_elem});
  addTestMap(test_map_ptr);

  lanelet::validation::MissingRegulatoryElementsForStopLinesValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 1;
  static constexpr const char * expected_message =
    "No regulatory element refers to this stop line.";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    EXPECT_EQ(99999, issue.id);
    EXPECT_EQ(expected_message, issue.message);
    EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
    EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
  }
}

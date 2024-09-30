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
#include <autoware_lanelet2_map_validator/validators/crosswalk/regulatory_element_details_for_crosswalks.hpp>
#include <autoware_lanelet2_map_validator/validators/traffic_light/regulatory_element_details_for_traffic_lights.hpp>

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
    sl_attr[AttributeName::Type] = AttributeValueString::StopLine;

    // Traffic Light
    tl_attr[AttributeName::Type] = AttributeValueString::TrafficLight;
    tl_attr[AttributeName::Subtype] = "red_yellow_green";
    tl_attr["height"] = "0.5";

    // Crosswalk polygon
    cw_poly_attr[AttributeName::Type] =
      lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon;

    // Crosswalk
    cw_attr[AttributeName::Type] = AttributeValueString::Lanelet;
    cw_attr[AttributeName::Subtype] = AttributeValueString::Crosswalk;
    cw_attr[AttributeName::SpeedLimit] = "10";
    cw_attr[AttributeName::OneWay] = "no";
    cw_attr[AttributeName::Location] = AttributeValueString::Urban;
    cw_attr[AttributeName::ParticipantPedestrian] = "yes";

    // Regulatory element of traffic light
    AttributeMap tl_re_attr = AttributeMap();
    tl_re_attr[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    tl_re_attr[AttributeName::Subtype] = AttributeValueString::TrafficLight;

    // Regulatory element of crosswalk
    AttributeMap cw_re_attr = AttributeMap();
    cw_re_attr[AttributeName::Type] = AttributeValueString::RegulatoryElement;
    cw_re_attr[AttributeName::Subtype] = AttributeValueString::Crosswalk;
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
    LineString3d sl1 = LineString3d(getId(), {p0, p1}, sl_attr);
    LineString3d sl2 = LineString3d(getId(), {p1, p2}, sl_attr);

    LineString3d tl1 = LineString3d(getId(), {p3, p4}, tl_attr);
    LineString3d tl2 = LineString3d(getId(), {p4, p5}, tl_attr);

    // LineStrings for crosswalk
    LineString3d cw_ls1 = LineString3d(getId(), {p6, p7});
    LineString3d cw_ls2 = LineString3d(getId(), {p7, p8});
    LineString3d cw_ls3 = LineString3d(getId(), {p9, p10});
    LineString3d cw_ls4 = LineString3d(getId(), {p10, p11});

    Polygon3d cw_poly1 = Polygon3d(getId(), {p7, p6, p9, p10, p7}, cw_poly_attr);
    Polygon3d cw_poly2 = Polygon3d(getId(), {p8, p7, p10, p11, p8}, cw_poly_attr);
    // Lanelets for crosswalk
    Lanelet cw1 = Lanelet(getId(), cw_ls1, cw_ls3, cw_attr);
    Lanelet cw2 = Lanelet(getId(), cw_ls2, cw_ls4, cw_attr);

    // Traffic light regulatory element
    RegulatoryElementPtr tl_reg_elem1, tl_reg_elem2;
    tl_reg_elem1 = TrafficLight::make(getId(), tl_re_attr, {tl1}, {sl1});
    tl_reg_elem2 = TrafficLight::make(getId(), tl_re_attr, {tl2}, {sl2});

    // Crosswalk regulatory element
    RegulatoryElementPtr cw_reg_elem1, cw_reg_elem2;
    cw_reg_elem1 = Crosswalk::make(getId(), cw_re_attr, cw1, cw_poly1, {sl1});
    cw_reg_elem2 = Crosswalk::make(getId(), cw_re_attr, cw2, cw_poly2, {sl2});

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
  AttributeMap sl_attr, tl_attr, cw_attr, cw_poly_attr, tl_re_attr, cw_re_attr;

private:
};

TEST_F(TestSuite, ValidatorAvailability)  // NOLINT for gtest
{
  std::string expected_validators_concat =
    "mapping.crosswalk.regulatory_element_details,"
    "mapping.traffic_light.regulatory_element_details";

  lanelet::validation::Strings validators =
    lanelet::validation::availabeChecks(expected_validators_concat);  // cspell:disable-line
  uint8_t expected_num_validators = 2;
  std::cout << "size: " << validators.size() << std::endl;
  EXPECT_EQ(expected_num_validators, validators.size());

  std::set<std::string> expected_validators_set = {
    "mapping.crosswalk.regulatory_element_details",
    "mapping.traffic_light.regulatory_element_details"};

  std::set<std::string> testing_validators_set = {
    lanelet::validation::RegulatoryElementsDetailsForCrosswalksValidator::name(),
    lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator::name()};

  for (const auto & name : testing_validators_set) {
    std::cout << name << std::endl;
    EXPECT_TRUE(expected_validators_set.find(name) != expected_validators_set.end())
      << "Unexpected validator found: " << name;
  }
}

TEST_F(TestSuite, RegulatoryElementOfTrafficLightWithoutTrafficLight)  // NOLINT for gtest
{
  // Check regulatory element of traffic light without traffic light

  RegulatoryElementPtr tl_reg_elem_no_tl;
  // Line string without traffic light attribute
  const auto & ls = LineString3d(99998, {});
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({ls});
  // Traffic light regulatory element without traffic light. It refers to the line string without
  // traffic light attribute.
  tl_reg_elem_no_tl = TrafficLight::make(
    99999, tl_re_attr, {ls},
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  test_map_ptr->add(tl_reg_elem_no_tl);
  addTestMap(test_map_ptr);

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 2;
  static constexpr const char * expected_message1 =
    "Refers of traffic light regulatory element must have type of traffic_light.";
  static constexpr const char * expected_message2 =
    "Regulatory element of traffic light must have a traffic light(refers).";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    if (issue.id == 99998) {
      EXPECT_EQ(expected_message1, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    } else if (issue.id == 99999) {
      EXPECT_EQ(expected_message2, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::RegulatoryElement, issue.primitive);
    } else {
      FAIL() << "Unexpected issue id: " << issue.id;
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfTrafficLightWithoutStopLine)  // NOLINT for gtest
{
  // Check regulatory element of traffic light without stop line

  RegulatoryElementPtr tl_reg_elem_no_sl;
  // Line string without stop line attribute
  const auto & ls = LineString3d(99998, {});
  const auto & tl = LineString3d(
    getId(), {Point3d(getId(), 0.0, 3.0, 5.0), Point3d(getId(), 0.0, 4.0, 5.0)}, tl_attr);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({tl});
  // Traffic light regulatory element without stop line. It refers to the line string without stop
  // line attribute.
  tl_reg_elem_no_sl = TrafficLight::make(99999, tl_re_attr, {tl}, {ls});
  test_map_ptr->add(tl_reg_elem_no_sl);
  addTestMap(test_map_ptr);

  lanelet::validation::RegulatoryElementsDetailsForTrafficLightsValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 2;
  static constexpr const char * expected_message1 =
    "ref_line of traffic light regulatory element must have type of stop_line.";
  static constexpr const char * expected_message2 =
    "Regulatory element of traffic light must have a stop line(ref_line).";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    if (issue.id == 99998) {
      EXPECT_EQ(expected_message1, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::LineString, issue.primitive);
    } else if (issue.id == 99999) {
      EXPECT_EQ(expected_message2, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::RegulatoryElement, issue.primitive);
    } else {
      FAIL() << "Unexpected issue id: " << issue.id;
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutPolygon)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without polygon

  RegulatoryElementPtr cw_reg_elem_no_poly;
  Lanelet cw_no_poly = Lanelet(
    getId(),
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr);

  // Crosswalk regulatory element without cross walk polygon. It refers to the polygon without cross
  // walk polygon attribute.
  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, cw_no_poly, Polygon3d(99998),
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  cw_no_poly.addRegulatoryElement(reg_elem);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_poly});
  addTestMap(test_map_ptr);

  lanelet::validation::RegulatoryElementsDetailsForCrosswalksValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 2;
  static constexpr const char * expected_message1 =
    "Crosswalk polygon of crosswalk regulatory element must have type of Crosswalk_polygon.";
  static constexpr const char * expected_message2 =
    "Regulatory element of cross walk is nice to have crosswalk_polygon.";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    if (issue.id == 99998) {
      EXPECT_EQ(expected_message1, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::Polygon, issue.primitive);
    } else if (issue.id == 99999) {
      EXPECT_EQ(expected_message2, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Warning, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::RegulatoryElement, issue.primitive);
    } else {
      FAIL() << "Unexpected issue id: " << issue.id;
    }
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutStopline)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without stop line

  Lanelet cw_no_sl = Lanelet(
    getId(),
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}),
    cw_attr);

  // Crosswalk regulatory element without stop line.
  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, cw_no_sl,
    Polygon3d(getId(), {Point3d(getId(), 3.0, 3.0, 0.1)}, cw_poly_attr), {});
  cw_no_sl.addRegulatoryElement(reg_elem);
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({cw_no_sl});
  addTestMap(test_map_ptr);

  lanelet::validation::RegulatoryElementsDetailsForCrosswalksValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 1;
  static constexpr const char * expected_message =
    "Regulatory element of cross walk does not have stop line(ref_line).";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    EXPECT_EQ(expected_message, issue.message);
    EXPECT_EQ(99999, issue.id);
    EXPECT_EQ(lanelet::validation::Severity::Info, issue.severity);
    EXPECT_EQ(lanelet::validation::Primitive::RegulatoryElement, issue.primitive);
  }
}

TEST_F(TestSuite, RegulatoryElementOfCrosswalkWithoutCrosswalk)  // NOLINT for gtest
{
  // Check regulatory element of crosswalk without crosswalk

  const auto poly = Polygon3d(getId(), {Point3d(getId(), 3.0, 3.0, 0.1)}, cw_poly_attr);
  // Lanelet without crosswalk attribute
  const auto ll = Lanelet(
    99998,
    LineString3d(getId(), {Point3d(getId(), 3.0, 0.0, 0.1), Point3d(getId(), 3.0, 1.0, 0.1)}),
    LineString3d(getId(), {Point3d(getId(), 3.0, 1.0, 0.1), Point3d(getId(), 3.0, 2.0, 0.1)}));
  // Crosswalk regulatory element without crosswalk. It refers to the lanelet without crosswalk
  RegulatoryElementPtr reg_elem = Crosswalk::make(
    99999, cw_re_attr, ll, poly,
    {LineString3d(
      getId(), {Point3d(getId(), 3.0, 3.0, 0.1), Point3d(getId(), 3.0, 4.0, 0.1)}, sl_attr)});
  LaneletMapPtr test_map_ptr = lanelet::utils::createMap({poly});
  addTestMap(test_map_ptr);
  test_map_ptr->add(reg_elem);

  lanelet::validation::RegulatoryElementsDetailsForCrosswalksValidator checker_;
  const auto & issues = checker_(*test_map_ptr);

  uint8_t expected_num_issues = 2;
  static constexpr const char * expected_message1 =
    "Refers of crosswalk regulatory element must have type of crosswalk.";
  static constexpr const char * expected_message2 =
    "Regulatory element of cross walk must have lanelet of crosswalk(refers).";
  EXPECT_EQ(expected_num_issues, issues.size());
  for (const auto & issue : issues) {
    if (issue.id == 99998) {
      EXPECT_EQ(expected_message1, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::Lanelet, issue.primitive);
    } else if (issue.id == 99999) {
      EXPECT_EQ(expected_message2, issue.message);
      EXPECT_EQ(lanelet::validation::Severity::Error, issue.severity);
      EXPECT_EQ(lanelet::validation::Primitive::RegulatoryElement, issue.primitive);
    } else {
      FAIL() << "Unexpected issue id: " << issue.id;
    }
  }
}

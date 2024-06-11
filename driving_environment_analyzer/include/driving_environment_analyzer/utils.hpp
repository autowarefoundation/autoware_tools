// Copyright 2024 TIER IV, Inc.
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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__UTILS_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__UTILS_HPP_

#include "driving_environment_analyzer/type_alias.hpp"

#include <string>
#include <utility>
#include <vector>

namespace driving_environment_analyzer::utils
{
using autoware::route_handler::RouteHandler;

template <class T>
std::vector<double> calcElevationAngle(const T & points);

double calcElevationAngle(const lanelet::ConstLanelet & lane, const Pose & pose);

double getRouteLength(const lanelet::ConstLanelets & lanes);

double getMaxCurvature(const lanelet::ConstLanelets & lanes);

double getLaneWidth(const lanelet::ConstLanelet & lane);

std::pair<double, double> getLaneWidth(const lanelet::ConstLanelets & lanes);

std::pair<double, double> getElevation(const lanelet::ConstLanelets & lanes);

std::pair<double, double> getSpeedLimit(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler);

double getRouteLengthWithSameDirectionLane(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler);

double getRouteLengthWithOppositeDirectionLane(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler);

double getRouteLengthWithNoAdjacentLane(
  const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler);

bool existSameDirectionLane(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

bool existOppositeDirectionLane(
  const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

bool existRoadShoulderLane(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

int getLeftLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

int getRightLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

int getTotalLaneletNum(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

bool existTrafficLight(const lanelet::ConstLanelets & lanes);

bool existIntersection(const lanelet::ConstLanelets & lanes);

bool existCrosswalk(const lanelet::ConstLanelet & lane, const RouteHandler & route_handler);

bool existCrosswalk(const lanelet::ConstLanelets & lanes, const RouteHandler & route_handler);

bool isStraightLane(const lanelet::ConstLanelet & lane);

size_t getObjectNumber(const PredictedObjects & objects, const std::uint8_t target_class);

std::string getLaneShape(const lanelet::ConstLanelet & lane);

std::string getModuleStatus(const CooperateStatusArray & status_array, const uint8_t module_type);

std::string getEgoBehavior(
  const lanelet::ConstLanelet & lane, const RouteHandler & route_handler, const Pose & pose);

}  // namespace driving_environment_analyzer::utils

#endif  // DRIVING_ENVIRONMENT_ANALYZER__UTILS_HPP_

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

#include "data_structs.hpp"

#include <string>

namespace autoware::behavior_analyzer
{

std::string TOPIC::TF = "/tf";                                             // NOLINT
std::string TOPIC::ODOMETRY = "/localization/kinematic_state";             // NOLINT
std::string TOPIC::ACCELERATION = "/localization/acceleration";            // NOLINT
std::string TOPIC::OBJECTS = "/perception/object_recognition/objects";     // NOLINT
std::string TOPIC::TRAJECTORY = "/planning/scenario_planning/trajectory";  // NOLINT
std::string TOPIC::STEERING = "/vehicle/status/steering_status";           // NOLINT
std::string TOPIC::ROUTE = "/planning/mission_planning/route";             // NOLINT

}  // namespace autoware::behavior_analyzer

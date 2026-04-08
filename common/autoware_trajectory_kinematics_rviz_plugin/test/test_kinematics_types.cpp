// Copyright 2026 TIER IV, Inc.
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

#include "kinematics_types.hpp"

#include <gtest/gtest.h>

TEST(KinematicsTypes, AccessAxisValueTime)
{
  autoware::visualization::trajectory_kinematics_rviz_plugin::SeriesPoint p;
  p.time_from_start_sec = 1.25;
  using autoware::visualization::trajectory_kinematics_rviz_plugin::accessAxisValue;
  using autoware::visualization::trajectory_kinematics_rviz_plugin::AxisId;
  EXPECT_DOUBLE_EQ(accessAxisValue(p, AxisId::TIME_FROM_START), 1.25);
}

TEST(KinematicsTypes, AxisLabelKnown)
{
  using autoware::visualization::trajectory_kinematics_rviz_plugin::AxisId;
  using autoware::visualization::trajectory_kinematics_rviz_plugin::axisLabel;
  EXPECT_STREQ(axisLabel(AxisId::LONGITUDINAL_VELOCITY), "Longitudinal velocity");
}

TEST(KinematicsTypes, FindAxisDefinitionNullForInvalid)
{
  using autoware::visualization::trajectory_kinematics_rviz_plugin::AxisId;
  using autoware::visualization::trajectory_kinematics_rviz_plugin::findAxisDefinition;
  EXPECT_EQ(findAxisDefinition(static_cast<AxisId>(255)), nullptr);
}

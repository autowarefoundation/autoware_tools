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

#ifndef KINEMATICS_TYPES_HPP_
#define KINEMATICS_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief One sample along a trajectory polyline with derived kinematics for plotting.
struct SeriesPoint
{
  double time_from_start_sec{0.0};
  double arc_length_m{0.0};
  double longitudinal_velocity_mps{0.0};
  double acceleration_mps2{0.0};
  /// Signed planar curvature [1/m] (Menger estimate on XY).
  double curvature_1pm{0.0};
  /// Approximate lateral acceleration [m/s²] using a_lat ≈ v_x² κ (planar bicycle-style estimate).
  double lateral_acceleration_mps2{0.0};
};

/// @brief A plottable series: ROS topic, logical key, human-readable label, and sampled points.
struct TrajectorySeriesData
{
  std::string topic;
  /// Stable id within the topic (e.g. "trajectory", "candidate_3").
  std::string key;
  std::string label;
  std::vector<SeriesPoint> points;
  /// True once actual trajectory data has been received; false for placeholder entries
  /// that exist only because the topic was added but no message has arrived yet.
  bool has_received_data{false};
};

/// @brief Selects which scalar is read from SeriesPoint for an axis or tooltip.
enum class AxisId : std::uint8_t {
  TIME_FROM_START = 0,
  ARC_LENGTH,
  LONGITUDINAL_VELOCITY,
  ACCELERATION,
  CURVATURE,
  LATERAL_ACCELERATION,
};

/// @brief Human-readable labels and units for axis titles and tooltips.
struct AxisDefinition
{
  AxisId id;
  const char * label;
  const char * unit;
};

/// @brief X-axis choices (time or arc length).
inline const std::vector<AxisDefinition> & xAxisDefinitions()
{
  static const std::vector<AxisDefinition> defs = {
    {AxisId::TIME_FROM_START, "Time from start", "s"},
    {AxisId::ARC_LENGTH, "Arc length", "m"},
  };
  return defs;
}

/// @brief Y-axis choices (longitudinal/lateral motion and curvature).
inline const std::vector<AxisDefinition> & yAxisDefinitions()
{
  static const std::vector<AxisDefinition> defs = {
    {AxisId::LONGITUDINAL_VELOCITY, "Longitudinal velocity", "m/s"},
    {AxisId::ACCELERATION, "Acceleration", "m/s²"},
    {AxisId::CURVATURE, "Curvature (signed)", "1/m"},
    {AxisId::LATERAL_ACCELERATION, "Lateral acceleration (v²κ)", "m/s²"},
  };
  return defs;
}

/// @brief Maps a kinematics axis id to the corresponding field in `p`.
inline double accessAxisValue(const SeriesPoint & p, AxisId id)
{
  switch (id) {
    case AxisId::TIME_FROM_START:
      return p.time_from_start_sec;
    case AxisId::ARC_LENGTH:
      return p.arc_length_m;
    case AxisId::LONGITUDINAL_VELOCITY:
      return p.longitudinal_velocity_mps;
    case AxisId::ACCELERATION:
      return p.acceleration_mps2;
    case AxisId::CURVATURE:
      return p.curvature_1pm;
    case AxisId::LATERAL_ACCELERATION:
      return p.lateral_acceleration_mps2;
  }
#if defined(__GNUC__) || defined(__clang__)
  __builtin_unreachable();
#else
  return 0.0;
#endif
}

/// @brief Looks up axis metadata from xAxisDefinitions() then yAxisDefinitions().
inline const AxisDefinition * findAxisDefinition(AxisId id)
{
  for (const auto & d : xAxisDefinitions()) {
    if (d.id == id) {
      return &d;
    }
  }
  for (const auto & d : yAxisDefinitions()) {
    if (d.id == id) {
      return &d;
    }
  }
  return nullptr;
}

/// @brief Returns the display label for `id`, or `"?"` if unknown.
inline const char * axisLabel(AxisId id)
{
  const AxisDefinition * d = findAxisDefinition(id);
  return d ? d->label : "?";
}

/// @brief Returns the unit string for `id`, or empty if unknown.
inline const char * axisUnit(AxisId id)
{
  const AxisDefinition * d = findAxisDefinition(id);
  return d ? d->unit : "";
}

/// @brief Optional fixed axis ranges for the plot (honored only when the corresponding lock flag is
/// true).
struct PlotAxisRangeOptions
{
  bool lock_x{false};
  double x_min{0.0};
  double x_max{1.0};
  bool lock_y{false};
  double y_min{-1.0};
  double y_max{1.0};
};

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // KINEMATICS_TYPES_HPP_

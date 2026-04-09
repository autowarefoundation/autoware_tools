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

#ifndef TRAJECTORY_KINEMATICS_PLOT_WIDGET_HPP_
#define TRAJECTORY_KINEMATICS_PLOT_WIDGET_HPP_

#include "kinematics_types.hpp"
#include "trajectory_kinematics_chart_view.hpp"

#include <QColor>
#include <QWidget>

#include <string>
#include <vector>

namespace QtCharts
{
class QLineSeries;
class QValueAxis;
}  // namespace QtCharts

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief Hosts a TrajectoryKinematicsChartView and owns Qt Charts line series for trajectories
/// plus a crosshair.
///
/// Prefer updating existing QLineSeries data in place when the set of series
/// (topic/key/label/color) and axes are unchanged, to avoid rebuilding QChart and resetting legend
/// state on every message.
class TrajectoryKinematicsPlotWidget : public QWidget
{
public:
  /// @brief Creates the chart view with antialiasing and a minimum plot height.
  explicit TrajectoryKinematicsPlotWidget(QWidget * parent = nullptr);

  /// @brief Refreshes the plot: in-place point replacement when structure matches, otherwise full
  /// chart rebuild.
  /// @param series Trajectories to draw (each becomes one QLineSeries named from `label`).
  /// @param colors Line colors aligned with `series`.
  /// @param x_axis Horizontal quantity (time or arc length).
  /// @param y_axis Vertical quantity (velocity, acceleration, curvature, or lateral acceleration).
  /// @param range_opts Optional locked min/max for X and/or Y; otherwise ranges are padded from
  /// data bounds.
  void updatePlot(
    const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
    AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts);

private:
  /// @brief Constructs a new QChart, axes, data series, invisible crosshair series, and legend;
  /// disposes the previous chart.
  void rebuildChart(
    const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
    AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts);

  /// @brief If `last_legend_structure_key_` still matches, replaces points and updates axes without
  /// recreating the chart.
  /// @return True if the chart was updated in place; false if rebuildChart() is required.
  bool tryUpdateSeriesInPlace(
    const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
    AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts);

  /// @brief Sets axis ranges from `range_opts` when locked; otherwise uses data min/max with
  /// padding (or defaults if empty/invalid).
  void applyAxisRanges(
    QtCharts::QValueAxis * ax, QtCharts::QValueAxis * ay, double xmin, double xmax, double ymin,
    double ymax, const PlotAxisRangeOptions & range_opts) const;

  TrajectoryKinematicsChartView * chart_view_{nullptr};

  std::string last_legend_structure_key_;
  std::vector<QtCharts::QLineSeries *> data_line_series_;
  QtCharts::QLineSeries * crosshair_series_{nullptr};
  QtCharts::QValueAxis * x_axis_ptr_{nullptr};
  QtCharts::QValueAxis * y_axis_ptr_{nullptr};
};

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // TRAJECTORY_KINEMATICS_PLOT_WIDGET_HPP_

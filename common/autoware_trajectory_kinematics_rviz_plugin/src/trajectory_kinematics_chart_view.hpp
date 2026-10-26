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

#ifndef TRAJECTORY_KINEMATICS_CHART_VIEW_HPP_
#define TRAJECTORY_KINEMATICS_CHART_VIEW_HPP_

#include "kinematics_types.hpp"

#include <QColor>
#include <QElapsedTimer>
#include <QPointer>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief Qt Charts view with a vertical crosshair and a multi-series HTML tooltip at the pointer
/// X.
///
/// Maps mouse position to plot coordinates via QChart::mapToValue(), finds the nearest sample per
/// series along the current X axis (binary search when that axis is monotonic in point order), and
/// throttles tooltip updates to reduce overhead during motion.
class TrajectoryKinematicsChartView : public QtCharts::QChartView
{
  Q_OBJECT

public:
  /// @brief Enables mouse tracking and styles tooltips to match the panel palette.
  explicit TrajectoryKinematicsChartView(QWidget * parent = nullptr);

  /// @brief Registers the dedicated line series drawn as the vertical crosshair (updated on hover).
  void setCrosshairSeries(QtCharts::QLineSeries * series);

  /// @brief Supplies the trajectories and colors used for tooltip content and nearest-point lookup.
  /// @param series Same trajectories currently plotted (order matches `colors`).
  /// @param colors Swatch colors for HTML rows (may be shorter than `series`; a neutral gray is
  /// used if missing).
  /// @param x_axis Axis used for cursor X and for choosing the nearest point index in each series.
  /// @param y_axis Axis whose value is shown per series at that index.
  void setHoverSeriesData(
    const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
    AxisId x_axis, AxisId y_axis);

protected:
  /// @brief Updates crosshair endpoints, optional HTML tooltip, and rate limiting state.
  void mouseMoveEvent(QMouseEvent * event) override;

  /// @brief Hides the tooltip and clears the crosshair when the pointer leaves the view.
  void leaveEvent(QEvent * event) override;

private:
  std::vector<TrajectorySeriesData> hover_series_;
  std::vector<QColor> hover_colors_;
  AxisId x_axis_{AxisId::TIME_FROM_START};
  AxisId y_axis_{AxisId::LONGITUDINAL_VELOCITY};

  QPointer<QtCharts::QLineSeries> crosshair_series_;

  QElapsedTimer tooltip_rate_timer_;
  double last_tooltip_x_{0.0};
  bool last_tooltip_x_valid_{false};
};

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // TRAJECTORY_KINEMATICS_CHART_VIEW_HPP_

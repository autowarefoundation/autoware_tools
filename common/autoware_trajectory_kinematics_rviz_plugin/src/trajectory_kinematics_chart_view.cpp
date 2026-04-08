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

#include "trajectory_kinematics_chart_view.hpp"

#include "material_colors.hpp"

#include <QEvent>
#include <QMouseEvent>
#include <QToolTip>
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QAbstractSeries>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{
namespace
{

/// Plot X axis is always time or arc length; both are non-decreasing along sampled points.
bool xAxisIsMonotonicInSeries(AxisId x_axis)
{
  return x_axis == AxisId::TIME_FROM_START || x_axis == AxisId::ARC_LENGTH;
}

size_t nearestPointIndexLinear(const TrajectorySeriesData & s, AxisId x_axis, double x_target)
{
  if (s.points.empty()) {
    return 0;
  }
  size_t best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < s.points.size(); ++i) {
    const double xv = accessAxisValue(s.points[i], x_axis);
    if (!std::isfinite(xv)) {
      continue;
    }
    const double d = std::abs(xv - x_target);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

size_t nearestPointIndexMonotonic(const TrajectorySeriesData & s, AxisId x_axis, double x_target)
{
  if (s.points.empty()) {
    return 0;
  }
  const auto & pts = s.points;
  auto less_x = [x_axis](const SeriesPoint & p, double xv) {
    return accessAxisValue(p, x_axis) < xv;
  };
  auto it = std::lower_bound(pts.begin(), pts.end(), x_target, less_x);
  if (it == pts.begin()) {
    return 0;
  }
  if (it == pts.end()) {
    return pts.size() - 1;
  }
  auto prev = it - 1;
  const double d0 = std::abs(accessAxisValue(*prev, x_axis) - x_target);
  const double d1 = std::abs(accessAxisValue(*it, x_axis) - x_target);
  return d0 <= d1 ? static_cast<size_t>(std::distance(pts.begin(), prev))
                  : static_cast<size_t>(std::distance(pts.begin(), it));
}

size_t nearestPointIndexForSeries(const TrajectorySeriesData & s, AxisId x_axis, double x_target)
{
  if (xAxisIsMonotonicInSeries(x_axis)) {
    return nearestPointIndexMonotonic(s, x_axis, x_target);
  }
  return nearestPointIndexLinear(s, x_axis, x_target);
}

QtCharts::QLineSeries * firstDataLineSeries(
  QtCharts::QChart * ch, const QtCharts::QLineSeries * crosshair)
{
  if (ch == nullptr) {
    return nullptr;
  }
  for (QtCharts::QAbstractSeries * ser : ch->series()) {
    auto * ls = qobject_cast<QtCharts::QLineSeries *>(ser);
    if (ls == nullptr || ls == crosshair) {
      continue;
    }
    return ls;
  }
  return nullptr;
}

constexpr int kTooltipMinIntervalMs = 50;
constexpr double kTooltipSameXEps = 1e-7;

}  // namespace

TrajectoryKinematicsChartView::TrajectoryKinematicsChartView(QWidget * parent)
: QtCharts::QChartView(parent)
{
  setMouseTracking(true);
  setRubberBand(QtCharts::QChartView::NoRubberBand);

  const auto & pal = style::default_colors;
  setStyleSheet(QStringLiteral(
                  "QToolTip {"
                  "  background-color: %1;"
                  "  color: %2;"
                  "  border: 1px solid %3;"
                  "  border-radius: 6px;"
                  "  padding: 6px 8px;"
                  "  font-size: 11px;"
                  "}")
                  .arg(QString::fromStdString(pal.surface_container_high))
                  .arg(QString::fromStdString(pal.on_surface))
                  .arg(QString::fromStdString(pal.outline_variant)));
}

void TrajectoryKinematicsChartView::setCrosshairSeries(QtCharts::QLineSeries * series)
{
  crosshair_series_ = series;
}

void TrajectoryKinematicsChartView::setHoverSeriesData(
  const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
  AxisId x_axis, AxisId y_axis)
{
  hover_series_ = series;
  hover_colors_ = colors;
  x_axis_ = x_axis;
  y_axis_ = y_axis;
}

void TrajectoryKinematicsChartView::mouseMoveEvent(QMouseEvent * event)
{
  QtCharts::QChartView::mouseMoveEvent(event);

  auto * ch = chart();
  if (hover_series_.empty() || ch == nullptr) {
    QToolTip::hideText();
    if (crosshair_series_) {
      crosshair_series_->clear();
    }
    return;
  }

  auto * ref_series = firstDataLineSeries(ch, crosshair_series_);
  if (ref_series == nullptr) {
    QToolTip::hideText();
    if (crosshair_series_) {
      crosshair_series_->clear();
    }
    return;
  }

  // Widget → scene → chart item space: required before QChart::mapToValue for correct axis
  // mapping.
  const QPointF scene_pos = mapToScene(event->pos());
  const QPointF chart_pos = ch->mapFromScene(scene_pos);
  if (!ch->plotArea().contains(chart_pos)) {
    QToolTip::hideText();
    if (crosshair_series_) {
      crosshair_series_->clear();
    }
    return;
  }

  // mapToValue uses the attached axes; ref_series must be a data line (not the crosshair) so X
  // is read in plot units.
  const QPointF value = ch->mapToValue(chart_pos, ref_series);
  const double x_cursor = value.x();
  if (!std::isfinite(x_cursor)) {
    QToolTip::hideText();
    if (crosshair_series_) {
      crosshair_series_->clear();
    }
    return;
  }

  const QList<QtCharts::QAbstractAxis *> vaxes = ch->axes(Qt::Vertical);
  auto * ay = qobject_cast<QtCharts::QValueAxis *>(vaxes.isEmpty() ? nullptr : vaxes.first());
  if (crosshair_series_ && ay != nullptr) {
    const double y_lo = ay->min();
    const double y_hi = ay->max();
    crosshair_series_->clear();
    crosshair_series_->append(x_cursor, y_lo);
    crosshair_series_->append(x_cursor, y_hi);
  }

  // Throttle rich HTML tooltip updates: crosshair still moves, but QString rebuilds are skipped at
  // high pointer rate.
  const bool same_x =
    last_tooltip_x_valid_ && std::abs(x_cursor - last_tooltip_x_) <= kTooltipSameXEps;
  const bool within_rate =
    tooltip_rate_timer_.isValid() && tooltip_rate_timer_.elapsed() < kTooltipMinIntervalMs;
  if (same_x && within_rate) {
    return;
  }
  tooltip_rate_timer_.restart();
  last_tooltip_x_ = x_cursor;
  last_tooltip_x_valid_ = true;

  const auto & pal = style::default_colors;
  const QString bg = QString::fromStdString(pal.surface_container_high);
  const QString fg = QString::fromStdString(pal.on_surface);
  const QString muted = QString::fromStdString(pal.on_surface_variant);

  QString html;
  html += QStringLiteral(
            "<html><body style='background-color:%1; color:%2; font-size:11px; margin:0; "
            "padding:4px;'>")
            .arg(bg)
            .arg(fg);
  html += QStringLiteral(
            "<div style='margin-bottom:6px;'><b>%1</b> = %2 "
            "<span style='color:%4;'>%3</span></div>")
            .arg(QString::fromUtf8(axisLabel(x_axis_)))
            .arg(x_cursor, 0, 'f', 4)
            .arg(QString::fromUtf8(axisUnit(x_axis_)))
            .arg(muted);

  for (size_t i = 0; i < hover_series_.size(); ++i) {
    const auto & s = hover_series_[i];
    if (s.points.empty()) {
      continue;
    }
    const size_t idx = nearestPointIndexForSeries(s, x_axis_, x_cursor);
    const double yv = accessAxisValue(s.points[idx], y_axis_);
    const QColor c = (i < hover_colors_.size()) ? hover_colors_[i] : QColor(200, 200, 200);
    html += QStringLiteral(
              "<div style='white-space:nowrap; color:%4;'>"
              "<span style='color:%1; font-size:16px; line-height:1;'>■</span>"
              "&nbsp;<b style='color:%5;'>%2</b> <span style='color:%4;'>%3</span>"
              "</div>")
              .arg(c.name(QColor::HexRgb))
              .arg(yv, 0, 'f', 4)
              .arg(QString::fromUtf8(axisUnit(y_axis_)))
              .arg(muted)
              .arg(fg);
  }
  html += QStringLiteral("</body></html>");

  QToolTip::showText(event->globalPos(), html, this, QRect(), 8000);
}

void TrajectoryKinematicsChartView::leaveEvent(QEvent * event)
{
  QToolTip::hideText();
  last_tooltip_x_valid_ = false;
  if (crosshair_series_) {
    crosshair_series_->clear();
  }
  QtCharts::QChartView::leaveEvent(event);
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

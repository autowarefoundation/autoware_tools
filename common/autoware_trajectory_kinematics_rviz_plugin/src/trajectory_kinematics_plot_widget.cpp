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

#include "trajectory_kinematics_plot_widget.hpp"

#include "material_colors.hpp"
#include "ui_font.hpp"

#include <QBrush>
#include <QMargins>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QVBoxLayout>
#include <QVector>
#include <QtCharts/QChart>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{
namespace
{

void applyAxisTheme(QtCharts::QValueAxis * axis, const style::MaterialColors & pal)
{
  const QColor fg = style::hexToQColor(pal.on_surface_variant);
  const QColor title = style::hexToQColor(pal.on_surface);
  axis->setLabelsColor(fg);
  axis->setTitleBrush(QBrush(title));
  axis->setLinePenColor(style::hexToQColor(pal.outline_variant));
  axis->setGridLineColor(style::hexToQColor(pal.surface_container_high));
}

void applyChartTheme(QtCharts::QChart * chart, const style::MaterialColors & pal)
{
  chart->setBackgroundRoundness(6);
  chart->setMargins(QMargins(8, 8, 8, 8));
  chart->setBackgroundBrush(QBrush(style::hexToQColor(pal.surface_container)));
  chart->setTitleBrush(QBrush(style::hexToQColor(pal.on_surface)));
  chart->legend()->setLabelColor(style::hexToQColor(pal.on_surface_variant));
  chart->legend()->setBackgroundVisible(true);
  chart->legend()->setBrush(QBrush(style::hexToQColor(pal.surface_container_low)));
}

void applyChartFonts(QtCharts::QChart * chart, QtCharts::QValueAxis * ax, QtCharts::QValueAxis * ay)
{
  const QFont base = makePanelUiFont();

  QFont title_font = base;
  title_font.setPixelSize(14);
  title_font.setWeight(QFont::Bold);
  chart->setTitleFont(title_font);

  QFont legend_font = base;
  legend_font.setPixelSize(12);
  legend_font.setWeight(QFont::DemiBold);
  chart->legend()->setFont(legend_font);

  QFont axis_title = base;
  axis_title.setPixelSize(12);
  axis_title.setWeight(QFont::DemiBold);
  QFont axis_labels = base;
  axis_labels.setPixelSize(11);
  axis_labels.setWeight(QFont::Medium);

  ax->setTitleFont(axis_title);
  ax->setLabelsFont(axis_labels);
  ay->setTitleFont(axis_title);
  ay->setLabelsFont(axis_labels);
}

std::string buildLegendStructureKey(
  const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
  AxisId x_axis, AxisId y_axis)
{
  std::ostringstream oss;
  oss << static_cast<int>(x_axis) << '|' << static_cast<int>(y_axis) << '|' << series.size() << '|';
  // Field separator \x01: ROS topic/key follow naming rules; labels are built from ASCII topic,
  // generator metadata, and numeric scores — none use 0x01 in practice.
  for (size_t i = 0; i < series.size(); ++i) {
    oss << series[i].topic << '\x01' << series[i].key << '\x01' << series[i].label << '\x01';
    if (i < colors.size()) {
      oss << colors[i].rgba();
    }
    oss << '\x02';
  }
  return oss.str();
}

void computeDataBounds(
  const std::vector<TrajectorySeriesData> & series, AxisId x_axis, AxisId y_axis, double * xmin,
  double * xmax, double * ymin, double * ymax)
{
  *xmin = std::numeric_limits<double>::infinity();
  *xmax = -std::numeric_limits<double>::infinity();
  *ymin = std::numeric_limits<double>::infinity();
  *ymax = -std::numeric_limits<double>::infinity();
  for (const auto & s : series) {
    for (const auto & p : s.points) {
      const double xv = accessAxisValue(p, x_axis);
      const double yv = accessAxisValue(p, y_axis);
      if (std::isfinite(xv) && std::isfinite(yv)) {
        *xmin = std::min(*xmin, xv);
        *xmax = std::max(*xmax, xv);
        *ymin = std::min(*ymin, yv);
        *ymax = std::max(*ymax, yv);
      }
    }
  }
}

QVector<QPointF> pointsForSeries(const TrajectorySeriesData & s, AxisId x_axis, AxisId y_axis)
{
  QVector<QPointF> pts;
  for (const auto & p : s.points) {
    const double xv = accessAxisValue(p, x_axis);
    const double yv = accessAxisValue(p, y_axis);
    if (std::isfinite(xv) && std::isfinite(yv)) {
      pts.append(QPointF(xv, yv));
    }
  }
  return pts;
}

}  // namespace

TrajectoryKinematicsPlotWidget::TrajectoryKinematicsPlotWidget(QWidget * parent) : QWidget(parent)
{
  setObjectName("TkmPlotHost");
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  chart_view_ = new TrajectoryKinematicsChartView(this);
  chart_view_->setRenderHint(QPainter::Antialiasing);
  chart_view_->setMinimumHeight(250);
  chart_view_->setMouseTracking(true);
  layout->addWidget(chart_view_);
  setLayout(layout);

  const auto & pal = style::default_colors;
  setFont(makePanelUiFont());
  setStyleSheet(QStringLiteral("QWidget#TkmPlotHost { background-color: %1; border-radius: 8px;"
                               "  font-family: \"Segoe UI\", \"Ubuntu\", \"Noto Sans\", sans-serif;"
                               "  font-size: 13px; font-weight: 600; }")
                  .arg(QString::fromStdString(pal.surface_container)));
}

void TrajectoryKinematicsPlotWidget::applyAxisRanges(
  QtCharts::QValueAxis * ax, QtCharts::QValueAxis * ay, double xmin, double xmax, double ymin,
  double ymax, const PlotAxisRangeOptions & range_opts) const
{
  if (range_opts.lock_x && range_opts.x_max > range_opts.x_min) {
    ax->setRange(range_opts.x_min, range_opts.x_max);
  } else if (std::isfinite(xmin) && std::isfinite(xmax) && xmin < xmax) {
    const double pad = 0.02 * (xmax - xmin);
    ax->setRange(xmin - pad, xmax + pad);
  } else {
    ax->setRange(0.0, 1.0);
  }

  if (range_opts.lock_y && range_opts.y_max > range_opts.y_min) {
    ay->setRange(range_opts.y_min, range_opts.y_max);
  } else if (std::isfinite(ymin) && std::isfinite(ymax)) {
    if (ymin < ymax) {
      const double pad = 0.02 * (ymax - ymin);
      ay->setRange(ymin - pad, ymax + pad);
    } else {
      ay->setRange(ymin - 1.0, ymax + 1.0);
    }
  } else {
    ay->setRange(-1.0, 1.0);
  }
}

bool TrajectoryKinematicsPlotWidget::tryUpdateSeriesInPlace(
  const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
  AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts)
{
  // Structure key encodes axes, series count, topic/key/label, and color — any change forces a
  // full QChart rebuild.
  const std::string key = buildLegendStructureKey(series, colors, x_axis, y_axis);
  if (key != last_legend_structure_key_) {
    return false;
  }
  if (
    x_axis_ptr_ == nullptr || y_axis_ptr_ == nullptr || crosshair_series_ == nullptr ||
    data_line_series_.size() != series.size()) {
    return false;
  }

  double xmin = 0.0;
  double xmax = 0.0;
  double ymin = 0.0;
  double ymax = 0.0;
  computeDataBounds(series, x_axis, y_axis, &xmin, &xmax, &ymin, &ymax);

  for (size_t si = 0; si < series.size(); ++si) {
    auto * ls = data_line_series_[si];
    const QVector<QPointF> pts = pointsForSeries(series[si], x_axis, y_axis);
    if (pts.isEmpty()) {
      ls->clear();
    } else {
      ls->replace(pts);
    }
    const QColor col = si < colors.size() ? colors[si] : QColor(139, 208, 240);
    if (ls->color() != col) {
      ls->setColor(col);
    }
  }

  applyAxisRanges(x_axis_ptr_, y_axis_ptr_, xmin, xmax, ymin, ymax, range_opts);
  chart_view_->setHoverSeriesData(series, colors, x_axis, y_axis);
  return true;
}

void TrajectoryKinematicsPlotWidget::rebuildChart(
  const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
  AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts)
{
  const auto & pal = style::default_colors;

  data_line_series_.clear();
  crosshair_series_ = nullptr;
  x_axis_ptr_ = nullptr;
  y_axis_ptr_ = nullptr;

  QtCharts::QChart * old_chart = chart_view_->chart();

  auto * chart = new QtCharts::QChart();
  chart->legend()->setVisible(true);
  chart->setTitle("Trajectory kinematics");
  applyChartTheme(chart, pal);

  double xmin = std::numeric_limits<double>::infinity();
  double xmax = -std::numeric_limits<double>::infinity();
  double ymin = std::numeric_limits<double>::infinity();
  double ymax = -std::numeric_limits<double>::infinity();

  for (size_t si = 0; si < series.size(); ++si) {
    const auto & s = series[si];
    auto * ls = new QtCharts::QLineSeries();
    ls->setName(QString::fromStdString(s.label));
    const QColor col = si < colors.size() ? colors[si] : QColor(139, 208, 240);
    ls->setColor(col);

    for (const auto & p : s.points) {
      const double xv = accessAxisValue(p, x_axis);
      const double yv = accessAxisValue(p, y_axis);
      if (std::isfinite(xv) && std::isfinite(yv)) {
        ls->append(xv, yv);
        xmin = std::min(xmin, xv);
        xmax = std::max(xmax, xv);
        ymin = std::min(ymin, yv);
        ymax = std::max(ymax, yv);
      }
    }

    chart->addSeries(ls);
    data_line_series_.push_back(ls);
  }

  auto * crosshair = new QtCharts::QLineSeries();
  crosshair->setName(QString());
  crosshair->setPen(QPen(QColor(255, 255, 255, 210), 1.5, Qt::SolidLine));
  chart->addSeries(crosshair);
  crosshair_series_ = crosshair;

  auto * ax = new QtCharts::QValueAxis();
  ax->setTitleText(QStringLiteral("%1 [%2]")
                     .arg(QString::fromUtf8(axisLabel(x_axis)))
                     .arg(QString::fromUtf8(axisUnit(x_axis))));
  ax->setLabelFormat("%.3f");
  applyAxisTheme(ax, pal);
  x_axis_ptr_ = ax;

  auto * ay = new QtCharts::QValueAxis();
  ay->setTitleText(QStringLiteral("%1 [%2]")
                     .arg(QString::fromUtf8(axisLabel(y_axis)))
                     .arg(QString::fromUtf8(axisUnit(y_axis))));
  ay->setLabelFormat("%.3f");
  applyAxisTheme(ay, pal);
  y_axis_ptr_ = ay;

  applyChartFonts(chart, ax, ay);

  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);

  for (auto * ser : chart->series()) {
    if (ser == crosshair) {
      continue;
    }
    ser->attachAxis(ax);
    ser->attachAxis(ay);
  }
  // Crosshair shares the same value axes so mapToValue() in the chart view stays consistent with
  // data lines.
  crosshair->attachAxis(ax);
  crosshair->attachAxis(ay);

  applyAxisRanges(ax, ay, xmin, xmax, ymin, ymax, range_opts);

  chart_view_->setChart(chart);
  chart_view_->setCrosshairSeries(crosshair);
  chart_view_->setHoverSeriesData(series, colors, x_axis, y_axis);
  for (QtCharts::QLegendMarker * m : chart->legend()->markers(crosshair)) {
    m->setVisible(false);
  }

  std::unique_ptr<QtCharts::QChart> dispose_previous(old_chart);

  last_legend_structure_key_ = buildLegendStructureKey(series, colors, x_axis, y_axis);
}

void TrajectoryKinematicsPlotWidget::updatePlot(
  const std::vector<TrajectorySeriesData> & series, const std::vector<QColor> & colors,
  AxisId x_axis, AxisId y_axis, const PlotAxisRangeOptions & range_opts)
{
  if (tryUpdateSeriesInPlace(series, colors, x_axis, y_axis, range_opts)) {
    return;
  }
  rebuildChart(series, colors, x_axis, y_axis, range_opts);
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

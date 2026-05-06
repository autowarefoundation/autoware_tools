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

#include "comfort_signal.hpp"

#include "metric_utils.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

double factorial(const int n)
{
  double value = 1.0;
  for (int i = 2; i <= n; ++i) {
    value *= static_cast<double>(i);
  }
  return value;
}

std::vector<double> unwrap_angles(const std::vector<double> & values)
{
  if (values.empty()) {
    return {};
  }
  std::vector<double> unwrapped(values.size(), 0.0);
  unwrapped.front() = values.front();
  for (std::size_t i = 1; i < values.size(); ++i) {
    const double delta = autoware_utils_math::normalize_radian(values.at(i) - values.at(i - 1U));
    unwrapped.at(i) = unwrapped.at(i - 1U) + delta;
  }
  return unwrapped;
}

std::optional<std::vector<double>> solve_linear_system(
  std::vector<std::vector<double>> lhs, std::vector<double> rhs)
{
  const std::size_t n = rhs.size();
  for (std::size_t pivot = 0; pivot < n; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < n; ++row) {
      if (std::abs(lhs.at(row).at(pivot)) > std::abs(lhs.at(best).at(pivot))) {
        best = row;
      }
    }
    if (std::abs(lhs.at(best).at(pivot)) < 1.0e-12) {
      return std::nullopt;
    }
    if (best != pivot) {
      std::swap(lhs.at(best), lhs.at(pivot));
      std::swap(rhs.at(best), rhs.at(pivot));
    }
    const double divisor = lhs.at(pivot).at(pivot);
    for (std::size_t col = pivot; col < n; ++col) {
      lhs.at(pivot).at(col) /= divisor;
    }
    rhs.at(pivot) /= divisor;
    for (std::size_t row = 0; row < n; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = lhs.at(row).at(pivot);
      for (std::size_t col = pivot; col < n; ++col) {
        lhs.at(row).at(col) -= factor * lhs.at(pivot).at(col);
      }
      rhs.at(row) -= factor * rhs.at(pivot);
    }
  }
  return rhs;
}

std::vector<double> local_polynomial_filter(
  const std::vector<double> & values, const std::vector<double> & times, const int window_length,
  const int poly_order, const int derivative_order)
{
  std::vector<double> output(values.size(), 0.0);
  if (values.empty() || values.size() != times.size()) {
    return output;
  }
  const int effective_poly_order =
    std::min<int>(poly_order, std::max<int>(0, static_cast<int>(values.size()) - 1));
  if (derivative_order > effective_poly_order) {
    return output;
  }

  const std::size_t coefficients = static_cast<std::size_t>(effective_poly_order + 1);
  const std::size_t target_window =
    std::max<std::size_t>(coefficients, std::min<std::size_t>(window_length, values.size()));

  for (std::size_t i = 0; i < values.size(); ++i) {
    std::size_t start = i > target_window / 2U ? i - target_window / 2U : 0U;
    std::size_t end = std::min(values.size(), start + target_window);
    if (end - start < target_window) {
      start = end > target_window ? end - target_window : 0U;
    }
    std::vector<std::vector<double>> lhs(coefficients, std::vector<double>(coefficients, 0.0));
    std::vector<double> rhs(coefficients, 0.0);
    for (std::size_t sample = start; sample < end; ++sample) {
      const double centered_t = times.at(sample) - times.at(i);
      std::vector<double> powers(coefficients, 1.0);
      for (std::size_t power = 1; power < coefficients; ++power) {
        powers.at(power) = powers.at(power - 1U) * centered_t;
      }
      for (std::size_t row = 0; row < coefficients; ++row) {
        rhs.at(row) += powers.at(row) * values.at(sample);
        for (std::size_t col = 0; col < coefficients; ++col) {
          lhs.at(row).at(col) += powers.at(row) * powers.at(col);
        }
      }
    }
    const auto solution = solve_linear_system(std::move(lhs), std::move(rhs));
    if (solution.has_value()) {
      output.at(i) =
        solution->at(static_cast<std::size_t>(derivative_order)) * factorial(derivative_order);
    }
  }
  return output;
}

}  // namespace

ComfortSignals compute_comfort_signals(
  const std::vector<ComfortSignalInput> & inputs, const double time_offset_s)
{
  ComfortSignals signals;
  std::vector<double> times;
  std::vector<double> yaws;
  std::vector<double> longitudinal_accelerations_raw;
  std::vector<double> lateral_accelerations_raw;
  std::vector<double> acceleration_magnitudes_raw;
  times.reserve(inputs.size());
  yaws.reserve(inputs.size());
  longitudinal_accelerations_raw.reserve(inputs.size());
  lateral_accelerations_raw.reserve(inputs.size());
  acceleration_magnitudes_raw.reserve(inputs.size());

  for (const auto & input : inputs) {
    times.push_back(input.time_s + time_offset_s);
    yaws.push_back(get_yaw(input.pose.orientation));
    longitudinal_accelerations_raw.push_back(input.longitudinal_acceleration_mps2);
    lateral_accelerations_raw.push_back(input.lateral_acceleration_mps2);
    acceleration_magnitudes_raw.push_back(
      std::hypot(input.longitudinal_acceleration_mps2, input.lateral_acceleration_mps2));
  }

  signals.longitudinal_accelerations =
    local_polynomial_filter(longitudinal_accelerations_raw, times, 8, 2, 0);
  signals.lateral_accelerations =
    local_polynomial_filter(lateral_accelerations_raw, times, 8, 2, 0);
  signals.acceleration_magnitudes =
    local_polynomial_filter(acceleration_magnitudes_raw, times, 8, 2, 0);
  signals.longitudinal_jerks =
    local_polynomial_filter(signals.longitudinal_accelerations, times, 15, 2, 1);
  signals.lateral_jerks = local_polynomial_filter(signals.lateral_accelerations, times, 15, 2, 1);
  signals.jerk_magnitudes =
    local_polynomial_filter(signals.acceleration_magnitudes, times, 15, 2, 1);

  const auto unwrapped_yaws = unwrap_angles(yaws);
  signals.yaw_rates = local_polynomial_filter(unwrapped_yaws, times, 15, 2, 1);
  signals.yaw_accelerations = local_polynomial_filter(unwrapped_yaws, times, 15, 3, 2);
  return signals;
}

}  // namespace autoware::planning_data_analyzer::metrics

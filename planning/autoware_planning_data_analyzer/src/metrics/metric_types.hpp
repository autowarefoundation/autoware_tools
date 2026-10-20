// Copyright 2025 TIER IV, Inc.
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

#ifndef METRICS__METRIC_TYPES_HPP_
#define METRICS__METRIC_TYPES_HPP_

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

/**
 * @brief Metric score with availability
 */
struct MetricScore
{
  double score{0.0};
  bool available{false};
};

}  // namespace autoware::planning_data_analyzer::metrics

namespace metrics
{

/**
 * @brief Statistical information
 */
struct Statistics
{
  double mean = 0.0;
  double std_dev = 0.0;
  double max_val = 0.0;
  double min_val = 0.0;

  nlohmann::json to_json() const
  {
    return {{"mean", mean}, {"std", std_dev}, {"max", max_val}, {"min", min_val}};
  }
};

/**
 * @brief Statistics calculation utility (pure function)
 * @tparam Container Numeric container type (e.g., std::vector<double>)
 * @param values Numeric sequence
 * @return Statistical information
 */
template <typename Container>
Statistics calculate_statistics(const Container & values)
{
  Statistics stats;

  if (values.empty()) {
    return stats;
  }

  // Calculate mean
  double sum = 0.0;
  for (const auto & val : values) {
    sum += val;
  }
  stats.mean = sum / values.size();

  // Calculate min/max
  stats.min_val = *std::min_element(values.begin(), values.end());
  stats.max_val = *std::max_element(values.begin(), values.end());

  // Calculate standard deviation
  double variance = 0.0;
  for (const auto & val : values) {
    variance += (val - stats.mean) * (val - stats.mean);
  }
  stats.std_dev = std::sqrt(variance / values.size());

  return stats;
}

inline double compute_percentile(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) return 0.0;
  if (sorted.size() == 1) return sorted[0];
  const double idx = p * static_cast<double>(sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = std::min(lo + 1, sorted.size() - 1);
  const double frac = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

}  // namespace metrics

#endif  // METRICS__METRIC_TYPES_HPP_

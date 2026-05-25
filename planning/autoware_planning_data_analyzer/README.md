# Planning Data Analyzer

## Overview

This package provides offline evaluation tools for trajectory planning performance analysis from recorded rosbag data.

## Open-Loop Evaluation

Open-loop evaluation computes both trajectory error metrics and planning-quality subscores.

### Trajectory Error Metrics

For each evaluated trajectory, the following are computed:

- ADE
- FDE
- AHE
- FHE
- lateral deviation
- longitudinal deviation
- TTC trace
- per-horizon metrics (full, and configured horizons such as 1.0s, 2.0s, 4.0s, 8.0s)

### Planning Quality Score (EPDMS)

To show multi-facet (comfort, progress, safety) quality of generated trajectory, EPDMS (Extended Predictive Driver Model Score) is computed based on the multiple subscores.

Detailed definitions and equations for each subscore can be found in the [EPDMS Metrics Document](docs/metrics/epdms_metrics.md).

Two variant topics are exported:

- synthetic_epdms_raw
- synthetic_epdms_human_filtered

### EPDMS's Subscores

EPDMS is obtained from combining the following subscores:

- history_comfort
- extended_comfort
- time_to_collision_within_bound
- lane_keeping
- ego_progress
- drivable_area_compliance
- no_at_fault_collision
- driving_direction_compliance
- traffic_light_compliance

### Evaluator Metrics (bag-recorded scalars)

The analyzer reads scalar metric topics from the input rosbag according to **[config/evaluator_config.yaml](config/evaluator_config.yaml)** (e.g. `control_evaluator`, `planning_evaluator`).
Output is written to **`evaluator_metric.json`** (separate from trajectory metrics).
If no configured topic has valid measurements, the file is not written.

The package default `config/evaluator_config.yaml` is used automatically.
Override only when needed: launch arg `evaluator_config_file`, or `-p open_loop.evaluator_config_path:=/path/to.yaml`.

When a single `evaluator_config` entry is configured, keys are flat at the top level (e.g. `all/<metric_key>/mean`).
Multiple entries are nested under each YAML `name`.

Aggregation views (per metric key):

- `all/<metric_key>/` — all odometry-synced measurements from the rosbag
- `included/<metric_key>/` — measurements not matching **any** exclusion rule
- `excluded/<rule_name>/<metric_key>/` — measurements matching that rule

Each view exports `count`, `mean`, `min`, `max`, `percentile_95`, `percentile_99`, and per-metric
`description` (aggregates of evaluator values recorded in the evaluation rosbag).

Supported exclusion rules: `intersection_area` (ego pose inside vector-map `intersection_area` polygon)

Odometry samples within 2 m of the first pose (pre-departure waiting) are excluded from aggregation.

## Quick Start

### Open Loop Evaluation

```sh
ros2 launch autoware_planning_data_analyzer planning_data_analyzer.launch.xml \
  bag_path:=$HOME/test_bag.mcap \
  output_dir:=$HOME/planning_analyzer_output
```

## Output Files

- `time_step_based_trajectory_metric.json` — open-loop trajectory aggregate metrics
- `time_step_based_trajectory_result.jsonl` — per-trajectory DLR-style results
- `time_step_based_trajectory_detailed_result.json` — full per-trajectory dump
- `evaluator_metric.json` — evaluator metric aggregates per `evaluator_config` (if bag contains configured topics)

## Parameters

Key parameters in `config/planning_data_analyzer.param.yaml`:

- `evaluation.mode`: `open_loop`
- `evaluation_interval_ms`: odometry sampling interval (default 100 ms)
- `sync_tolerance_ms`: evaluator/trajectory topic sync tolerance (default 100 ms)
- `open_loop.evaluator_config_path`: path to evaluator metric groups YAML (`name`, `topics`,
  `exclusion.rules`); launch arg `evaluator_config_file` sets this (package default if omitted)

---

## For More Information

- Configuration: [config/planning_data_analyzer.param.yaml](config/planning_data_analyzer.param.yaml), [config/evaluator_config.yaml](config/evaluator_config.yaml)
- Launch files: [launch/](launch/)

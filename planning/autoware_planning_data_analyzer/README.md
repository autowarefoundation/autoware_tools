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

### Plnnaing Quality Score (EPDMS)

To show multi-facet (comfort, progress, safety) quality of generated trajectory, EPDMS (Extended Predictive Driver Model Score) is computed based on the multiple subscores.

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

These subscores are exported both as per-trajectory outputs and as aggregate statistics.

## Quick Start

### Open Loop Evaluation

```sh
ros2 run autoware_planning_data_analyzer autoware_planning_data_analyzer_node --ros-args \
  -p bag_path:=~/my_bag \
  -p evaluation.mode:=open_loop \
  -p trajectory_topic:=/planning/trajectory \
  -p json_output_path:=~/results.json
```

## Output Files

- **`evaluation_result.json`** - Detailed metrics (ADE, FDE, TTC, etc.) and summary statistics
- **`evaluation_output.bag/`** - Evaluation metrics as rosbag for visualization
- **`debug_images/*.png`** - Trajectory visualization (with `--viz` flag)

## Parameters

Key parameters in `config/planning_data_analyzer.param.yaml`:

- `evaluation.mode`: Evaluation mode (`open_loop`)
- `evaluation_interval_ms`: Sampling interval (default: 100ms)
- `sync_tolerance_ms`: Time synchronization tolerance (default: 100ms)
- `trajectory_topic`: Trajectory topic to evaluate
- `open_loop.*`: Open-loop specific settings

---

## For More Information

- Configuration: [config/planning_data_analyzer.param.yaml](config/planning_data_analyzer.param.yaml)
- Launch files: [launch/](launch/)

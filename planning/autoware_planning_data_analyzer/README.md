# Planning Data Analyzer

## Overview

This package provides offline evaluation tools for trajectory planning performance analysis from recorded rosbag data.

## Evaluation Modes

1. **Open Loop**: Evaluate prediction accuracy against ground truth trajectories
2. **OR Scene**: Evaluate override regression scenarios (LIVE vs HISTORICAL)

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

### OR Scene Evaluation

```sh
# Using the unified script
./install/autoware_planning_data_analyzer/share/autoware_planning_data_analyzer/scripts/run_evaluation.sh \
  -b ~/result_bag.mcap \
  -m or_scene \
  -t /planning/diffusion_planner/trajectory \
  -o ~/results \
  --live \
  --viz
```

### Multi-Model Pipeline

```sh
# Evaluate multiple models from DLR results
./install/autoware_planning_data_analyzer/share/autoware_planning_data_analyzer/scripts/multi_model_pipeline.sh \
  --scenario ~/dataset/scenario.yaml \
  --trajectory /planning/model_a/trajectory \
  --trajectory /planning/model_b/trajectory \
  --output ~/comparison \
  --viz
```

## Output Files

- **`evaluation_result.json`** - Detailed metrics (ADE, FDE, TTC, etc.) and summary statistics
- **`evaluation_output.bag/`** - Evaluation metrics as rosbag for visualization
- **`debug_images/*.png`** - Trajectory visualization (with `--viz` flag)

## Parameters

Key parameters in `config/offline_evaluation.param.yaml`:

- `evaluation.mode`: Evaluation mode (`open_loop` or `or_scene`)
- `evaluation_interval_ms`: Sampling interval (default: 100ms)
- `sync_tolerance_ms`: Time synchronization tolerance (default: 100ms)
- `trajectory_topic`: Trajectory topic to evaluate
- `or_scene_evaluation.*`: OR scene specific settings

### Utility Scripts

Located in `scripts/` directory:

- **Bag Processing**: `merge_bags.py`, `rename_bag_topic.py`, `add_prefix_to_bag.py`
- **Ground Truth**: `add_gt_trajectory_to_bag.py`
- **OR Analysis**: `detect_or_and_route.py`, `evaluate_or_segments.py`
- **Visualization**: `generate_or_visualization.py`
- **Comparison**: `compare_live_historical.py`

See `scripts/README.md` for detailed documentation.

---

## For More Information

- Offline evaluation scripts: [scripts/README.md](scripts/README.md)
- Configuration: [config/offline_evaluation.param.yaml](config/offline_evaluation.param.yaml)
- Launch files: [launch/](launch/)

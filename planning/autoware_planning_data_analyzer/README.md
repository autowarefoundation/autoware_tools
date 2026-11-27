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

These subscores are exported both as per-trajectory outputs and as aggregate statistics.

### Evaluator Metrics (bag-recorded scalars)

It aggregates the results of filtering the topics specified in `config/evaluator_config.yaml` according to the exception rules.

- lateral_deviation_centerline
  - for excluded `intersection_area`

## Closed-Loop Evaluation

Closed-loop evaluation judges one simulated run, for example a planning simulation that replays a
recorded scene and lets Autoware drive from the scene window's start. The input bag is the run's
bag; a scene file says which part of it is judged and where the recorded (human) drive is.

The run is laid out on 0.1 s rows from the window's start: the executed ego from the odometry
topic, and road users (pedestrians, bicycles and vehicles) from the tracked-objects topic. A road
user seen for less than 1 s is a detection blip and is dropped, and its speed on a row is fitted
to its positions within 0.5 s either side (replayed tracks carry no reliable velocity).

The metrics and their thresholds:

- `no_at_fault_collision` (NC) of the driven path, from the window's start until the run's end,
  with each road user judged at its first contact, as NAVSIM does. A road user centred behind the
  ego's rear bumper and heading within 45 deg of the ego drove into it: not the ego's fault.
- `margin` (lane changes, avoidances, pull-outs): no contact the ego causes, judged by the rule
  above with straddling read on lanes (a lanelet with up to two lanelets before and after it, so
  the seam between consecutive lanelets is not straddling) and excused while the ego is within
  0.5 m of the recorded path, and the closest box-to-box distance to every road user that counts,
  in its role. A lane change or a pull-out counts every candidate road user (within 15 m, or 5 m
  for pedestrians and bicycles) while the ego is out of its start lane; an avoidance counts
  standing vehicles (avoided objects) and oncoming traffic; an easy scene counts every road user.
  Every line is 0 m for now, so only a collision fails. A road user that drives more than 0.5 m
  into the ego within 3 s after a contact that is not the ego's fault stops the judging (a
  replayed road user cannot react).
- The scene's verdict is margin's for lane changes, avoidances and pull-outs, and NC's for other
  labels; a metric without a verdict leaves the scene without one.

### Scene File

```json
{
  "schema_version": 1,
  "scene_id": "dataset/2026-11-06/13-59-17@1793941903441943903",
  "eval_label": "lane_change",
  "difficulty": "easy",
  "window": { "start_ns": 1793941903441943903, "end_ns": 1793941913441979467 },
  "run_end_ns": 1793941913441979467,
  "reference": { "bag_path": "/path/to/recorded_bag", "time_offset_ns": 0 }
}
```

- `window` and `run_end_ns` (default: the window's end) are in the clock of the evaluated bag, or
  in the recorded drive's with `"window_clock": "recorded"`; they then move to the evaluated
  bag's clock by the reference's offset.
- `reference` (optional) is the recorded drive: a bag and the offset that turns its times into
  the evaluated bag's (evaluated = recorded + offset). Without it, the evaluated bag's own odometry
  is the recorded drive, e.g. when a recorded bag itself is evaluated. `"time_offset_ns": "auto"`
  (with `"window_clock": "recorded"`) finds the offset by lining the two drives up over `match_s`
  (default 5 s) before the window's start, where the simulation replays the recorded drive.
- `margin` (optional): `kind` (`lane_change`, `avoidance`, `pull_out`; default from `eval_label`),
  `trailing_vehicle` (an avoidance with a trailing vehicle: every other road user counts as in a
  lane change) and `every_road_user` (default: an easy scene).
- `ignored_track_ids` (optional): road users never judged, e.g. known tracking errors.

## Quick Start

### Open Loop Evaluation

```sh
ros2 run autoware_planning_data_analyzer autoware_planning_data_analyzer_node --ros-args \
  -p bag_path:=~/my_bag \
  -p evaluation.mode:=open_loop \
  -p trajectory_topic:=/planning/trajectory \
  -p json_output_path:=~/results.json
```

### Closed Loop Evaluation

```sh
ros2 run autoware_planning_data_analyzer autoware_planning_data_analyzer_node --ros-args \
  --params-file $(ros2 pkg prefix autoware_planning_data_analyzer)/share/autoware_planning_data_analyzer/config/planning_data_analyzer.param.yaml \
  --params-file <vehicle_description>/config/vehicle_info.param.yaml \
  -p evaluation.mode:=closed_loop \
  -p input_bag_path:=/path/to/simulation_bag \
  -p output_dir:=/path/to/output \
  -p closed_loop.scene_path:=/path/to/scene.json \
  -p closed_loop.map_path:=/path/to/lanelet2_map.osm
```

## Output Files

- **`evaluation_result.json`** - Detailed metrics (ADE, FDE, TTC, etc.) and summary statistics
- **`evaluation_output.bag/`** - Evaluation metrics as rosbag for visualization
- **`debug_images/*.png`** - Trajectory visualization (with `--viz` flag)
- **evaluator_metric.json** — evaluator metric aggregates per `evaluator_config` (if bag contains configured topics)
- **`closed_loop_result.json`** - Closed loop: the scene, the episode's rows, NC with every first contact, each metric's result, and the verdict (`passed`: true, false, or null when it cannot be judged)

## Parameters

Key parameters in `config/planning_data_analyzer.param.yaml`:

- `evaluation.mode`: Evaluation mode (`open_loop` or `closed_loop`)
- `evaluation_interval_ms`: Sampling interval (default: 100ms)
- `sync_tolerance_ms`: Time synchronization tolerance (default: 100ms)
- `trajectory_topic`: Trajectory topic to evaluate
- `open_loop.enable_epdms_calculation`: Enables EPDMS subscore, synthetic EPDMS, and EPDMS debug calculation
- `open_loop.*`: Open-loop specific settings
- `closed_loop.scene_path`: Scene file of the run (closed loop)
- `closed_loop.map_path`: `lanelet2_map.osm` with `map_projector_info.yaml` beside it; empty: `map_topic` in the bag (closed loop)

---

## For More Information

- Configuration: [config/planning_data_analyzer.param.yaml](config/planning_data_analyzer.param.yaml), [config/evaluator_config.yaml](config/evaluator_config.yaml)
- Launch files: [launch/](launch/)

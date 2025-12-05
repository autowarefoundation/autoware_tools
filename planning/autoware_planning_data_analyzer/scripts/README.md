# Offline Evaluation Scripts

Utility scripts for offline trajectory evaluation and bag processing.

## Main Scripts

### `run_evaluation.sh` - Unified Evaluation Runner

Run offline evaluation in any mode (open_loop, closed_loop, or_scene).

**Usage:**
```bash
./run_evaluation.sh -b <bag_path> -m <mode> -t <trajectory_topic> -o <output_dir> [OPTIONS]
```

**Examples:**
```bash
# Open loop evaluation
./run_evaluation.sh -b ~/bag -m open_loop -t /planning/trajectory -o ~/results

# OR scene evaluation with visualization
./run_evaluation.sh -b ~/result_bag.mcap -m or_scene --live --viz -o ~/results
```

**Options:**
- `-b, --bag PATH`: Input bag file or directory
- `-m, --mode MODE`: Evaluation mode (open_loop, closed_loop, or_scene)
- `-t, --trajectory TOPIC`: Trajectory topic to evaluate
- `-o, --output DIR`: Output directory
- `--live`: Evaluate LIVE trajectories (or_scene mode)
- `--historical`: Evaluate HISTORICAL trajectories (or_scene mode)
- `--viz`: Enable debug visualization
- `--map PATH`: Map directory path (optional)

---

### `multi_model_pipeline.sh` - Multi-Model Evaluation

Evaluate multiple trajectory models from DLR results.

**Usage:**
```bash
./multi_model_pipeline.sh --scenario <path> --trajectory <topic> --output <dir> [OPTIONS]
```

**Examples:**
```bash
# Run DLR and evaluate single model
./multi_model_pipeline.sh \
  --scenario ~/dataset/scenario.yaml \
  --trajectory /planning/diffusion_planner/trajectory \
  --output ~/results

# Evaluate multiple models from existing DLR result
./multi_model_pipeline.sh \
  --skip-dlr \
  --result-bag ~/dataset/out/latest/result_bag/result_bag_0.mcap \
  --input-bag ~/dataset/input_bag \
  --trajectory /planning/model_a/trajectory \
  --trajectory /planning/model_b/trajectory \
  --output ~/results --viz
```

**Options:**
- `--scenario PATH`: T4 dataset scenario.yaml
- `--trajectory TOPIC`: Trajectory topic (can be specified multiple times)
- `--output DIR`: Output directory
- `--skip-dlr`: Skip DLR simulation, use existing result
- `--result-bag PATH`: Path to existing result bag
- `--input-bag PATH`: Path to input bag
- `--viz`: Enable visualization
- `--map PATH`: Map directory

---

## Utility Scripts

### Bag Processing

- **`merge_bags.py`** - Merge multiple rosbag files
- **`rename_bag_topic.py`** - Rename topics in a bag
- **`add_prefix_to_bag.py`** - Add prefix to topic names
- **`cut_or_segment_with_route.py`** - Extract bag segments around OR events

### Ground Truth & Evaluation

- **`add_gt_trajectory_to_bag.py`** - Generate and add ground truth trajectories
- **`detect_or_and_route.py`** - Detect OR (override) events in bags
- **`evaluate_or_segments.py`** - Evaluate specific OR segments
- **`evaluate_all_live_trajectories.py`** - Batch evaluate all LIVE trajectories

### Analysis & Visualization

- **`generate_or_visualization.py`** - Create visualization images for OR events
- **`compare_live_historical.py`** - Compare LIVE vs HISTORICAL performance
- **`merge_segment_models.py`** - Merge results from multiple models
- **`multi_run_evaluator.py`** - Evaluate multiple runs with statistics

---

## Quick Start

### 1. Simple Open Loop Evaluation

```bash
# Evaluate a single bag
./run_evaluation.sh \
  -b ~/my_bag \
  -m open_loop \
  -t /planning/trajectory \
  -o ~/results
```

### 2. OR Scene Evaluation (LIVE Trajectories)

```bash
# With DLR result bag
./run_evaluation.sh \
  -b ~/dataset/out/latest/result_bag/result_bag_0.mcap \
  -m or_scene \
  -t /planning/diffusion_planner/trajectory \
  -o ~/results \
  --live \
  --viz
```

### 3. Multi-Model Comparison

```bash
# Evaluate 3 different planning models
./multi_model_pipeline.sh \
  --scenario ~/dataset/scenario.yaml \
  --trajectory /planning/diffusion_planner/trajectory \
  --trajectory /planning/optimizer/trajectory \
  --trajectory /planning/sampler/trajectory \
  --output ~/comparison \
  --viz

# Compare results
cd ~/comparison
for dir in */; do
  echo "$dir:"
  jq '.summary.mean_ade' $dir/evaluation_result.json
done
```

---

## Output Files

All evaluation modes produce:
- **`evaluation_result.json`** - Detailed metrics and summary
- **`evaluation_output.bag/`** - Evaluation metrics as rosbag
- **`or_events.json`** - Detected OR events (or_scene mode only)

With `--viz` enabled:
- **`debug_images/*.png`** - Visualization images per OR event

---

## Requirements

- ROS 2 workspace with `autoware_planning_data_analyzer` built
- Python 3.8+ with rosbag2_py
- For DLR integration: driving_log_replayer_v2

---

## Troubleshooting

### "No storage could be initialized"
Run `ros2 bag reindex -s sqlite3 <bag_dir>` to create metadata.yaml

### "Database directory already exists"
Delete existing output: `rm -rf <output_path>`

### Missing trajectories in result bag
Ensure DLR scenario.yaml has `publish_profile: planning_control`

---

For more details, see the main package README.

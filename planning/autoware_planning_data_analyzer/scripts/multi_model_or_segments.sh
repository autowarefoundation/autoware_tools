#!/bin/bash
#
# Multi-Model OR Segment Evaluation
#
# Automates:
# 1. Detecting OR events
# 2. Cutting rosbag into OR segments (with route injection)
# 3. Running multiple model weights through DLR on each segment
# 4. Generating final bags with all LIVE trajectories
#

set -e  # Exit on error
set -o pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
die() {
    echo -e "${RED}ERROR: $1${NC}" >&2
    cleanup_on_error
    exit 1
}

info() {
    echo -e "${GREEN}$1${NC}"
}

warn() {
    echo -e "${YELLOW}WARNING: $1${NC}"
}

cleanup_on_error() {
    if [ -f "${CONFIG_FILE}.backup" ]; then
        mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"
        info "✓ Restored original config"
    fi
}

# Trap errors
trap cleanup_on_error ERR

# Parse arguments
usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Required:
  --models MODEL_PATH1 MODEL_PATH2 ...
                        Paths to model weight directories (e.g., /path/to/v2.0 /path/to/v2.1)
  --config CONFIG_FILE  Path to diffusion_planner.param.yaml
  --dataset DATASET_DIR Path to T4 dataset directory
  --output OUTPUT_DIR   Output directory for results

Optional:
  --before-or SECONDS   Seconds before OR to include in segment (default: 5)
  --after-or SECONDS    Seconds after OR to include in segment (default: 10)
  --help                Show this help message

Example:
  $0 --models ~/autoware_data/diffusion_planner/v2.0 ~/autoware_data/diffusion_planner/v2.1 \\
     --config ~/pilot-auto/src/autoware/universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml \\
     --dataset /path/to/t4_dataset \\
     --output /tmp/multi_model_segments
EOF
}

# Default values
BEFORE_OR=5
AFTER_OR=10
MODELS=()

# Parse command line
while [[ $# -gt 0 ]]; do
    case $1 in
        --models)
            shift
            while [[ $# -gt 0 ]] && [[ ! $1 =~ ^-- ]]; do
                MODELS+=("$1")
                shift
            done
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --dataset)
            DATASET="$2"
            shift 2
            ;;
        --output)
            OUTPUT="$2"
            shift 2
            ;;
        --before-or)
            BEFORE_OR="$2"
            shift 2
            ;;
        --after-or)
            AFTER_OR="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Validate required arguments
[[ ${#MODELS[@]} -eq 0 ]] && die "No models specified. Use --models"
[[ -z "$CONFIG_FILE" ]] && die "Config file not specified. Use --config"
[[ -z "$DATASET" ]] && die "Dataset not specified. Use --dataset"
[[ -z "$OUTPUT" ]] && die "Output directory not specified. Use --output"

info "=========================================="
info "Multi-Model OR Segment Evaluation"
info "=========================================="
echo ""
echo "Models: ${MODELS[@]}"
echo "Config: $CONFIG_FILE"
echo "Dataset: $DATASET"
echo "Output: $OUTPUT"
echo "Time window: OR - ${BEFORE_OR}s to OR + ${AFTER_OR}s"
echo ""

# PHASE 1: Validation
info "Phase 1: Validating inputs..."

# Check models
for model_path in "${MODELS[@]}"; do
    [[ -d "$model_path" ]] || die "Model directory not found: $model_path"

    # Check required files
    [[ -f "$model_path/diffusion_planner.onnx" ]] || die "Missing: $model_path/diffusion_planner.onnx"
    [[ -f "$model_path/diffusion_planner.param.json" ]] || die "Missing: $model_path/diffusion_planner.param.json"

    # CRITICAL: Check engine file (any .engine file in directory)
    engine_count=$(find "$model_path" -maxdepth 1 -name "*.engine" | wc -l)
    if [[ $engine_count -eq 0 ]]; then
        echo ""
        die "TensorRT engine file not found in: $model_path

You MUST generate the engine file BEFORE running this script.
Without it, Autoware will build the engine during DLR replay, causing:
  - 5-10 minute delays
  - Missed trajectory recording windows
  - Incomplete LIVE data

To generate the engine file:
  1. Update $CONFIG_FILE to point to $model_path
  2. Launch Autoware planning simulator once
  3. Wait for 'TensorRT engine setup completed' message
  4. Verify .engine file created in $model_path
  5. Then re-run this script"
    fi

    model_name=$(basename "$model_path")
    info "  ✓ Model validated: $model_name"
done

# Check config file
[[ -f "$CONFIG_FILE" ]] || die "Config file not found: $CONFIG_FILE"
info "  ✓ Config file found"

# Check dataset
[[ -f "$DATASET/scenario.yaml" ]] || die "Missing: $DATASET/scenario.yaml"
[[ -d "$DATASET/map" ]] || die "Missing: $DATASET/map/"
[[ -f "$DATASET/map/lanelet2_map.osm" ]] || die "Missing: $DATASET/map/lanelet2_map.osm"

# Find input bag
if [[ -d "$DATASET/input_bag" ]]; then
    # Count rosbag directories (each bag is a subdirectory with .db3 or .mcap files)
    bag_count=$(find "$DATASET/input_bag" -mindepth 1 -maxdepth 1 -type d | wc -l)

    if [[ $bag_count -eq 0 ]]; then
        # No subdirectories, input_bag itself is the bag
        INPUT_BAG="$DATASET/input_bag"
        info "  ✓ Single rosbag found"
    elif [[ $bag_count -eq 1 ]]; then
        # One subdirectory, use it
        INPUT_BAG=$(find "$DATASET/input_bag" -mindepth 1 -maxdepth 1 -type d | head -1)
        info "  ✓ Single rosbag found"
    else
        # Multiple bags, need to merge
        info "  Found $bag_count rosbags, merging..."

        # Create output directory first (needed for merged bag)
        mkdir -p "$OUTPUT"
        MERGED_BAG="$OUTPUT/merged_input_bag"

        # Source ros2bag_extensions
        source ~/ros2_ws/install/setup.bash

        # Get all bag directories
        bag_dirs=$(find "$DATASET/input_bag" -mindepth 1 -maxdepth 1 -type d | sort)

        # Merge bags
        rm -rf "$MERGED_BAG"
        ros2 bag merge -o "$MERGED_BAG" $bag_dirs || die "Failed to merge rosbags"

        INPUT_BAG="$MERGED_BAG"
        info "  ✓ Rosbags merged into: $MERGED_BAG"

        # Re-source pilot-auto (was overridden by ros2_ws)
        source ~/pilot-auto/install/setup.bash
    fi
else
    die "Input bag not found in $DATASET/input_bag"
fi
info "  ✓ Dataset validated"

# Create output directory (if not already created during merge)
mkdir -p "$OUTPUT"
info "  ✓ Output directory: $OUTPUT"

echo ""

# PHASE 2: Detect OR events and extract route
info "Phase 2: Detecting OR events and extracting route..."

source ~/pilot-auto/install/setup.bash

python3 "$SCRIPT_DIR/detect_or_and_route.py" \
    --input "$INPUT_BAG" \
    --output "$OUTPUT/or_events.json" \
    --route-output "$OUTPUT/route_message.bin" \
    --before "$BEFORE_OR" \
    --after "$AFTER_OR" \
    || die "OR detection failed"

# Read number of OR events
OR_COUNT=$(jq '.or_events | length' "$OUTPUT/or_events.json")
info "✓ Detected $OR_COUNT OR events"

echo ""

# PHASE 3: Cut segments with route injection
info "Phase 3: Cutting OR segments..."

mkdir -p "$OUTPUT/segments"

for i in $(seq 0 $((OR_COUNT - 1))); do
    event_info=$(jq ".or_events[$i]" "$OUTPUT/or_events.json")
    start_ns=$(echo "$event_info" | jq -r '.segment_start_ns')
    end_ns=$(echo "$event_info" | jq -r '.segment_end_ns')
    or_time=$(echo "$event_info" | jq -r '.or_timestamp_sec')

    segment_dir="$OUTPUT/segments/or_event_$i"
    mkdir -p "$segment_dir"

    echo "Cutting OR event #$i (OR at t=${or_time}s)..."

    python3 "$SCRIPT_DIR/cut_or_segment_with_route.py" \
        --input "$INPUT_BAG" \
        --output "$segment_dir" \
        --start "$start_ns" \
        --end "$end_ns" \
        --route-data "$OUTPUT/route_message.bin" \
        || die "Failed to cut segment $i"

    info "  ✓ Segment $i created"
done

echo ""

# PHASE 4: Add GT to each segment
info "Phase 4: Adding ground truth to segments..."

for i in $(seq 0 $((OR_COUNT - 1))); do
    segment_dir="$OUTPUT/segments/or_event_$i"
    segment_with_gt="$OUTPUT/segments/or_event_${i}_with_gt"

    echo "Adding GT to segment $i..."

    python3 ~/pilot-auto/src/autoware/new_planning_framework/autoware_offline_evaluation_tools/scripts/add_gt_trajectory_to_bag.py \
        --input "$segment_dir" \
        --output "$segment_with_gt" \
        --horizon 8.0 \
        --resolution 0.1 \
        || die "Failed to add GT to segment $i"

    info "  ✓ GT added to segment $i"
done

echo ""

# PHASE 5: Multi-model DLR iterations
info "Phase 5: Running multi-model DLR iterations..."

# Backup config
cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"
info "  ✓ Backed up config: ${CONFIG_FILE}.backup"

# For each segment
for segment_id in $(seq 0 $((OR_COUNT - 1))); do
    echo ""
    info "=========================================="
    info "Processing OR Segment #$segment_id"
    info "=========================================="

    segment_with_gt="$OUTPUT/segments/or_event_${segment_id}_with_gt"

    # For each model (all use SAME input segment - no chaining!)
    for model_idx in "${!MODELS[@]}"; do
        model_path="${MODELS[$model_idx]}"
        model_name=$(basename "$model_path")
        output_dir="$OUTPUT/or_event_${segment_id}_${model_name}_output"

        echo ""
        echo "--- Segment $segment_id | Model $model_name ---"

        # Update config
        sed -i "s|onnx_model_path:.*|onnx_model_path: ${model_path}/diffusion_planner.onnx|" "$CONFIG_FILE"
        sed -i "s|args_path:.*|args_path: ${model_path}/diffusion_planner.param.json|" "$CONFIG_FILE"
        echo "  ✓ Config updated for $model_name"

        # Run DLR with topic remap (each model uses SAME segment input)
        prefixed_traj_topic="/${model_name}/planning/trajectory_generator/diffusion_planner_node/output/trajectory"
        dlr_cmd="ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py"
        dlr_cmd="$dlr_cmd scenario_path:=${DATASET}/scenario.yaml"
        dlr_cmd="$dlr_cmd input_bag:=${segment_with_gt}"
        dlr_cmd="$dlr_cmd output_dir:=${output_dir}"
        dlr_cmd="$dlr_cmd base_trajectory_topic:=${prefixed_traj_topic}"

        echo "  Running DLR (trajectory remapped to ${prefixed_traj_topic})..."
        rm -rf "$output_dir"
        eval "$dlr_cmd" || die "DLR failed for segment $segment_id, model $model_name"

        # Verify result
        result_bag="${output_dir}/result_bag"
        [[ -d "$result_bag" ]] || die "Result bag not created: $result_bag"

        info "  ✓ DLR completed for $model_name"

        # Add prefixed trajectory topic
        echo "  Adding prefixed trajectory..."
        python3 "$SCRIPT_DIR/add_prefix_to_bag.py" \
            --bag "$result_bag" \
            --prefix "$model_name" \
            || die "Failed to add prefix for $model_name"
    done

    # Merge all model outputs for this segment into final bag
    final_bag="$OUTPUT/or_event_${segment_id}"
    echo ""
    echo "  Merging all models for segment $segment_id..."

    # Build model names list
    model_names=""
    for model_path in "${MODELS[@]}"; do
        model_name=$(basename "$model_path")
        model_names="$model_names $model_name"
    done

    # Run merge script
    python3 "$SCRIPT_DIR/merge_segment_models.py" \
        --output-dir "$OUTPUT" \
        --segment-id "$segment_id" \
        --models $model_names \
        --final-output "$final_bag" \
        || die "Failed to merge segment $segment_id"

    # Save final bag path for this segment (ls returns full path, don't double it)
    SEGMENT_FINAL_BAG[$segment_id]="$(ls $final_bag/*.mcap 2>/dev/null | head -1)"

    info "✓ Segment $segment_id complete with all ${#MODELS[@]} models"
done

# Restore config
mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"
info "\n✓ Restored original config"

echo ""

# PHASE 6: Summary
info "=========================================="
info "COMPLETE!"
info "=========================================="
echo ""
echo "Final bags (one per OR event, all models included):"
echo ""

for i in $(seq 0 $((OR_COUNT - 1))); do
    final_bag="${SEGMENT_FINAL_BAG[$i]}"
    or_time=$(jq -r ".or_events[$i].or_timestamp_sec" "$OUTPUT/or_events.json")

    echo "OR Event #$i (at t=${or_time}s):"
    echo "  File: $final_bag"
    echo "  Size: $(du -h "$final_bag" | cut -f1)"
    echo ""

    # Show key topics
    source ~/pilot-auto/install/setup.bash
    ros2 bag info "$final_bag" 2>/dev/null | grep -E "(Duration|Messages)" | head -2 | sed 's/^/  /'

    echo "  LIVE Trajectories:"
    for model_path in "${MODELS[@]}"; do
        model_name=$(basename "$model_path")
        count=$(ros2 bag info "$final_bag" 2>/dev/null | grep "/${model_name}/planning" | grep -oP 'Count: \K[0-9]+' || echo "0")
        echo "    /$model_name/planning/.../trajectory: $count messages"
    done

    gt_count=$(ros2 bag info "$final_bag" 2>/dev/null | grep "/ground_truth/trajectory" | grep -oP 'Count: \K[0-9]+' || echo "0")
    echo "    /ground_truth/trajectory: $gt_count messages"
    echo ""
done

echo "Next steps:"
echo "  - Use these bags for OR scene evaluation"
echo "  - Each bag is self-contained for one OR event"
echo "  - All models' LIVE trajectories are included with prefixes"
echo ""
info "=========================================="

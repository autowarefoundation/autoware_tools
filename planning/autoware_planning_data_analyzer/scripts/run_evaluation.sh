#!/bin/bash
# Unified evaluation script for offline trajectory evaluation
# Supports open_loop, closed_loop, and or_scene evaluation modes

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -b, --bag PATH          Path to input bag file or directory"
    echo "  -m, --mode MODE         Evaluation mode: open_loop, closed_loop, or_scene (default: open_loop)"
    echo "  -t, --trajectory TOPIC  Trajectory topic to evaluate"
    echo "  -o, --output DIR        Output directory for results"
    echo "  --map PATH              Path to map directory (optional)"
    echo "  --live                  Evaluate LIVE trajectories (for or_scene mode)"
    echo "  --historical            Evaluate HISTORICAL trajectories (for or_scene mode)"
    echo "  --viz                   Enable debug visualization (for or_scene mode)"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  # Open loop evaluation"
    echo "  $0 -b ~/bag -m open_loop -t /planning/trajectory -o ~/results"
    echo ""
    echo "  # OR scene evaluation with LIVE trajectories"
    echo "  $0 -b ~/result_bag.mcap -m or_scene --live --viz -o ~/results"
    exit 1
}

# Default values
MODE="open_loop"
EVALUATE_LIVE="true"
ENABLE_VIZ="false"
BAG_PATH=""
TRAJECTORY_TOPIC=""
OUTPUT_DIR=""
MAP_PATH=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bag)
            BAG_PATH="$2"
            shift 2
            ;;
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        -t|--trajectory)
            TRAJECTORY_TOPIC="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --map)
            MAP_PATH="$2"
            shift 2
            ;;
        --live)
            EVALUATE_LIVE="true"
            shift
            ;;
        --historical)
            EVALUATE_LIVE="false"
            shift
            ;;
        --viz)
            ENABLE_VIZ="true"
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            ;;
    esac
done

# Validate required arguments
if [ -z "$BAG_PATH" ]; then
    echo -e "${RED}Error: Bag path is required${NC}"
    usage
fi

if [ -z "$OUTPUT_DIR" ]; then
    echo -e "${RED}Error: Output directory is required${NC}"
    usage
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}Starting $MODE evaluation...${NC}"
echo "Bag: $BAG_PATH"
echo "Output: $OUTPUT_DIR"

# Build ROS 2 parameters
PARAMS="-p bag_path:=$BAG_PATH"
PARAMS="$PARAMS -p evaluation.mode:=$MODE"
PARAMS="$PARAMS -p evaluation_output_bag_path:=$OUTPUT_DIR/evaluation_output.bag"
PARAMS="$PARAMS -p json_output_path:=$OUTPUT_DIR/evaluation_result.json"

if [ -n "$TRAJECTORY_TOPIC" ]; then
    PARAMS="$PARAMS -p trajectory_topic:=$TRAJECTORY_TOPIC"
fi

if [ "$MODE" = "or_scene" ]; then
    PARAMS="$PARAMS -p or_scene_evaluation.evaluate_live_trajectories:=$EVALUATE_LIVE"
    PARAMS="$PARAMS -p or_scene_evaluation.enable_debug_visualization:=$ENABLE_VIZ"
    PARAMS="$PARAMS -p or_scene_evaluation.debug_output_dir:=$OUTPUT_DIR/debug_images"
    PARAMS="$PARAMS -p or_scene_evaluation.or_events_output_path:=$OUTPUT_DIR/or_events.json"

    if [ -n "$MAP_PATH" ]; then
        PARAMS="$PARAMS -p or_scene_evaluation.map_path:=$MAP_PATH"
    fi
fi

# Run evaluation
echo -e "${YELLOW}Running autoware_planning_data_analyzer_node...${NC}"
ros2 run autoware_planning_data_analyzer autoware_planning_data_analyzer_node --ros-args $PARAMS

echo -e "${GREEN}Evaluation complete!${NC}"
echo "Results saved to: $OUTPUT_DIR"

if [ "$MODE" = "or_scene" ] && [ "$ENABLE_VIZ" = "true" ]; then
    echo -e "${GREEN}Debug images: $OUTPUT_DIR/debug_images${NC}"
fi

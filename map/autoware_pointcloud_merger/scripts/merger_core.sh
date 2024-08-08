#!/bin/bash

PCD_MER="autoware_pointcloud_merger_node"

# Show usage
function usage() {
    cat <<_EOT_
Usage:
  $0 <PCD_0> ... <PCD_N> <OUTPUT_PCD> <CONFIG_FILE>

Description:
  Merge PCD files into a single PCD file with or without downsampling.

Options:
  None

_EOT_
}

# Parse options
if [ "$OPTIND" = 1 ]; then
    while getopts h OPT; do
        case $OPT in
        h)
            usage
            exit 0
            ;;
        \?)
            echo "Undefined option $OPT"
            usage
            exit 1
            ;;
        esac
    done
else
    echo "No installed getopts-command." 1>&2
    exit 1
fi
shift $((OPTIND - 1))

# Check the number of runtime arguments
if [ "$#" -lt 3 ]; then
    echo "Error: merger_core.sh requires 3 or more arguments."
    usage
    exit 1
fi

# Input arguments
ARGC=$#
ARGV=("$@")

# Total number of input PCD files
N_PCD=$((ARGC - 2))

# Prepare PCD file names
PCD_FILES=()
for ((i = 0; i < N_PCD; i++)); do
    PCD_FILES=("${PCD_FILES[@]}" "${ARGV[i]}")
done

# Prepare other file paths
OUTPUT_PCD=${ARGV[$((ARGC - 2))]}
CONFIG_FILE=${ARGV[$((ARGC - 1))]}

# Call the pointcloud_divider
$PCD_MER "${N_PCD}" "${PCD_FILES[@]}" "${OUTPUT_PCD}" "${CONFIG_FILE}"

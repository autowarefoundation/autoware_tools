#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Analyze and visualize point cloud resolution and point-to-point distances.

This script provides comprehensive analysis tools for point cloud data, including:
- Computing resolution (average point-to-point distance) using k-nearest neighbors
- Visualizing point clouds with color-coded distances
- Generating distance distribution histograms
- Supporting both single PCD files and directories of PCD files

Usage:
    python pointcloud_check.py <input_file> [options]

Arguments:
    input_file          Path to a PCD file or directory containing PCD files

Options:
    --neighbors, -k     Number of nearest neighbors to consider (default: 3)
    --threshold        Distance threshold for visualization (default: 0.2)
    --cache           Enable caching of computed distances to file
    --output, -o      Output path for saving colored point cloud
    --histogram       Show histogram of point-to-point distances

Key Features:
- Automatic merging of multiple point clouds when processing a directory
- Color-coded visualization (blue for distances below threshold, yellow for above)
- Statistical analysis including min, max, median, and standard deviation
- Interactive 3D visualization
- Distance distribution histogram with statistical annotations
- Distance computation result caching support for large point clouds

Example Usage:
    # Analyze a single PCD file with visualization
    python pointcloud_check.py input.pcd --histogram

    # Process directory of PCD files with custom threshold
    python pointcloud_check.py path/to/pcds/ --threshold 0.3

    # Save processed point cloud and cache results, re-run the command will directly utilize the cached results
    python pointcloud_check.py input.pcd --output processed.pcd --cache
"""

import argparse
import logging
import os
from pathlib import Path
from typing import List
from typing import Optional
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from tqdm import tqdm

DEFAULT_K_NEIGHBORS = 3
DEFAULT_THRESHOLD = 0.2
DEFAULT_COLOR_IN_THRESHOLD = [0, 0.1, 0.5]
DEFAULT_COLOR_ABOVE_THRESHOLD = [1, 1, 0.2]
VISUALIZATION_WINDOW_WIDTH = 1024
VISUALIZATION_WINDOW_HEIGHT = 768


def setup_logger() -> logging.Logger:
    """Configure logging settings."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    return logging.getLogger(__name__)


def colorize_pointcloud(
    point_cloud: o3d.geometry.PointCloud,
    distances: np.ndarray,
    threshold: float = DEFAULT_THRESHOLD,
    color_in_threshold: List[float] = DEFAULT_COLOR_IN_THRESHOLD,
    color_above_threshold: List[float] = DEFAULT_COLOR_ABOVE_THRESHOLD,
) -> o3d.geometry.PointCloud:  # Add return type hint
    # Color points based on distance threshold
    colors = np.zeros((len(distances), 3))
    colors[distances >= threshold] = color_above_threshold  # White for distant points
    colors[distances < threshold] = color_in_threshold  # Blue for close points

    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    return point_cloud


def visualize_pointcloud(point_cloud: o3d.geometry.PointCloud):
    """
    Visualize a point cloud using Open3D's visualization utility.

    Args:
        point_cloud (o3d.geometry.PointCloud): The point cloud to visualize.
            Should be a valid Open3D PointCloud object.

    Note:
        This function opens an interactive 3D visualization window where you can:
        - Rotate the view using left mouse button
        - Pan by pressing mouse wheel
        - Zoom using mouse wheel
    """
    o3d.visualization.draw_geometries(
        [point_cloud],
        window_name="Point Cloud Resolution Visualization",
        width=VISUALIZATION_WINDOW_WIDTH,
        height=VISUALIZATION_WINDOW_HEIGHT,
        point_show_normal=False,
    )


def plot_distance_histogram(
    distances: np.ndarray, threshold: float, range_limit: Tuple[float, float] = (0, 0.8)
) -> None:
    """Plot a histogram showing the distribution of point-to-point distances with statistical annotations.

    This function creates a histogram visualization of point cloud distances, including key statistics
    and a threshold line. The histogram shows frequency ratios rather than absolute counts.

    Args:
        distances (np.ndarray): Array of point-to-point distances to be plotted.
            Should contain float values representing distances in meters.
        threshold (float): Distance threshold value to be shown as a vertical line
            in the plot. Used to visualize a critical distance value.
        range_limit (Tuple[float, float], optional): The range of distances to include
            in the histogram, specified as (min, max) in meters.
            Defaults to (0, 0.8).

    Returns:
        None: Displays the plot using matplotlib.pyplot.show()

    Statistics Displayed:
        - Minimum distance
        - Maximum distance
        - Median distance
        - Standard deviation

    Example:
        >>> distances = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
        >>> plot_distance_histogram(distances, threshold=0.3)
    """
    plt.figure(figsize=(10, 6))

    # Calculate histogram with density=True to get frequency ratios
    plt.hist(distances, bins=100, range=range_limit, density=True, stacked=True)

    # Add vertical line for threshold
    plt.axvline(x=threshold, color="r", linestyle="--", label=f"Threshold ({threshold}m)")

    # Calculate statistics
    min_dist = np.min(distances)
    max_dist = np.max(distances)
    median_dist = np.median(distances)
    std_dist = np.std(distances)
    ratio_above_threshold = np.sum(distances > threshold) / len(distances)

    # Create statistics text
    stats_text = (
        f"Min: {min_dist:.6f}m\n"
        f"Max: {max_dist:.6f}m\n"
        f"Median: {median_dist:.6f}m\n"
        f"Std Dev: {std_dist:.6f}m\n"
        f"Above Thr: {ratio_above_threshold:.3f}"
    )

    # Add text box with statistics
    # Position is in axes coordinates (0,0 is bottom left, 1,1 is top right)
    plt.text(
        0.8,
        0.8,
        stats_text,
        transform=plt.gca().transAxes,
        verticalalignment="top",
        horizontalalignment="right",
        # cspell: disable-next-line
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
    )

    plt.title("Point-to-Point Distance Distribution")
    plt.xlabel("Distance (m)")
    plt.ylabel("Frequency Ratio")
    plt.grid(True, alpha=0.3)
    plt.legend()  # Add legend to show threshold line label
    plt.show()


def visualize_results(pcd, distances, args):
    """
    Visualize analysis results including histogram of point-to-point distances and the point cloud itself.

    Args:
        pcd (o3d.geometry.PointCloud): The point cloud to visualize
        distances (numpy.ndarray): Array of point-to-point distances
        args (argparse.Namespace): Command line arguments containing:
            - histogram (bool): Whether to show distance distribution histogram
            - threshold (float): Distance threshold value
            - output (str): Path to save the point cloud (optional)

    The function can:
    1. Display a histogram of point-to-point distances (if args.histogram is True)
    2. Save the point cloud to a file (if args.output is provided)
    3. Show an interactive 3D visualization of the point cloud
    """
    if args.histogram:
        plot_distance_histogram(distances, args.threshold)

    if args.output:
        o3d.io.write_point_cloud(args.output, pcd)

    visualize_pointcloud(pcd)


def load_point_clouds(folder_path: Path) -> List[o3d.geometry.PointCloud]:
    """
    Load all PCD files from the specified directory.

    Args:
        folder_path (Path): Path to the directory containing PCD files.

    Returns:
        List[o3d.geometry.PointCloud]: List of loaded point cloud objects.

    Raises:
        FileNotFoundError: If the specified directory doesn't exist.
    """
    if not folder_path.exists():
        raise FileNotFoundError(f"Directory not found: {folder_path}")

    point_clouds = []
    logger = logging.getLogger(__name__)

    pcd_files = list(folder_path.glob("*.pcd"))
    if not pcd_files:
        logger.warning("No PCD files found in the specified directory.")
        return point_clouds

    for file_path in pcd_files:
        try:
            pcd = o3d.io.read_point_cloud(str(file_path))
            logger.info(f"Loaded {file_path.name} with {len(pcd.points)} points")
            point_clouds.append(pcd)
        except Exception as e:
            logger.error(f"Error loading {file_path}: {str(e)}")

    return point_clouds


def merge_point_clouds(
    point_clouds: List[o3d.geometry.PointCloud],
) -> Optional[o3d.geometry.PointCloud]:
    """
    Merge multiple point clouds into a single point cloud.

    Args:
        point_clouds (List[o3d.geometry.PointCloud]): List of point clouds to merge.

    Returns:
        Optional[o3d.geometry.PointCloud]: Merged point cloud or None if input list is empty.
    """
    if not point_clouds:
        return None

    merged_cloud = point_clouds[0]
    for pc in point_clouds[1:]:
        merged_cloud += pc

    return merged_cloud


def compute_resolution(
    point_cloud: o3d.geometry.PointCloud, k: int = DEFAULT_K_NEIGHBORS
) -> Tuple[float, np.ndarray]:
    """
    Compute the resolution of a point cloud using k-nearest neighbors.

    Args:
        point_cloud (o3d.geometry.PointCloud): Input point cloud
        k (int): Number of nearest neighbors to consider (default: 3)

    Returns:
        Tuple[float, np.ndarray]: Tuple containing:
            - Average resolution (mean distance between points)
            - Array of distances for each point

    Note:
        The resolution is calculated as the average distance between each point
        and its k-1 nearest neighbors (excluding the point itself).
    """
    kdtree = o3d.geometry.KDTreeFlann(point_cloud)
    distances = []

    # cspell: disable-next-line
    for point in tqdm(point_cloud.points, dynamic_ncols=True):
        # Find k nearest neighbors for each point
        [_, idx, dists] = kdtree.search_knn_vector_3d(point, k + 1)
        # Calculate average distance to neighbors (excluding self)
        avg_distance = np.mean(np.sqrt(dists[1:]))
        distances.append(avg_distance)

    distances = np.array(distances)
    resolution = np.mean(distances)

    return resolution, distances


def main():
    parser = argparse.ArgumentParser(description="Compute the resolution of a point cloud file.")
    parser.add_argument("input_file", type=str, help="Input PCD file path")
    parser.add_argument(
        "--neighbors",
        "-k",
        type=int,
        default=3,
        help="Number of nearest neighbors to consider (default: 3)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.2,
        help="distance threshold of the pointcloud visualization",
    )
    parser.add_argument(
        "--cache",
        action="store_true",
        help="Cache the computed distance array into a temporary file, and will try to use cache",
    )
    parser.add_argument(
        "--output", "-o", type=str, help="Output path for saving colored point cloud"
    )
    parser.add_argument(
        "--histogram",
        action="store_true",
        help="Show histogram of point-to-point distances",
    )

    args = parser.parse_args()
    logger = setup_logger()

    try:
        input_path = Path(args.input_file)
        if not input_path.exists():
            raise FileNotFoundError(f"File not found: {input_path}")

        # Load point cloud
        if os.path.isdir(input_path):
            logger.info(
                f"{input_path} is a directory, will merge all the pcd files under the directory"
            )
            pointclouds = load_point_clouds(input_path)
            pcd = merge_point_clouds(pointclouds)
        else:
            logger.info(
                f"{input_path} is a single pcd file, will directly load and analyze the pointcloud"
            )
            pcd = o3d.io.read_point_cloud(str(input_path))
        logger.info(f"Loaded point cloud with {len(pcd.points)} points")

        if args.cache and os.path.isfile(f"{input_path.stem}_distances.npy"):
            distances = np.load(f"{input_path.stem}_distances.npy")
        else:
            # Compute resolution
            resolution, distances = compute_resolution(pcd, args.neighbors)

            if args.cache:
                # Save distances to numpy file
                np.save(f"{input_path.stem}_distances.npy", distances)

            logger.info(f"Point cloud resolution (average point distance): {resolution:.6f}")

        pcd = colorize_pointcloud(pcd, distances, args.threshold)
        visualize_results(pcd, distances, args)

        return 0

    except Exception as e:
        logger.error(f"An error occurred: {str(e)}")
        return 1


if __name__ == "__main__":
    exit(main())

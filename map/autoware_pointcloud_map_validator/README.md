# Autoware Pointcloud Map Validator

Analyze and visualize point cloud resolution and point-to-point distances.

This script provides comprehensive analysis tools for point cloud data, including:

- Computing resolution (average point-to-point distance) using k-nearest neighbors
- Visualizing point clouds with color-coded distances
- Generating distance distribution histograms
- Supporting both single PCD files and directories of PCD files

```yaml
Usage:
    ros2 run autoware_pointcloud_map_validator autoware_pointcloud_checker <input_file> [options]
    # or
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
    # Analyze a single PCD file with visualization and statistic output
    ros2 run autoware_pointcloud_map_validator autoware_pointcloud_checker input.pcd --histogram

    # Process directory of PCD files with custom threshold
    ros2 run autoware_pointcloud_map_validator autoware_pointcloud_checker path/to/pcds/ --threshold 0.3

    # Save processed point cloud and cache results, re-run the command will directly utilize the cached results
    ros2 run autoware_pointcloud_map_validator autoware_pointcloud_checker input.pcd --output processed.pcd --cache
```

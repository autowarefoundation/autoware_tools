# autoware_pointcloud_divider

(Updated 2024/08/16)

This is a tool for processing pcd files, and it can perform the following functions:

- Dividing point clouds
- Downsampling point clouds
- Generating metadata to efficiently handle the divided point clouds

## Supported Data Format

**Currently, only `pcl::PointXYZ` and `pcl::PointXYZ` are supported. Any PCD will be loaded as those two types.**

This tool can be used with files that have data fields other than `XYZI` (e.g., `XYZRGB`) and files that only contain `XYZ`.

- Data fields other than `XYZI` are ignored during loading.
- When loading `XYZ`-only data, the `intensity` field is assigned 0.

## Installation

```bash
cd <PATH_TO_pilot-auto.*> # OR <PATH_TO_autoware>
cd src/
git clone git@github.com:autowarefoundation/autoware_tools.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --symlink-install --packages-up-to pointcloud_divider
```

## Usage

- Select directory, process all files found with `find $INPUT_DIR -name "*.pcd"`.

  ```bash
  ros2 launch autoware_pointcloud_divider pointcloud_divider.launch.xml input_pcd_or_dir:=<INPUT_DIR> output_pcd_dir:=<OUTPUT_DIR> prefix:=<PREFIX>
  ```

  | Name       | Description                           |
  | ---------- | ------------------------------------- |
  | INPUT_DIR  | Directory that contains all PCD files |
  | OUTPUT_DIR | Output directory name                 |
  | PREFIX     | Prefix of output PCD file name        |

`INPUT_DIR` and `OUTPUT_DIR` should be specified as **absolute paths**.

NOTE: The folder `OUTPUT_DIR` is auto generated. If it already exists, all files within that folder will be deleted before the tool runs. Hence, users should backup the important files in that folder if necessary.

### Parameters

{{ json_to_markdown("map/autoware_pointcloud_divider/schema/pointcloud_divider.schema.json") }}

How the point cloud is processed.

![node_diagram](docs/how_to_be_downsampled.drawio.svg)

How the PCD file is named

![node_diagram](docs/output_file_name_pattern.drawio.svg)

### Parameter example

1. Dividing point clouds without downsampling

   ```yaml
   use_large_grid: false
   leaf_size: -1.0 # any negative number
   grid_size_x: 20
   grid_size_y: 20
   ```

2. Dividing and downsampling point clouds

   ```yaml
   use_large_grid: false
   leaf_size: 0.2
   grid_size_x: 20
   grid_size_y: 20
   ```

## Metadata YAML Format

The metadata file should be named `metadata.yaml`. It contains the following fields:

- `x_resolution`: The resolution along the X-axis.
- `y_resolution`: The resolution along the Y-axis.

Additionally, the file contains entries for individual point cloud files (`.pcd` files) and their corresponding grid coordinates. The key is the file name, and the value is a list containing the X and Y coordinates of the lower-left corner of the grid cell associated with that file. The grid cell's boundaries can be calculated using the `x_resolution` and `y_resolution` values.

For example:

```yaml
x_resolution: 100.0
y_resolution: 150.0
A.pcd: [1200, 2500] # -> 1200 <= x <= 1300, 2500 <= y <= 2650
B.pcd: [1300, 2500] # -> 1300 <= x <= 1400, 2500 <= y <= 2650
C.pcd: [1200, 2650] # -> 1200 <= x <= 1300, 2650 <= y <= 2800
D.pcd: [1400, 2650] # -> 1400 <= x <= 1500, 2650 <= y <= 2800
```

## LICENSE

Parts of files grid_info.hpp, pcd_divider.hpp, and pcd_divider.cpp are copied from [MapIV's pointcloud_divider](https://github.com/MapIV/pointcloud_divider) and are under [BSD-3-Clauses](LICENSE) license. The remaining code are under [Apache License 2.0](../../LICENSE)

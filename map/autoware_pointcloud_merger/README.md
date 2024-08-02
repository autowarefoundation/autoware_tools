# autoware_pointcloud_merger

(Updated 2024/06/18)

This is a tool for processing pcd files, and it can perform the following functions:

- Merging multiple PCD files to a single PCD file
- Downsampling point clouds

## Supported Data Format

**Currently, only `pcl::PointXYZI` is supported. Any PCD will be loaded as `pcl::PointXYZI` .**

This tool can be used with files that have data fields other than `XYZI` (e.g., `XYZRGB`) and files that only contain `XYZ`.

- Data fields other than `XYZI` are ignored during loading.
- When loading `XYZ`-only data, the `intensity` field is assigned 0.

## Installation

```bash
cd <PATH_TO_pilot-auto.*> # OR <PATH_TO_autoware>
cd src/
git clone git@github.com:autowarefoundation/autoware_tools.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --symlink-install --packages-up-to pointcloud_merger
```

## Usage

- Select directory, process all files found with `find $INPUT_DIR -name "*.pcd"`.

  ```bash
  pointcloud_merger.sh <INPUT_DIR> <OUTPUT_PCD> <CONFIG>
  ```

- Select individual files

  ```bash
  merger_core.sh <PCD_0> ... <PCD_N> <OUTPUT_PCD> <CONFIG>
  ```

  | Name            | Description                                  |
  | --------------- | -------------------------------------------- |
  | INPUT_DIR       | Directory that contains all PCD files        |
  | PCD_0 ... PCD_N | Input PCD file name                          |
  | OUTPUT_PCD      | Name of the output PCD file                  |
  | CONFIG          | Config file ([default](config/default.yaml)) |

`INPUT_DIR`, `PCD_N`, `OUTPUT_PCD` and `CONFIG` can be specified as both **relative paths** and **absolute paths**.

## Parameter

- **leaf_size** [double]

  The leaf_size of voxel grid filter for pointcloud downsampling. The unit is meters [m].
  If the value is less than or equal to 0, downsampling is skipped.

## LICENSE

Parts of files grid_info.hpp, pointcloud_divider.hpp, pointcloud_divider.cpp, and pointcloud_divider_core.cpp are copied from [MapIV's pointcloud_divider](https://github.com/MapIV/pointcloud_divider) and are under [BSD-3-Clauses](LICENSE) license. The remaining code are under [Apache License 2.0](../../LICENSE)

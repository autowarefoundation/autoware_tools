# autoware_pointcloud_merger

This is a tool for processing pcd files, and it can perform the following functions:

- Merging multiple PCD files to a single PCD file
- Downsampling point clouds

## Supported Data Format

**Currently, only `pcl::PointXYZ` and `pcl::PointXYZI` are supported. Any PCD will be loaded as those two types .**

This tool can be used with files that have data fields other than `XYZI` (e.g., `XYZRGB`) and files that only contain `XYZ`.

- Data fields other than `XYZI` are ignored during loading.
- When loading `XYZ`-only data, the `intensity` field is assigned 0.

## Installation

```bash
cd <PATH_TO_pilot-auto.*> # OR <PATH_TO_autoware>
cd src/
git clone git@github.com:autowarefoundation/autoware_tools.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --symlink-install --packages-up-to autoware_pointcloud_merger
```

## Usage

- Merger all PCD files from the input directory into a single output PCD

  ```bash
  ros2 launch autoware_pointcloud_merger pointcloud_merger.launch.xml input_pcd_dir:=<INPUT_DIR> output_pcd:=<OUTPUT_PCD>
  ```

  | Name       | Description                                 |
  | ---------- | ------------------------------------------- |
  | INPUT_DIR  | Directory that contains all input PCD files |
  | OUTPUT_PCD | Name of the output PCD file                 |

`INPUT_DIR` and `OUTPUT_PCD` should be specified as **absolute paths**.

## Parameter

{{ json_to_markdown("map/autoware_pointcloud_merger/schema/pointcloud_merger.schema.json") }}

## LICENSE

Parts of files pcd_merger.hpp, and pcd_merger.cpp are copied from [MapIV's pointcloud_divider](https://github.com/MapIV/pointcloud_divider) and are under [BSD-3-Clauses](LICENSE) license. The remaining code are under [Apache License 2.0](../../LICENSE)

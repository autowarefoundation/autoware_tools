# pointcloud_divider

(Updated 2024/06/18)

This is a tool for processing pcd files, and it can perform the following functions:

* Dividing point clouds
* Merging point clouds
* Downsampling point clouds
* Generating metadata to efficiently handle the divided point clouds

## Supported Data Format

**Currently, only `pcl::PointXYZI` is supported. Any PCD will be loaded as `pcl::PointXYZI` .**

This tool can be used with files that have data fields other than `XYZI` (e.g., `XYZRGB`) and files that only contain `XYZ`.

* Data fields other than `XYZI` are ignored during loading.
* When loading `XYZ`-only data, the `intensity` field is assigned 0.

## Installation

```bash
$ git clone https://github.com/MapIV/pointcloud_divider.git
$ cd pointcloud_divider
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Usage

  * Select directory, process all files found with `find $INPUT_DIR -name "*.pcd"`.

  ```bash
  $ ./scripts/pointcloud_divider.sh <INPUT_DIR> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  * Select individual files

  ```bash
  $ ./scripts/divider_core.sh <PCD_0> ... <PCD_N> <OUTPUT_DIR> <PREFIX> <CONFIG>
  ```

  | Name            | Description                                  |
  |-----------------|----------------------------------------------|
  | INPUT_DIR       | Directory that contains all PCD files        |
  | PCD_0 ... PCD_N | Input PCD file name                          |
  | OUTPUT_DIR      | Output directory name                        |
  | PREFIX          | Prefix of output PCD file name               |
  | CONFIG          | Config file ([default](config/default.yaml)) |

 `INPUT_DIR`, `PCD_N`, `OUTPUT_DIR` and `CONFIG` can be specified as both **relative paths** and **absolute paths**.

NOTE:  The `OUTPUT_DIR` must already exist. If it does not, an error will occur, and the process will terminate.

## Parameter

* **merge_pcds** [boolean]

    All PCD files are merge into a single PCD. No divided PCD files are generated.

* **leaf_size** [double]

    The leaf_size of voxel grid filter for pointcloud downsampling. The unit is meters [m].
    If the value is less than or equal to 0, downsampling is skipped.

* **grid_size_[xy]** [int]

   The size of the grid for dividing point clouds. The unit is meters [m].
   **NOTE: Even if `merge_pcds` is true, this is used to determine the clusters for downsampling.**
   Therefore, when downsampling without dividing the point cloud, users should not set an excessively large value, such as 100,000. Specifying a large grid size will attempt to load all point clouds into memory and process them at once, which will result in abnormal memory usage.

* **use_large_grid** [boolean]

    Pack output PCD files in larger grid directory.
    When `merge_pcds` is true, this parameter is ignored.
    The large grid is fixed at 10 times the size of grid_size_[xy].
    For example, if the point cloud is divided into 10m x 10m PCD files, a subdirectory like 00100_00100 will contain up to 100 PCD files.

How the point cloud is processed.

![node_diagram](docs/how_to_be_downsampled.drawio.svg)

How the PCD file is named

![node_diagram](docs/output_file_name_pattern.drawio.svg)

### Parameter example

1. Dividing a single point cloud without downsampling

```yaml
  use_large_grid: false
  merge_pcds: false
  leaf_size: -1.0 # any negative number
  grid_size_x: 20
  grid_size_y: 20
```

2. Downsampling and merging divided point clouds into a single file

```yaml
  use_large_grid: false
  merge_pcds: true
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
...
```

## LICENSE

This repository is under [BSD-3-Clauses](LICENSE) license

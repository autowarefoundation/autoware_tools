# Point Cloud Projection Converter

This project includes a tool for converting point cloud data between different geodetic projection systems. The projection systems supported to convert from MGRS to Transverse Mercator.

The conversion details (input and output projection types) are specified in two YAML configuration files.

For example, to convert from MGRS to Transverse Mercator projection, you would use configuration files like this:

```yaml
# input.yaml
projector_type: "MGRS"
mgrs_grid: "54SUE"
```

```yaml
# output.yaml
projector_type: "TransverseMercator"
map_origin:
  latitude: xx
  longitude: yy
```

## Dependencies

- PCL (Point Cloud Library) 1.3 or higher
- yaml-cpp
- GeographicLib
- OpenMP

## Usage

```bash
ros2 run autoware_pointcloud_projection_converter pointcloud_projection_converter path_to_input_pcd_file path_to_output_pcd_file path_to_input_yaml path_to_output_yaml
```

Replace `path_to_pointcloud_file`, `path_to_output_pcd_file`, `path_to_input_yaml`, and `path_to_output_yaml` with the paths to your input YAML configuration file, output YAML configuration file, and PCD file, respectively.

## Special thanks

This package reuses code from [kminoda/projection_converter](https://github.com/kminoda/projection_converter).
We have received permission to use it through [this inquiry](https://github.com/kminoda/projection_converter/issues/3).

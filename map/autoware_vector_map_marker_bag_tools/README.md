# autoware_vector_map_marker_bag_tools

This package provides a tool that launches `tier4_map_component.launch.xml`, captures one `/map/vector_map_marker` message, and inserts that message into a new rosbag based on an existing bag.

## Usage

```bash
ros2 run autoware_vector_map_marker_bag_tools insert_vector_map_marker.py \
  /path/to/input_bag \
  --map-path /path/to/map
```

If `output_bag` is omitted, the tool writes the result to `map_added/` under the input bag directory. If `map_added/` already exists, it is overwritten.

Use `--in-place` to replace the original bag directory directly.

```bash
ros2 run autoware_vector_map_marker_bag_tools insert_vector_map_marker.py \
  /path/to/input_bag \
  --map-path /path/to/map \
  --in-place
```

For tiled maps without `pointcloud_map.pcd`, the tool automatically falls back to `pointcloud_map_file:=.` when `pointcloud_map_metadata.yaml` exists.

You can also specify the output directory explicitly if needed.

```bash
ros2 run autoware_vector_map_marker_bag_tools insert_vector_map_marker.py \
  /path/to/input_bag \
  /path/to/output_bag \
  --map-path /path/to/map \
  --pointcloud-map-file . \
  --lanelet2-map-file lanelet2_map.osm
```

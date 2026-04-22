# autoware_lanelet2_map_merger

This is a tool for processing Lanelet2 (`.osm`) map files. It can perform the following
function:

- Merging multiple divided `.osm` files into a single `.osm` file

It is the inverse of
[`autoware_lanelet2_map_divider`](../autoware_lanelet2_map_divider/README.md).

## Supported Data Format

- Input: a directory containing `.osm` files (typically the `lanelet2_map.osm/`
  directory produced by `autoware_lanelet2_map_divider`).
- Output: a single merged `.osm` file.

The projection required to load/save the maps is obtained from a
`map_projector_info.yaml` file (identical to the one consumed by
`autoware_map_projection_loader`). `MGRS`, `LocalCartesianUTM`, `LocalCartesian`,
and `TransverseMercator` are supported.

## Usage

```bash
ros2 launch autoware_lanelet2_map_merger lanelet2_map_merger.launch.xml \
  input_lanelet2_map_dir:=<INPUT_DIR> \
  output_lanelet2_map:=<OUTPUT_OSM> \
  map_projector_info_path:=<PROJECTOR_YAML>
```

| Name           | Description                                 |
| -------------- | ------------------------------------------- |
| INPUT_DIR      | Directory containing the input `.osm` files |
| OUTPUT_OSM     | Path of the output merged `.osm` file       |
| PROJECTOR_YAML | Path to `map_projector_info.yaml`           |

`INPUT_DIR`, `OUTPUT_OSM`, and `PROJECTOR_YAML` should be specified as **absolute paths**.

## Parameters

{{ json_to_markdown("map/autoware_lanelet2_map_merger/schema/lanelet2_map_merger.schema.json") }}

## How the maps are merged

1. Discover every `.osm` file inside the input directory.
2. Load each map using the projector from `map_projector_info.yaml`.
3. Merge all loaded maps into a single `LaneletMap` using the same logic as
   `lanelet2_map_loader` (`merge_lanelet2_maps`): lanelets, areas, regulatory
   elements, line strings, polygons, and points are all copied, and points shared
   between maps (same ID) are deduplicated so that successor/predecessor
   relationships are preserved.
4. Write the merged map with `lanelet::write` using the same projector.

## Relation to `autoware_lanelet2_map_divider`

Running the merger on the output of the divider reconstructs the original map.
For example:

```bash
# divide
ros2 launch autoware_lanelet2_map_divider lanelet2_map_divider.launch.xml \
  input_lanelet2_map:=/map/lanelet2_map.osm \
  output_lanelet2_map_dir:=/map_divided \
  map_projector_info_path:=/map/map_projector_info.yaml

# merge back
ros2 launch autoware_lanelet2_map_merger lanelet2_map_merger.launch.xml \
  input_lanelet2_map_dir:=/map_divided/lanelet2_map.osm \
  output_lanelet2_map:=/map/lanelet2_map_merged.osm \
  map_projector_info_path:=/map/map_projector_info.yaml
```

## LICENSE

Apache License 2.0.

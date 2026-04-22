# autoware_lanelet2_map_divider

This is a tool for processing Lanelet2 (`.osm`) map files. It can perform the following functions:

- Dividing a Lanelet2 map into grid-aligned segments
- Generating metadata to efficiently handle the divided maps

The produced directory layout and metadata are compatible with the
`lanelet2_map_loader` selected-map-loading feature, as described in the
[`autoware_map_loader` README](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader).

## Supported Data Format

- Input: a single `.osm` Lanelet2 map file. If you have a map that is already split
  across multiple `.osm` files, merge it into one first with
  [`autoware_lanelet2_map_merger`](../autoware_lanelet2_map_merger/README.md).
- Output: one `.osm` file per non-empty grid cell, plus a `lanelet2_map_metadata.yaml`
  file describing the grid.

The projection required to load/save the map is obtained from a
`map_projector_info.yaml` file (identical to the one consumed by
`autoware_map_projection_loader`). `MGRS`, `LocalCartesianUTM`, `LocalCartesian`,
`TransverseMercator`, and `Local` are supported.

## Usage

```bash
ros2 launch autoware_lanelet2_map_divider lanelet2_map_divider.launch.xml \
  input_lanelet2_map:=<INPUT_OSM> \
  output_lanelet2_map_dir:=<OUTPUT_DIR> \
  map_projector_info_path:=<PROJECTOR_YAML> \
  [grid_size_x:=<GRID_SIZE_X>] [grid_size_y:=<GRID_SIZE_Y>] [prefix:=<PREFIX>]
```

| Name                    | Description                                                                                    |
| ----------------------- | ---------------------------------------------------------------------------------------------- |
| INPUT_OSM               | Path to the input `.osm` file                                                                  |
| OUTPUT_DIR              | Output directory. Contains `lanelet2_map.osm/<gx>_<gy>.osm` files and the metadata YAML        |
| PROJECTOR_YAML          | Path to `map_projector_info.yaml` used when loading/saving the Lanelet2 map                    |
| GRID_SIZE_X             | The X size (m) of each output segment. Default 100.0                                           |
| GRID_SIZE_Y             | The Y size (m) of each output segment. Default 100.0                                           |
| PREFIX                  | Optional prefix for output file names (if empty, files are named `<gx>_<gy>.osm`)              |

`INPUT_OSM`, `OUTPUT_DIR`, and `PROJECTOR_YAML` should be specified as **absolute paths**.

NOTE: The folder `OUTPUT_DIR` is auto-generated. If it already exists, all files inside
will be deleted before the tool runs. Back up important files beforehand.

### Parameters

{{ json_to_markdown("map/autoware_lanelet2_map_divider/schema/lanelet2_map_divider.schema.json") }}

### Parameter example

Dividing a map into 100 m x 100 m segments:

```bash
ros2 launch autoware_lanelet2_map_divider lanelet2_map_divider.launch.xml \
  input_lanelet2_map:=/path/to/lanelet2_map.osm \
  output_lanelet2_map_dir:=/path/to/output_dir \
  map_projector_info_path:=/path/to/map_projector_info.yaml
```

## Output directory layout

```
<OUTPUT_DIR>
├── lanelet2_map.osm
│   ├── <gx0>_<gy0>.osm
│   ├── <gx1>_<gy0>.osm
│   └── ...
└── lanelet2_map_metadata.yaml
```

The directory is shaped so that it can be copied next to an existing
`pointcloud_map.pcd/` and `map_projector_info.yaml` to form a map package
that `lanelet2_map_loader` can consume in selected-map-loading mode.

## Grid alignment and file naming

- A point `(x, y)` belongs to grid cell `(gx, gy)` where
  `gx = floor(x / grid_size_x) * grid_size_x` and
  `gy = floor(y / grid_size_y) * grid_size_y`.
- The file name for a cell is `<prefix>_<gx>_<gy>.osm` (or `<gx>_<gy>.osm` if
  `prefix` is empty).
- Grid indices are coordinate-aligned (not bounding-box-relative), so re-running the
  divider on a larger/smaller input that overlaps the same region produces the same
  file names for the overlapping cells. This makes the tool friendly to
  differential-update workflows.

## Metadata YAML Format

The metadata file is `lanelet2_map_metadata.yaml` and follows the same format
consumed by `lanelet2_map_loader`:

```yaml
x_resolution: 100.0
y_resolution: 100.0
0_0.osm: [0.0, 0.0]        # -> 0 <= x < 100, 0 <= y < 100
100_0.osm: [100.0, 0.0]    # -> 100 <= x < 200, 0 <= y < 100
0_100.osm: [0.0, 100.0]
100_100.osm: [100.0, 100.0]
```

- `x_resolution` and `y_resolution` are the grid sizes in meters.
- Each other entry maps a file name to `[min_x, min_y]`, the coordinates of the
  lower-left corner of that cell in the projected frame. The cell covers
  `[min_x, min_x + x_resolution)` x `[min_y, min_y + y_resolution)`.

## How the map is split

1. Load the input `.osm` file using the projector from `map_projector_info.yaml`.
2. Compute the axis-aligned bounding box from all points in the map and
   determine the grid cells covered by the bounding box (coordinate-aligned).
3. For each cell, query `laneletLayer`, `areaLayer`, `lineStringLayer`, and
   `pointLayer` using the cell's 2D bounding box and add the matches to a new
   `LaneletMap`.
4. Write the cell map with `lanelet::write`, using the same projector as the one
   used during loading, so coordinates round-trip through lat/lon correctly.
5. Write `lanelet2_map_metadata.yaml` alongside the cell directory.

Cells with no overlapping primitives are skipped — no empty files are produced.

## Relation to `lanelet2_map_loader`

The generated directory can be fed directly to
[`lanelet2_map_loader`](https://github.com/autowarefoundation/autoware_core/tree/main/map/autoware_map_loader)
with

- `lanelet2_map_path:=<OUTPUT_DIR>/lanelet2_map.osm`
- `metadata_file_path:=<OUTPUT_DIR>/lanelet2_map_metadata.yaml`
- `enable_selected_map_loading:=true`

so that the loader can serve per-cell maps via
`service/get_selected_lanelet2_map`.

## LICENSE

Apache License 2.0.

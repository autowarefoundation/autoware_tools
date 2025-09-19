# Static Centerline Generator

## Purpose

This package statically calculates the centerline satisfying path footprints inside the drivable area.

On narrow-road driving, the default centerline, which is the middle line between lanelets' right and left boundaries, often causes path footprints outside the drivable area.
To make path footprints inside the drivable area, we use online path shape optimization by [the autoware_path_optimizer package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/autoware_path_optimizer/).

Instead of online path shape optimization, we introduce static centerline optimization.
With this static centerline optimization, we have following advantages.

- We can see the optimized centerline shape in advance.
  - With the default autoware, path shape is not determined until the vehicle drives there.
  - This enables offline path shape evaluation.
- We do not have to calculate a heavy and sometimes unstable path optimization since the path footprints are already inside the drivable area.

## Use cases

There are two interfaces to communicate with the centerline optimizer.

### Vector Map Builder Interface

Note: This function of Vector Map Builder has not been released. Please wait for a while.
Currently there is no documentation about Vector Map Builder's operation for this function.

The optimized centerline can be generated from Vector Map Builder's operation.

We can run

- path planning server
- http server to connect path planning server and Vector Map Builder

with the following command by designating `<vehicle_model>`

```sh
ros2 launch autoware_static_centerline_generator run_planning_server.launch.xml vehicle_model:=<vehicle-model>
```

FYI, port ID of the http server is 4010 by default.

### Command Line Interface

The optimized centerline can be generated from the command line interface by designating

- `<input-osm-path>`
- `<output-osm-path>` (not mandatory)
- `<start-lanelet-id>`
- `<start-pose>` (not mandatory)
- `<end-lanelet-id>`
- `<end-pose>` (not mandatory)
- `<vehicle-model>`
- `<goal-method>` (not mandatory, `path_generator` or `behavior_path_planner` only)
- `<lanelet-sequence>` (not mandatory)

```sh
ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml run_backgrond:=false lanelet2_input_file_path:=<input-osm-path> lanelet2_output_file_path:=<output-osm-path> start_lanelet_id:=<start-lane-id> start_pose:=<start-pose> end_lanelet_id:=<end-lane-id> end_pose:=<end-pose> vehicle_model:=<vehicle-model> goal_method:=<goal-method> lanelet_sequence:=<lanelet-sequence>
```

**Note that `<goal-method>:=behavior_path_planner` is not currently supported.**

The default output map path containing the optimized centerline locates `/tmp/autoware_static_centerline_generator/lanelet2_map.osm`.
If you want to change the output map path, you can remap the path by designating `<output-osm-path>`.

By specifying `start-pose`, `goal-pose`, and `goal-method`, the centerline from `start-pose` to `goal-pose` can be embedded.
`<start-pose>`, `<goal-pose>` are entered like `[position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]` with double type.
In order to run smoothly to the goal pose, `goal-method` is used.
Only `path_generator` or `behavior_path_planner` can be entered for `<goal_method>`.
In `<lanelet-sequence>`, you can specify the lanelet_ids for the static centerline to be embedded like `"100,101,102"`.
The input route must be continuous and a drivable path.

> [!WARNING]
> If the start pose is off the center of the lane, it is necessary to manually embed a centerline that smoothly connects the start pose and the start lane in advance using VMB, etc.

## Architecture

![static_centerline_generator_architecture](./media/static_centerline_generator_architecture.drawio.svg)

## Visualization

When performing validation, launch rviz as follows.

![rviz](./media/rviz.png)

- Gray lines indicate map information such as `lanelet` or `stop line`.
- The pink and purple lines show the boundaries of the `lanelet` with embedded fixed paths and the embedded static centerline themselves.
  - The pink and purple breaks indicate where `lanelets` switch.
- The green boxes are safe footprints.

### Unsafe Footprints

Optimized centerline may present the following hazards.
These can be identified using the `unsafe footprints` marker.

- When the footprint approaches the vehicle boundary
- When making abrupt directional changes

Footprint color changes based on distance to the boundary and the angular difference between front and rear points, with distance displayed as text.

![rviz](./media/unsafe_footprints.png)

By default, footprint colors are:

- Distance to lanelet boundary < 0.1 [m]: Red
- Distance to lanelet boundary < 0.2 [m]: Orange
- Distance to lanelet boundary < 0.3 [m]: Yello
- Yaw difference exceeds jitter_deg_threshold (default is 40) [deg]: Blue

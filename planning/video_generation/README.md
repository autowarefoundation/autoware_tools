# ROS 2 bag video generator

`generate_bag_video.py` turns successive ROS 2 MCAP bags into synchronized
30 FPS assets and one composite video. Run it in a ROS 2 Humble environment
with MoviePy 2 and Shapely installed. Run `generate.sh` from your configured zsh session;
it sources the local Autoware workspace, uses `.venv/bin/python` when available,
and supplies the checked-in MPPI example inputs. Run `zsh generate.sh` for that
example. Additional CLI arguments override its defaults.

Pass `--start-before-motion-s 5` to start the video five seconds before the
first ego speed of at least 0.1 m/s. The default motion source is
`/vehicle/status/velocity_status`; use `--motion-topic` and
`--motion-threshold-mps` to select another supported speed source or threshold.
The start is limited to the first bag timestamp, and an error is reported if no
motion is found. Without this option, video generation starts at the beginning
of the first bag. Explicit events that overlap the chosen start are clipped;
events entirely before it are omitted. The plot time axis uses the video start.

```bash
zsh generate.sh \
  --bag-files /path/to/first.mcap /path/to/second.mcap \
  --rviz-config /path/to/view.rviz \
  --vehicle-model j6_gen2 \
  --camera-config /path/to/three_cameras.yaml \
  --birdseye \
  --trajectory-topic /planning/trajectory \
  --mppi-trajectory-topic /planning/trajectory_generator/trajectory_modifier/debug/mppi/optimized_trajectory \
  --plot-config /path/to/plot.yaml \
  --intermediate-dir /path/to/intermediate \
  --output-dir /path/to/output
```

Use `plot_config.example.yaml` as the explicit-event template or `mppi_plot.yaml`
as the Bool-topic clearance example. A scalar `metrics` entry names a
`std_msgs/msg/Float32` or `autoware_internal_debug_msgs/msg/Float64Stamped`
topic. Optional `label` and `color` fields set its plot appearance. Explicit
`events` contain absolute bag timestamps as integer nanoseconds; each includes
`start_ns`, excludes `end_ns`, and plays at its `speed` (default
0.5). Events must not overlap and must lie within the global bag timeline.

Alternatively, `event_detection` names a `std_msgs/msg/Bool` topic. A true
value starts an event and the next false value ends it. `pre_margin_s` and
`post_margin_s` expand each interval, clamp it to the bag timeline, and merge
overlapping or touching intervals. An event still active at the end of the bag
ends at the bag end. The event `speed` defaults to 0.5. Outside events,
playback uses `default_speed` (default 2.0). The plot stays visible throughout;
its lines are empty when no metric samples are available. The current replay
speed is shown in the upper-left of the video. Use either
`events` or `event_detection` in one YAML file.
Set `plot_window_s` to show only the most recent number of seconds in the plot;
omit it to show the complete bag timeline.

The `clearance` plot mode in `mppi_plot.yaml` produces two panels: road-border
clearance and parked-vehicle clearance. Each compares MPPI's optimized
trajectory with its reference trajectory (labelled ML planner). The minimum is
taken over every published trajectory pose using the 2D j6_gen2 footprint from
the installed vehicle description. Road-border centerlines are recovered from
the bag's `/map/vector_map_marker` `road_borders` triangles. Parked avoidance
targets come from the planner's per-object selector decisions in `/diagnostics`;
their shapes come from the matching tracked-object snapshot. An overlap is
plotted as zero metres. Missing or stale inputs, and cycles with no selected
parked target, leave a gap in that series. Clearance mode requires
`--vehicle-model` and supports the bag's `autoware_planning_msgs/msg/Trajectory`
messages.

`--camera-config` selects the cameras for the sidebar grid. Its `cameras` list is
ordered row by row; each entry requires a unique `name`, a visible `label`, an
`image_topic`, and an `info_topic`. The default `mppi_bev_cameras.yaml` selects
Cameras 1, 2, and 3 for the first three cells of a 2×2 grid; `--birdseye` puts
the bird's-eye view in the fourth cell. `mppi_cameras.yaml` keeps the earlier
four-camera selection and `mppi_all_cameras.yaml` keeps the 11-camera selection
for comparison. Each feed is rendered at
800 pixels wide after trajectory projection and written to
`camera_<name>_1x.mp4`. The grid fills the lower part of the dedicated right
column, preserving each camera's aspect ratio; a 960 × 450 plot fills the upper
part. Eight-pixel gaps separate grid cells. The current example writes to
`output/mppi_bev_example/final_video.mp4`; prior results stay in their separate
output directories.

The bird's-eye view uses `/localization/kinematic_state` odometry by default
(`--birdseye-ego-topic` can select another odometry topic). It shows map road
borders, the current j6_gen2 ego footprint, and MPPI and ML planner trajectories
with footprint outlines at every pose. The ML planner line uses the live
`ml_planner_node/output/trajectory` topic (`--birdseye-ml-topic` can override it),
so it remains visible outside MPPI events. MPPI disappears one second after its
last update. Tracked cars, trucks, buses, and trailers with reported speed below
0.5 m/s appear as parked polygons while their object snapshot is fresh. The
view stays centered on the ego footprint's geometric center, with forward
pointing up and a rectangular view extending 4 m ahead and behind
(`--birdseye-range-m` can change it). Trajectory footprint outlines use 0.40
alpha and 2-pixel lines. The grid spacing is 1 m at this scale. The full tile
width is used, so the lateral range follows its aspect ratio. Its synchronized asset is
`birdseye_1x.mp4`.
The original single-camera CLI remains available
with `--camera-topic` and `--camera-info-topic`, producing `camera_1x.mp4`.

The first image on each camera topic sets its asset resolution. Frames before it
are black; subsequent gaps hold the last image. Bags must be in time order and
must not overlap. The script also writes `plot_1x.mp4` and `rviz_1x.mp4` to the
intermediate directory and `final_video.mp4` to the output directory.
With trajectory options, the camera asset includes a transparent cyan ribbon
for `--trajectory-topic` throughout the recording. During configured events,
it also includes a narrower magenta ribbon for `--mppi-trajectory-topic` when
that trajectory has updated within one second. Widths default to 2.0 and 1.0
meters (`--ribbon-width-m`, `--mppi-ribbon-width-m`), and alpha defaults to 0.4
(`--ribbon-alpha`). Both `nav_msgs/msg/Path` and
`autoware_planning_msgs/msg/Trajectory` are supported. The overlay uses the
bag's `/tf`, `/tf_static`, and CameraInfo directly. It supports calibrated
`plumb_bob` and `rational_polynomial` cameras with raw or compressed images.
Each rendered camera frame is held through frame gaps, preserving the 30 FPS timeline.
RViz is captured under Xvfb and aligned to the bag timeline using `/clock`;
rendering and screen capture may differ by about one 30 FPS frame.
The RViz capture hides the X11 mouse pointer. It starts `xfwm4` in a temporary
D-Bus session, sends F11 to RViz, and verifies that the window fills the
1440 × 1080 virtual screen before recording.
The capture uses a 1440 × 1080 virtual screen. The final video is 2400 × 1080,
with RViz on the left and a separate 960-pixel column for cameras and plot.
Install `xdotool`, `xfwm4`, and `dbus-run-session` alongside `Xvfb` and
`ffmpeg`. The included `mppi.rviz` has a 1400 × 1000 normal window geometry;
F11 expands it for capture.
With `--vehicle-model j6_gen2`, the script expands the installed
`j6_gen2_description/urdf/vehicle.xacro` and runs `robot_state_publisher`
during RViz capture. This supplies `/robot_description` even when the bag does
not contain it. The example RViz config requests transient-local durability
for that topic, and the bag's `/tf` supplies the vehicle pose.
RViz runs with a temporary copy of the config so recording does not save
bag-specific display state over the source file.

Before any ROS node starts, the script checks that the inherited environment
selects `rmw_cyclonedds_cpp` and a local `CYCLONEDDS_URI` XML file whose only
configured interface is `lo`. RViz and `ros2 bag play` receive that same
environment; the in-process `/clock` subscriber uses it as well. The wrapper
does not load `~/.zshrc` itself, so launch it from the zsh session where those
variables are already exported.

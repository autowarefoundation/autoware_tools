# ROS 2 bag video generator

`generate_bag_video.py` turns successive ROS 2 MCAP bags into synchronized
30 FPS assets and one composite video. Run it in a ROS 2 Humble environment
with MoviePy 2, Shapely, and PyAV installed (`.venv/bin/python -m pip install av`
for the existing virtual environment). Run `generate.sh` from your configured zsh session;
it sources the local Autoware workspace, uses `.venv/bin/python` when available,
and targets the WebAuto run range `2026-11-12T04:00:00Z` through
`2026-11-12T06:00:00Z` for project `x2_dev` and vehicle
`f4dbd93f-37d7-4948-b92e-a47d65e0bb7b`. Run `zsh generate.sh` for that
range. The wrapper stores intermediate files and temporary RViz captures on
`/media/maximeclement/PortableSSD1/MPPI/video_generation`; final videos remain
under the local `output/mppi_bev_example` directory. Additional CLI arguments
override its defaults. Passing `--bag-files` selects local mode instead.

To discover and download complete runs from WebAuto, supply a project, vehicle,
and timezone-aware ISO 8601 range instead of `--bag-files`:

```bash
zsh generate.sh \
  --project-id x2_dev \
  --vehicle-id YOUR_VEHICLE_ID \
  --datetime-from '2026-11-05T10:00:00+09:00' \
  --datetime-to '2026-11-05T12:00:00+09:00'
```

The range includes runs whose `_0.mcap` starts at or after `--datetime-from`
and whose final source bag ends at or before `--datetime-to`. WebAuto's
paginated log-file search uses both datetime flags and extends its search end
to the current time when necessary to find the next `_0.mcap` for the same
recording stream. That next start confirms the previous
run's last numbered bag. A trailing run without a later `_0` is deferred;
runs with missing or duplicate numbered parts are skipped. The script downloads
each selected source bag separately through `webauto data log-file
pull-filtered-rosbag`, requesting the topics used by RViz, cameras, plots,
events, motion detection, TF, and trajectory overlays. The pulls keep each
source bag's full time span. WebAuto source timestamps set the video timeline
even if the last requested topic stops publishing early.

Remote mode writes `intermediate/<run-name>/downloads/` and the 1× assets,
then `output/<run-name>/final_video.mp4` and `run_manifest.json`, relative to
the supplied directory roots. Successful filtered downloads are reused when
their topic set is unchanged. A failed run is reported and the remaining runs
are still attempted. The local `--bag-files` mode retains its original output
paths. Both modes need the configured ROS environment for rendering, and remote
mode also needs the `webauto` CLI in `PATH` with access to the project.
When `event_detection` is configured, remote mode checks each complete run's
downloaded Bool topic before rendering. Runs with no `true` sample are logged
as skipped, create no video or manifest, and do not count as failures. Local
`--bag-files` mode continues to render the bags supplied explicitly.

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

The final video adds a 120-pixel timeline strip below the 2400×1080 scene,
making the output 2400×1200. It shows the recording time in JST, elapsed time,
Autoware control state, every raw MPPI activation, and a moving playhead. The
strip uses the visible video interval, including any `--start-before-motion-s`
trim. MPPI markers exclude event margins and include activations that did not
qualify for a slowdown. State is unknown before its first message and across
recording gaps longer than one second; adjacent bag parts carry the last state.
When `event_detection` is absent, the MPPI row has no
activation markers. The corresponding 1× asset is `timeline_1x.mp4` in the
intermediate directory.

For MPPI recordings, `event_detection.slowdown_filter` can require ego motion
under Autoware control before slowing a full activation (including its time
margins). It reads `/api/operation_mode/state` and
`/vehicle/status/velocity_status` by default. At least one velocity sample in
the original true interval must meet `min_speed_mps` (default 0.1) while the
latest operation mode state has `mode == AUTONOMOUS` and
`is_autoware_control_enabled == true`. Missing state or motion samples do not
qualify an activation. A nonqualifying activation still appears in MPPI camera
and clearance visuals, while the video continues at `default_speed`.

```yaml
event_detection:
  topic: /planning/trajectory_generator/trajectory_modifier/debug/mppi/enabled
  pre_margin_s: 0.5
  post_margin_s: 0.5
  speed: 0.5
  slowdown_filter:
    operation_mode_topic: /api/operation_mode/state
    motion_topic: /vehicle/status/velocity_status
    min_speed_mps: 0.1
```

Set `plot_window_s` to show only the most recent number of seconds in the plot;
omit it to show the complete bag timeline.

The `clearance` plot mode in `mppi_plot.yaml` produces two panels: road-border
clearance and parked-vehicle clearance. During MPPI events, each compares MPPI's
optimized trajectory with its reference trajectory (labelled ML planner). The
MPPI and reference curves remain empty when those topics are absent from a run;
the camera and bird's-eye views still show the live ML trajectory in local mode.
Remote mode skips runs without MPPI activation. The
road-border panel shows the live ML planner trajectory whenever its clearance
is below `ml_road_border_threshold_m` (default 1.5 m), including event margins
before or after reference data is available. During reference coverage, its full
curve is shown instead. Configure the continuous source with `live_ml_topic`;
live values at or above the threshold are left blank. The minimum is
taken over every published trajectory pose using the 2D j6_gen2 footprint from
the installed vehicle description. Road-border centerlines are recovered from
the bag's `/map/vector_map_marker` `road_borders` triangles. Parked avoidance
targets come from the planner's per-object selector decisions in `/diagnostics`;
their shapes come from the matching tracked-object snapshot. An overlap is
plotted as zero metres. Missing or stale inputs, and cycles with no selected
parked target, leave a gap in that series. Clearance mode requires
`--vehicle-model` and supports the bag's `autoware_planning_msgs/msg/Trajectory`
messages.

`--camera-config` selects the cameras for the sidebar. Each entry requires a
unique `name`, a `label`, an `image_topic`, and an `info_topic`. Labels are drawn
only when multiple cameras are shown; a single camera has no text overlay. The
default `mppi_bev_cameras.yaml` selects Camera 8, the front high FOV camera.
With `--birdseye`, the upper right area has a 476 × 450 plot and a 476 × 450
bird's-eye view side by side; the camera fills the lower right area while
preserving its aspect ratio. Other multi-camera configurations retain the grid
layout: `mppi_cameras.yaml` selects four cameras and `mppi_all_cameras.yaml`
selects 11. Each feed is rendered at 800 pixels wide after trajectory
projection and written to `camera_<name>_1x.mp4`. Eight-pixel gaps separate
sidebar panels. The current example writes to
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
view stays centered on the midpoint of the ego footprint's front edge, with
forward pointing up. It zooms from a minimum 4 m fore/aft half-range to fit the
ego footprint and current trajectory paths and outlines, up to 10 m from the
front edge. Set the minimum with `--birdseye-range-m` and the cap with
`--birdseye-max-range-m`; geometry beyond the cap is clipped by the view.
Trajectory footprint outlines use 0.40 alpha and 2-pixel lines. The grid spacing
is 1 m at the closest scale. The full tile width is used, so the lateral range
follows its aspect ratio. Its synchronized asset is `birdseye_1x.mp4`.
The original single-camera CLI remains available
with `--camera-topic` and `--camera-info-topic`, producing `camera_1x.mp4`.

The first image on each camera topic sets its asset resolution. Frames before it
are black; subsequent gaps hold the last image. Bags must be in time order and
must not overlap. The script also writes `plot_1x.mp4` and `rviz_1x.mp4` to the
intermediate directory and `final_video.mp4` to the output directory.
With trajectory options, the camera asset includes a transparent cyan ribbon
for `--trajectory-topic` throughout the recording. During configured events,
it also includes a narrower gentle-green ribbon for `--mppi-trajectory-topic` when
that trajectory has updated within one second. Widths default to 2.0 and 1.0
meters (`--ribbon-width-m`, `--mppi-ribbon-width-m`), and alpha defaults to 0.4
(`--ribbon-alpha`). Both `nav_msgs/msg/Path` and
`autoware_planning_msgs/msg/Trajectory` are supported. The overlay uses the
bag's `/tf`, `/tf_static`, and CameraInfo directly. It supports calibrated
`plumb_bob` and `rational_polynomial` cameras with raw images, JPEG compressed
images, and `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` video packets.
The packet decoder supports AV1, H.264, and HEVC, preserves packet timestamps
despite decoder delay, and holds the previous frame through missing packets.
Each rendered camera frame is held through frame gaps, preserving the 30 FPS timeline.
When filtered bags omit camera static TF, `--sensor-calibration-dir` supplies
the vehicle's `sensors_calibration.yaml` and `sensor_kit_calibration.yaml`.
The script reconstructs the fixed `base_link` to sensor kit to camera optical
frame chain, filling only transforms absent from the bag. The default
`generate.sh` points to this vehicle's `j6_gen2_34` calibration; choose the
matching vehicle directory for other recordings. An overlay cannot appear
when a short trajectory remains entirely behind the camera.
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
Replay uses `rviz_playback_qos.yaml` to offer transient-local durability on
`/tf_static` and `/map/vector_map_marker`, whose filtered MCAP metadata does
not retain the original offered QoS profiles.
RViz runs with a temporary copy of the config so recording does not save
bag-specific display state over the source file.

Before any ROS node starts, the script checks that the inherited environment
selects `rmw_cyclonedds_cpp` and a local `CYCLONEDDS_URI` XML file whose only
configured interface is `lo`. RViz and `ros2 bag play` receive that same
environment; the in-process `/clock` subscriber uses it as well. The wrapper
does not load `~/.zshrc` itself, so launch it from the zsh session where those
variables are already exported.

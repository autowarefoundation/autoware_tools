# ROS 2 bag video generator

`generate_bag_video.py` turns successive ROS 2 MCAP bags into three synchronized
30 FPS assets and one composite video. Run it in a ROS 2 Humble environment
with MoviePy 2 installed. Run `generate.sh` from your configured zsh session;
it sources the local Autoware workspace, uses `.venv/bin/python` when available,
and supplies the checked-in MPPI example inputs. Run `zsh generate.sh` for that
example. Additional CLI arguments override its defaults.

```bash
zsh generate.sh \
  --bag-files /path/to/first.mcap /path/to/second.mcap \
  --rviz-config /path/to/view.rviz \
  --camera-topic /camera/image/compressed \
  --plot-config /path/to/plot.yaml \
  --intermediate-dir /path/to/intermediate \
  --output-dir /path/to/output
```

Use `plot_config.example.yaml` as the explicit-event template or `mppi_plot.yaml`
as the Bool-topic event template. Each `metrics` entry names a
`std_msgs/msg/Float32` or `autoware_internal_debug_msgs/msg/Float64Stamped`
topic. Optional `label` and `color` fields set its plot appearance. Explicit
`events` contain absolute bag timestamps as integer nanoseconds; each includes
`start_ns`, excludes `end_ns`, shows the plot, and plays at its `speed` (default
0.5). Events must not overlap and must lie within the global bag timeline.

Alternatively, `event_detection` names a `std_msgs/msg/Bool` topic. A true
value starts an event and the next false value ends it. `pre_margin_s` and
`post_margin_s` expand each interval, clamp it to the bag timeline, and merge
overlapping or touching intervals. An event still active at the end of the bag
ends at the bag end. The event `speed` defaults to 0.5. Outside events, the
plot is hidden and playback uses `default_speed` (default 2.0). Use either
`events` or `event_detection` in one YAML file.
Set `plot_window_s` to show only the most recent number of seconds in the plot;
omit it to show the complete bag timeline.

The first camera image sets the output resolution. Frames before it are black;
subsequent gaps hold the last image. Bags must be in time order and must not
overlap. The script writes `camera_1x.mp4`, `plot_1x.mp4`, and `rviz_1x.mp4`
to the intermediate directory and `final_video.mp4` to the output directory.
RViz is captured under Xvfb and aligned to the bag timeline using `/clock`;
rendering and screen capture may differ by about one 30 FPS frame.
The capture uses a 1920 × 1080 virtual screen. Install `xdotool` alongside
`Xvfb` and `ffmpeg`; the script applies the RViz config's `Window Geometry`
before recording and stops if the window does not fit the screen. The included
`mppi.rviz` is sized to 1880 × 1000 at position (20, 20).

Before any ROS node starts, the script checks that the inherited environment
selects `rmw_cyclonedds_cpp` and a local `CYCLONEDDS_URI` XML file whose only
configured interface is `lo`. RViz and `ros2 bag play` receive that same
environment; the in-process `/clock` subscriber uses it as well. The wrapper
does not load `~/.zshrc` itself, so launch it from the zsh session where those
variables are already exported.

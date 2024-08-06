# Autoware Debug Tools

This package provides tools for debugging Autoware.

## Processing Time Visualizer

This tool visualizes `tier4_debug_msgs/msg/ProcessingTimeTree` messages.

### Usage

1. Run the following command to start the visualizer.

   ```bash
   ros2 run autoware_debug_tools processing_time_visualizer
   ```

2. Select a topic to visualize.

   ![select_topic](images/select-topic.png)

3. Then, the visualizer will show the processing time tree.

   ![visualize-tree](images/visualize-tree.png)

## System Usage Monitor

The purpose of the System Usage Monitor is to monitor, visualize and publish the CPU usage and memory usage of the ROS processes. By providing a real-time terminal-based visualization, users can easily confirm the cpu and memory usage as in the picture below.

![system_usage_monitor](images/system_usage_monitor.png)

You can run the program by the following command.

```bash
ros2 run autoware_debug_tools system_usage_monitor
```

## System Performance Plotter

This script plots the following metrics by each Autoware's module.

- processing time
- CPU usage
- memory usage

### Usage

Run the following commands according to your purpose.

```bash
# plot processing time
ros2 run autoware_debug_tools processing_time_plotter <bag-path>

# plot CPU usage
ros2 run autoware_debug_tools cpu_usage_plotter <bag-path>

# plot memory usage
ros2 run autoware_debug_tools memory_usage_plotter <bag-path>
```

There are several options.

- `-c`:
  - can designate modules in the specific component (e.g. `all`, `planning`, `system`, etc).
- `-n N:
  - can pick up top N critical modules.
- `-g <string>`
  - can filter the modules which include `<string>`.
- `-y <float>`
  - can set the height of the plot.

### Examples

```bash
ros2 run autoware_debug_tools processing_time_plotter <bag-path> -c planning -g behavior_path -y 300
```

![processing_time_plot_example](images/processing_time_plot_example.png)

```bash
ros2 run autoware_debug_tools cpu_usage_plotter <bag-path> -n 20
```

![cpu_usage_plot_example](images/cpu_usage_plot_example.png)

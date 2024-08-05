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

#### summarized output

Running with `--summarize`, it will output the summarized information.

```Text
> ros2 run autoware_debug_tools processing_time_visualizer --summarize

objectsCallback: 17.99 [ms], run count: 1
    ├── removeStaleTrafficLightInfo: 0.00 [ms], run count: 1
    ├── updateObjectData: 0.03 [ms], run count: 13
    ├── getCurrentLanelets: 4.81 [ms], run count: 13
    │   ├── checkCloseLaneletCondition: 2.43 [ms], run count: 130
    │   ├── isDuplicated: 0.02 [ms], run count: 17
    │   └── calculateLocalLikelihood: 0.66 [ms], run count: 12
    ├── updateRoadUsersHistory: 0.30 [ms], run count: 13
    └── getPredictedReferencePath: 5.47 [ms], run count: 5
        ├── predictObjectManeuver: 0.40 [ms], run count: 5
        │   └── predictObjectManeuverByLatDiffDistance: 0.34 [ms], run count: 5
        │       └── calcRightLateralOffset: 0.03 [ms], run count: 12
        ├── calculateManeuverProbability: 0.01 [ms], run count: 5
        └── addReferencePaths: 4.66 [ms], run count: 15
            ├── updateFuturePossibleLanelets: 0.08 [ms], run count: 8
            └── convertPathType: 4.29 [ms], run count: 8

```

## System Usage Monitor

The purpose of the System Usage Monitor is to monitor, visualize and publish the CPU usage and memory usage of the ROS processes. By providing a real-time terminal-based visualization, users can easily confirm the cpu and memory usage as in the picture below.

![system_usage_monitor](image/system_usage_monitor.png)

You can run the program by the following command.

```bash
ros2 run autoware_debug_tools system_usage_monitor
```

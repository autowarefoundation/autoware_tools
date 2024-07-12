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

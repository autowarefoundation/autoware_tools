# Trajectory Visualizer

This tool allow the visualization of Autoware trajectory messages (`autoware_planning_msgs::msg::Trajectory`).

## How to use `visualizer.py`

0. **Source an Autoware workspace:** the workspace needs to contain the `autoware_planning_msgs` package.
1. **Run the script:**

   ```bash
   python visualizer.py
   ```

2. **A window will pop up:** This window will be used to visualize trajectories.

3. **Choose what to plot:**
   - **X-axis and Y-axis:** Use the dropdown menus at the top left to select what data you want on the X and Y axes of the graph (e.g., "Time" vs. "Velocity").
   - **Topics:** The list in the middle left shows available trajectory topics. Check the boxes to select which trajectories you want to see on the plot.
   - **Refresh List:** Press the "Refresh List" button to update the list of available topics. This is useful if new trajectory data becomes available.

4. **See the plot:** The right side of the window shows the plot of the selected trajectory data.

## Code structure

- `visualizer.py`: This is the main script that starts the visualizer. It sets up the ROS 2 connection and the graphical interface.
- `gui.py`: This file creates the graphical user interface (GUI) using Tkinter. It handles the buttons, dropdown menus, lists, and the plot area.
- `plotter.py`: This file uses Matplotlib to create the plots. It draws the trajectory data on the graph.
- `ros2_interface.py`: This file handles the communication with ROS 2. It gets the trajectory data from ROS 2 topics.
- `trajectory_data.py`: This file calculates different properties of the trajectory, like velocity, curvature, etc. It provides the data that can be plotted.

## How to add new properties to plot

1. Open `trajectory_data.py`.
2. Write a new function taking a `Trajectory` object as input and returning a list of values (one for each point in the trajectory).
3. Add your function to the dictionary returned by `get_data_functions()`. The key should be a string that describes the property (this is what will show up in the dropdown menu in the GUI) and the value should be the function you created.

#!/usr/bin/env python3

# Copyright 2025 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# cspell:disable
import tkinter as tk
from tkinter import ttk

from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from plotter import Plotter
from ros2_interface import ROS2Interface
from trajectory_data import get_data_functions
from trajectory_node_graph import TrajectoryNodeGraph


class TkinterApp:
    def __init__(self, root, ros_interface_node: ROS2Interface, config):
        self.root = root
        self.root.title("Autoware Trajectory Visualizer")
        self.root.geometry("800x600")
        self.ros_interface = ros_interface_node
        self.topics = config["initial_topics"]

        # Predetermined list for the dropdown
        self.axis_options = get_data_functions()
        self.current_x_axis_selection = tk.StringVar()
        self.current_y_axis_selection = tk.StringVar()

        # --- Main Frames ---
        self.left_frame = ttk.Frame(self.root, padding="10")
        self.left_frame.grid(row=0, column=0, sticky="nswe", padx=0, pady=0)
        self.mid_frame = ttk.Frame(self.root, padding="0", width=1)
        self.mid_frame.grid_propagate(False)
        self.mid_frame.grid(row=0, column=1, sticky="nswe", padx=0, pady=0)
        self.right_frame = ttk.Frame(self.root, padding="10")
        self.right_frame.grid(row=0, column=2, sticky="nswe", padx=0, pady=0)

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=0)  # Never resize the middle frame
        self.root.grid_columnconfigure(2, weight=3)  # Give more weight to the plot area
        self.root.grid_rowconfigure(0, weight=1)

        # --- Left Frame Widgets ---
        axis_option_keys = [str(key) for key in self.axis_options.keys()]

        # Dropdown List (Combobox)
        ttk.Label(self.left_frame, text="X-axis:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.x_axis_dropdown = ttk.Combobox(
            self.left_frame,
            textvariable=self.current_x_axis_selection,
            values=axis_option_keys,
            state="readonly",
        )
        self.x_axis_dropdown.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.x_axis_dropdown.current(0)  # Set default selection
        # Dropdown List (Combobox)
        ttk.Label(self.left_frame, text="Y-axis:").grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.y_axis_dropdown = ttk.Combobox(
            self.left_frame,
            textvariable=self.current_y_axis_selection,
            values=axis_option_keys,
            state="readonly",
        )
        self.y_axis_dropdown.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self.y_axis_dropdown.current(1)  # Set default selection

        # Set initial selection from config
        for i in range(len(axis_option_keys)):
            if axis_option_keys[i].casefold().startswith(config["initial_axis"]["x"].casefold()):
                self.x_axis_dropdown.current(i)
            if axis_option_keys[i].casefold().startswith(config["initial_axis"]["y"].casefold()):
                self.y_axis_dropdown.current(i)

        # bind dropdown selection events to update axis labels
        self.x_axis_dropdown.bind("<<ComboboxSelected>>", self.update_axis_labels)
        self.y_axis_dropdown.bind("<<ComboboxSelected>>", self.update_axis_labels)

        # Listbox with Multiple Selection
        ttk.Label(self.left_frame, text="Topics:").grid(
            row=2, column=0, columnspan=2, padx=5, pady=5, sticky="w"
        )
        self.listbox_frame = ttk.Frame(self.left_frame)
        self.listbox_frame.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

        self.listbox_scrollbar_y = ttk.Scrollbar(self.listbox_frame, orient=tk.VERTICAL)
        self.listbox_scrollbar_x = ttk.Scrollbar(self.listbox_frame, orient=tk.HORIZONTAL)
        self.listbox = tk.Listbox(
            self.listbox_frame,
            selectmode=tk.MULTIPLE,
            yscrollcommand=self.listbox_scrollbar_y.set,
            xscrollcommand=self.listbox_scrollbar_x.set,
            exportselection=False,
        )
        self.listbox.bind("<<ListboxSelect>>", self.on_listbox_select)
        self.listbox_scrollbar_y.config(command=self.listbox.yview)
        self.listbox_scrollbar_x.config(command=self.listbox.xview)

        self.listbox_scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Allow listbox to expand
        self.left_frame.grid_rowconfigure(3, weight=1)
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.left_frame.grid_columnconfigure(1, weight=1)

        # Button to Refresh List
        self.refresh_button = ttk.Button(
            self.left_frame, text="Refresh List", command=self.refresh_topic_list
        )
        self.refresh_button.grid(row=4, column=0, columnspan=2, padx=5, pady=10, sticky="ew")
        # --- Mid Frame --- Button to hide/show the left frame
        self.hide_button = ttk.Button(
            self.mid_frame, text="<", command=self.hide_show_left_frame, width=1
        )
        self.hide_button.pack(fill="both", expand=True)
        self.left_hidden = False
        # --- Right Frame Widgets ---
        # Matplotlib Plot Area
        self.plotter = Plotter()
        self.canvas = FigureCanvasTkAgg(self.plotter.fig, master=self.right_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.msg_per_topic = {}
        self.plot(list(range(len(self.topics))))  # plot all initial topics
        self.refresh_topic_list()
        # Start with no topics selected - initialize empty plot
        self.plotter.init_plot(
            self.current_x_axis_selection.get(), self.current_y_axis_selection.get(), []
        )

    def on_listbox_select(self, event):  # noqa: ARG002
        """Handle listbox selection event, preventing node separator selection."""
        current_selection = list(self.listbox.curselection())
        modified = False

        # Check each selected index
        for idx in current_selection[:]:  # Use slice to create a copy
            item_text = self.listbox.get(idx)
            # If it's a node separator, deselect it
            if item_text.startswith("---"):
                self.listbox.selection_clear(idx)
                current_selection.remove(idx)
                modified = True

        # Only trigger plot if selection contains actual topics
        if current_selection:
            self.plot(current_selection)
        elif modified:
            # If all selections were node separators, don't plot
            pass

    def hide_show_left_frame(self):
        if self.left_hidden:
            self.left_frame.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)
            self.hide_button.config(text="<")
            self.root.grid_columnconfigure(0, weight=1)
        else:
            self.left_frame.grid_forget()
            self.hide_button.config(text=">")
            self.root.grid_columnconfigure(0, weight=0)
        self.left_hidden = not self.left_hidden

    def refresh_topic_list(self):
        self.listbox.delete(0, tk.END)  # Clear existing items

        # Use TrajectoryNodeGraph for better organization
        try:
            # First, get currently available topics
            current_topics = self.ros_interface.get_trajectory_topics()

            # Try to analyze using TrajectoryNodeGraph
            analyzer = TrajectoryNodeGraph(node=self.ros_interface)
            results = None

            try:
                # Try live analysis first
                results = analyzer.analyze()

                # Check if we got valid results with nodes
                if not results or not results.get("graph") or not results.get("nodes_in_order"):
                    raise RuntimeError("Live analysis returned empty graph")

            except Exception as live_error:
                # If live analysis fails (e.g., bag playback), try loading from YAML
                print(f"Live analysis failed ({live_error}), trying YAML fallback...")

                yaml_data = TrajectoryNodeGraph.load_from_yaml()
                if yaml_data:
                    # Filter YAML data with currently active topics
                    results = analyzer.filter_graph_with_active_topics(yaml_data, current_topics)
                    if results:
                        print("Using cached graph configuration from YAML")
                else:
                    print("No YAML configuration found")

            # Get organized topics only if we have results
            if not results or not results.get("topics_by_node"):
                raise RuntimeError("No valid results to display")

            # Get organized topics
            topics_by_node = results.get("topics_by_node", {})

            # Build topics list with node separators
            self.topics = []
            displayed_topics = set()  # Track topics already displayed

            # First, display topics from YAML nodes
            for node_name, node_topics in topics_by_node.items():
                if node_topics:
                    # Add node separator
                    node_line = f"--- [{node_name}] ---"
                    self.listbox.insert(tk.END, node_line)
                    self.listbox.itemconfig(tk.END, {"fg": "gray"})

                    for topic_info in node_topics:
                        # topic_info is ('main'/'other', topic_name, msg_type)
                        priority, topic_name, msg_type = topic_info

                        # Store in original format
                        self.topics.append((topic_name, msg_type))
                        displayed_topics.add(topic_name)

                        # Display in original format
                        display_text = f"{topic_name} [{msg_type.split('/')[-1]}]"
                        self.listbox.insert(tk.END, display_text)

                        # Highlight main flow topics
                        if priority == "main":
                            self.listbox.itemconfig(tk.END, {"fg": "blue"})

            # Add remaining topics not in YAML under "others" section
            other_topics = []
            for topic_name, msg_type in current_topics:
                # Only add if not already displayed and under /planning or /control
                if topic_name not in displayed_topics and (
                    topic_name.startswith("/planning") or topic_name.startswith("/control")
                ):
                    other_topics.append((topic_name, msg_type))

            # Sort other topics alphabetically
            other_topics.sort(key=lambda x: x[0])

            # Add "others" section if there are any
            if other_topics:
                # Add separator
                node_line = "--- [others] ---"
                self.listbox.insert(tk.END, node_line)
                self.listbox.itemconfig(tk.END, {"fg": "gray"})

                for topic_name, msg_type in other_topics:
                    # Store in topics list
                    self.topics.append((topic_name, msg_type))

                    # Display in original format
                    display_text = f"{topic_name} [{msg_type.split('/')[-1]}]"
                    self.listbox.insert(tk.END, display_text)

            return
        except Exception as e:
            # Fall back to original method if analysis fails
            print(f"TrajectoryNodeGraph analysis failed: {e}")

        # Original implementation (fallback)
        self.topics = self.ros_interface.get_trajectory_topics()
        for topic, msg_type in self.topics:
            self.listbox.insert(tk.END, f"{topic} [{msg_type.split('/')[-1]}]")

    def update(self, topic, msg):
        self.msg_per_topic[topic] = msg

    def plot(self, topic_indexes):
        self.ros_interface.remove_callbacks()
        self.msg_per_topic.clear()
        x_axis_selection = self.current_x_axis_selection.get()
        y_axis_selection = self.current_y_axis_selection.get()

        # Map listbox indexes to actual topic indexes
        selected_topics = []
        topic_idx = 0
        listbox_idx = 0

        for i in range(self.listbox.size()):
            item_text = self.listbox.get(i)
            if item_text.startswith("---"):
                # Skip node separators
                listbox_idx += 1
                continue

            if listbox_idx in topic_indexes:
                if topic_idx < len(self.topics):
                    selected_topics.append(self.topics[topic_idx])

            topic_idx += 1
            listbox_idx += 1

        self.plotter.init_plot(x_axis_selection, y_axis_selection, [t[0] for t in selected_topics])
        for topic, msg_type in selected_topics:
            if msg_type.endswith("Trajectory"):
                self.ros_interface.add_callback(
                    topic,
                    Trajectory,
                    lambda msg, captured_topic=topic: self.update(captured_topic, msg),
                )
            elif msg_type.endswith("PathWithLaneId"):
                self.ros_interface.add_callback(
                    topic,
                    PathWithLaneId,
                    lambda msg, captured_topic=topic: self.update(captured_topic, msg),
                )
            elif msg_type.endswith("Path"):
                self.ros_interface.add_callback(
                    topic, Path, lambda msg, captured_topic=topic: self.update(captured_topic, msg)
                )

    def replot(self):
        x_axis_selection = self.current_x_axis_selection.get()
        y_axis_selection = self.current_y_axis_selection.get()
        x_axis_fns = self.axis_options[x_axis_selection]
        y_axis_fns = self.axis_options[y_axis_selection]
        shift_x_data = "Arc Length" in x_axis_selection
        ego_odom = self.ros_interface.ego_odom
        for topic, traj in self.msg_per_topic.items():
            if shift_x_data:
                x_data = x_axis_fns.trajectory_fn(traj, ego_odom)
            else:
                x_data = x_axis_fns.trajectory_fn(traj)
            y_data = y_axis_fns.trajectory_fn(traj)
            self.plotter.update_data(topic, x_data, y_data)
        if ego_odom is not None:
            self.plotter.update_ego_data(x_axis_fns.ego_fn(ego_odom), y_axis_fns.ego_fn(ego_odom))
        self.plotter.replot()
        self.canvas.draw_idle()

    def update_axis_labels(self, event=None):
        """Update plot axis labels when dropdown selection changes."""
        x_axis_selection = self.current_x_axis_selection.get()
        y_axis_selection = self.current_y_axis_selection.get()
        self.plotter.update_labels(x_axis_selection, y_axis_selection)
        self.canvas.draw_idle()


if __name__ == "__main__":
    root = tk.Tk()
    app = TkinterApp(root)
    root.mainloop()

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
from collections import Counter

from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from plotter import Plotter
from ros2_interface import ROS2Interface
from trajectory_data import get_data_functions
from trajectory_data import get_reference_arc_from_curvature
from trajectory_node_graph import TrajectoryNodeGraph


class TkinterApp:
    def __init__(self, root, ros_interface_node: ROS2Interface, config):
        self.root = root
        self.root.title("Autoware Trajectory Visualizer")
        self.root.geometry("800x600")
        self.ros_interface = ros_interface_node
        self.topics = config["initial_topics"]
        self.available_topics = []

        # Predetermined list for the dropdown
        self.axis_options = get_data_functions()
        axis_option_keys = [str(key) for key in self.axis_options.keys()]
        self.current_x_axis_selection = tk.StringVar()
        self.current_y_axis_selection = tk.StringVar()
        self.current_y_zoom = tk.DoubleVar(value=1.0)
        self.plot_configs = self._load_initial_plot_configs(config, axis_option_keys)
        self.active_plot_index = 0

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
        ttk.Label(self.left_frame, text="Plots:").grid(
            row=0, column=0, columnspan=2, padx=5, pady=5, sticky="w"
        )
        self.plot_listbox = tk.Listbox(
            self.left_frame,
            height=4,
            exportselection=False,
        )
        self.plot_listbox.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
        self.plot_listbox.bind("<<ListboxSelect>>", self.on_plot_select)

        self.add_plot_button = ttk.Button(self.left_frame, text="Add Plot", command=self.add_plot)
        self.add_plot_button.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.remove_plot_button = ttk.Button(
            self.left_frame, text="Remove Plot", command=self.remove_plot
        )
        self.remove_plot_button.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        # Dropdown List (Combobox)
        ttk.Label(self.left_frame, text="X-axis:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.x_axis_dropdown = ttk.Combobox(
            self.left_frame,
            textvariable=self.current_x_axis_selection,
            values=axis_option_keys,
            state="readonly",
        )
        self.x_axis_dropdown.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        # Dropdown List (Combobox)
        ttk.Label(self.left_frame, text="Y-axis:").grid(row=3, column=1, padx=5, pady=5, sticky="w")
        self.y_axis_dropdown = ttk.Combobox(
            self.left_frame,
            textvariable=self.current_y_axis_selection,
            values=axis_option_keys,
            state="readonly",
        )
        self.y_axis_dropdown.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

        ttk.Label(self.left_frame, text="Y zoom:").grid(row=5, column=0, padx=5, pady=(10, 0), sticky="w")
        self.y_zoom_scale = ttk.Scale(
            self.left_frame,
            from_=0.1,
            to=10.0,
            variable=self.current_y_zoom,
            orient=tk.HORIZONTAL,
            command=self.update_y_zoom,
        )
        self.y_zoom_scale.grid(row=6, column=0, padx=5, pady=5, sticky="ew")
        self.y_zoom_value_label = ttk.Label(self.left_frame, text="1.0x")
        self.y_zoom_value_label.grid(row=6, column=1, padx=5, pady=5, sticky="w")

        # bind dropdown selection events to update axis labels
        self.x_axis_dropdown.bind("<<ComboboxSelected>>", self.update_axis_labels)
        self.y_axis_dropdown.bind("<<ComboboxSelected>>", self.update_axis_labels)

        # Listbox with Multiple Selection
        ttk.Label(self.left_frame, text="Topics:").grid(
            row=7, column=0, columnspan=2, padx=5, pady=5, sticky="w"
        )
        self.listbox_frame = ttk.Frame(self.left_frame)
        self.listbox_frame.grid(row=8, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

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
        self.left_frame.grid_rowconfigure(1, weight=0)
        self.left_frame.grid_rowconfigure(8, weight=1)
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.left_frame.grid_columnconfigure(1, weight=1)

        # Button to Refresh List
        self.refresh_button = ttk.Button(
            self.left_frame, text="Refresh List", command=self.refresh_topic_list
        )
        self.refresh_button.grid(row=9, column=0, columnspan=2, padx=5, pady=10, sticky="ew")
        # --- Mid Frame --- Button to hide/show the left frame
        self.hide_button = ttk.Button(
            self.mid_frame, text="<", command=self.hide_show_left_frame, width=1
        )
        self.hide_button.pack(fill="both", expand=True)
        self.left_hidden = False
        # --- Right Frame Widgets ---
        # Matplotlib Plot Area
        self.plotter = Plotter()
        self.plotter.init_plot(self.plot_configs, [])
        self.canvas = FigureCanvasTkAgg(self.plotter.fig, master=self.right_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.msg_per_topic = {}
        self.needs_replot = True
        self._refresh_plot_listbox()
        self._set_axis_controls_from_active_plot()
        self.refresh_topic_list()
        # Auto-select initial topics from config
        self.select_initial_topics()

    def _match_axis_option(self, axis_option_keys, prefix, fallback_index):
        if not prefix:
            return axis_option_keys[fallback_index]
        for axis_option in axis_option_keys:
            if axis_option.casefold().startswith(prefix.casefold()):
                return axis_option
        return axis_option_keys[fallback_index]

    def _create_plot_config(self, x_axis: str, y_axis: str, y_zoom: float = 1.0):
        return {"name": "", "x_axis": x_axis, "y_axis": y_axis, "y_zoom": max(y_zoom, 1e-3)}

    def _load_initial_plot_configs(self, config, axis_option_keys):
        initial_plots = config.get("initial_plots", [])
        if initial_plots:
            plot_configs = []
            for plot_config in initial_plots:
                x_axis = self._match_axis_option(axis_option_keys, plot_config.get("x", ""), 0)
                y_axis = self._match_axis_option(axis_option_keys, plot_config.get("y", ""), 1)
                plot_configs.append(
                    self._create_plot_config(x_axis, y_axis, float(plot_config.get("y_zoom", 1.0)))
                )
            return plot_configs

        initial_axis = config.get("initial_axis", {})
        return [
            self._create_plot_config(
                self._match_axis_option(axis_option_keys, initial_axis.get("x", ""), 0),
                self._match_axis_option(axis_option_keys, initial_axis.get("y", ""), 1),
            )
        ]

    def _refresh_plot_listbox(self):
        self.plot_listbox.delete(0, tk.END)
        for index, plot_config in enumerate(self.plot_configs):
            plot_config["name"] = f"Plot {index + 1}"
            self.plot_listbox.insert(tk.END, plot_config["name"])

        if not self.plot_configs:
            return

        self.active_plot_index = max(0, min(self.active_plot_index, len(self.plot_configs) - 1))
        self.plot_listbox.selection_clear(0, tk.END)
        self.plot_listbox.selection_set(self.active_plot_index)
        self.plot_listbox.activate(self.active_plot_index)

    def _set_axis_controls_from_active_plot(self):
        if not self.plot_configs:
            return

        plot_config = self.plot_configs[self.active_plot_index]
        self.current_x_axis_selection.set(plot_config["x_axis"])
        self.current_y_axis_selection.set(plot_config["y_axis"])
        self.current_y_zoom.set(plot_config["y_zoom"])
        self.y_zoom_value_label.config(text=f"{plot_config['y_zoom']:.1f}x")

    def _sync_active_plot_config(self):
        if not self.plot_configs:
            return

        self.plot_configs[self.active_plot_index]["x_axis"] = self.current_x_axis_selection.get()
        self.plot_configs[self.active_plot_index]["y_axis"] = self.current_y_axis_selection.get()
        self.plot_configs[self.active_plot_index]["y_zoom"] = max(self.current_y_zoom.get(), 1e-3)

    def on_plot_select(self, event):  # noqa: ARG002 unused-argument
        selection = self.plot_listbox.curselection()
        if not selection:
            return

        self._sync_active_plot_config()
        self.active_plot_index = selection[0]
        self._set_axis_controls_from_active_plot()

    def add_plot(self):
        self._sync_active_plot_config()
        active_plot = self.plot_configs[self.active_plot_index]
        self.plot_configs.append(
            self._create_plot_config(
                active_plot["x_axis"], active_plot["y_axis"], active_plot["y_zoom"]
            )
        )
        self.active_plot_index = len(self.plot_configs) - 1
        self._refresh_plot_listbox()
        self._set_axis_controls_from_active_plot()
        self.plotter.init_plot(self.plot_configs, self._get_expected_plot_names())
        self.needs_replot = True

    def remove_plot(self):
        if len(self.plot_configs) <= 1:
            return

        self._sync_active_plot_config()
        self.plot_configs.pop(self.active_plot_index)
        self.active_plot_index = min(self.active_plot_index, len(self.plot_configs) - 1)
        self._refresh_plot_listbox()
        self._set_axis_controls_from_active_plot()
        self.plotter.init_plot(self.plot_configs, self._get_expected_plot_names())
        self.needs_replot = True

    def on_listbox_select(self, event):  # noqa: ARG002 unused-argument
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
            # Update self.topics with the new selection
            self._update_topics_from_selection(current_selection)
            self._plot_without_updating_topics(current_selection)
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

    def select_initial_topics(self):
        """Auto-select topics that are currently in self.topics (initially from config.yaml)."""
        # Convert self.topics to set of topic names only (ignore msg type for matching)
        selected_topic_names = {topic for topic, _ in self.topics}

        # Find matching topics in the listbox
        indices_to_select = []
        topic_idx = 0
        listbox_idx = 0

        for i in range(self.listbox.size()):
            item_text = self.listbox.get(i)
            if item_text.startswith("---"):
                listbox_idx += 1
                continue

            if topic_idx < len(self.available_topics):
                current_topic_name, _ = self.available_topics[topic_idx]
                # Select if this topic is in self.topics
                if current_topic_name in selected_topic_names:
                    indices_to_select.append(listbox_idx)
                    self.listbox.selection_set(listbox_idx)

            topic_idx += 1
            listbox_idx += 1

        # Plot selected topics (without updating self.topics)
        if indices_to_select:
            self._plot_without_updating_topics(indices_to_select)

    def refresh_topic_list(self):
        self.listbox.delete(0, tk.END)  # Clear existing items

        # Clear selection but keep self.topics unchanged
        self.listbox.selection_clear(0, tk.END)

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

            # Build available_topics list with node separators
            self.available_topics = []
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
                        self.available_topics.append((topic_name, msg_type))
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
                    # Store in available_topics list
                    self.available_topics.append((topic_name, msg_type))

                    # Display in original format
                    display_text = f"{topic_name} [{msg_type.split('/')[-1]}]"
                    self.listbox.insert(tk.END, display_text)

            # Restore selection state based on self.topics
            self.select_initial_topics()
            return
        except Exception as e:
            # Fall back to original method if analysis fails
            print(f"TrajectoryNodeGraph analysis failed: {e}")

        # Original implementation (fallback)
        self.available_topics = self.ros_interface.get_trajectory_topics()
        for topic, msg_type in self.available_topics:
            self.listbox.insert(tk.END, f"{topic} [{msg_type.split('/')[-1]}]")

        # Restore selection state based on self.topics
        self.select_initial_topics()

    def update(self, topic, msg):
        self.msg_per_topic[topic] = msg
        self.needs_replot = True

    def _get_candidate_generator_names(self, msg):
        generator_names = {}
        for info in getattr(msg, "generator_info", []):
            generator_id = tuple(getattr(info.generator_id, "uuid", []))
            generator_name = getattr(getattr(info, "generator_name", None), "data", "")
            if generator_id:
                generator_names[generator_id] = generator_name
        return generator_names

    def _format_generator_plot_name(self, topic, generator_name, generator_id, duplicate_count, index):
        if generator_name:
            plot_name = f"{topic} [{generator_name}]"
            if duplicate_count <= 1:
                return plot_name
            if generator_id:
                suffix = "".join(f"{value:02x}" for value in generator_id[:4])
                return f"{plot_name} ({suffix})"
            return f"{plot_name} #{index}"

        if generator_id:
            suffix = "".join(f"{value:02x}" for value in generator_id[:4])
            return f"{topic} [{suffix}]"
        return f"{topic} #{index}"

    def _get_candidate_plot_entries(self, topic, msg):
        generator_names = self._get_candidate_generator_names(msg)
        candidates = list(getattr(msg, "candidate_trajectories", []))
        base_names = []
        for candidate in candidates:
            generator_id = tuple(getattr(candidate.generator_id, "uuid", []))
            generator_name = generator_names.get(generator_id, "")
            if generator_name:
                base_names.append(generator_name)
            elif generator_id:
                base_names.append("".join(f"{value:02x}" for value in generator_id[:4]))
            else:
                base_names.append("")
        duplicate_counts = Counter(base_names)
        plot_entries = []
        for index, candidate in enumerate(candidates):
            generator_id = tuple(getattr(candidate.generator_id, "uuid", []))
            generator_name = generator_names.get(generator_id, "")
            base_name = base_names[index]
            plot_name = self._format_generator_plot_name(
                topic, generator_name, generator_id, duplicate_counts[base_name], index
            )
            plot_entries.append((plot_name, candidate))
        return plot_entries

    def _get_expected_plot_names(self):
        plot_names = []
        for topic, msg_type in self.topics:
            msg = self.msg_per_topic.get(topic)
            if msg_type.endswith("CandidateTrajectories"):
                if msg is not None:
                    plot_names.extend(
                        [plot_name for plot_name, _ in self._get_candidate_plot_entries(topic, msg)]
                    )
                continue
            plot_names.append(topic)
        return plot_names

    def _get_plot_entries(self):
        plot_entries = []
        for topic, msg_type in self.topics:
            msg = self.msg_per_topic.get(topic)
            if msg is None:
                continue
            if msg_type.endswith("CandidateTrajectories"):
                plot_entries.extend(self._get_candidate_plot_entries(topic, msg))
            else:
                plot_entries.append((topic, msg))
        return plot_entries

    def _update_topics_from_selection(self, topic_indexes):
        """Update self.topics based on listbox selection."""
        selected_topics = []
        topic_idx = 0
        listbox_idx = 0

        for i in range(self.listbox.size()):
            item_text = self.listbox.get(i)
            if item_text.startswith("---"):
                listbox_idx += 1
                continue

            if listbox_idx in topic_indexes:
                if topic_idx < len(self.available_topics):
                    selected_topics.append(self.available_topics[topic_idx])

            topic_idx += 1
            listbox_idx += 1

        self.topics = selected_topics

    def _plot_without_updating_topics(self, topic_indexes):
        self.ros_interface.remove_callbacks()
        self.msg_per_topic.clear()
        self._sync_active_plot_config()
        self.plotter.reset_y_limits()

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
                if topic_idx < len(self.available_topics):
                    selected_topics.append(self.available_topics[topic_idx])

            topic_idx += 1
            listbox_idx += 1

        initial_plot_names = [
            topic for topic, msg_type in selected_topics if not msg_type.endswith("CandidateTrajectories")
        ]
        self.plotter.init_plot(self.plot_configs, initial_plot_names)
        self.needs_replot = True
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
            elif msg_type.endswith("CandidateTrajectories"):
                self.ros_interface.add_callback(
                    topic,
                    CandidateTrajectories,
                    lambda msg, captured_topic=topic: self.update(captured_topic, msg),
                )
            elif msg_type.endswith("Path"):
                self.ros_interface.add_callback(
                    topic, Path, lambda msg, captured_topic=topic: self.update(captured_topic, msg)
                )

    def replot(self):
        if not self.needs_replot:
            return

        self._sync_active_plot_config()
        ego_odom = self.ros_interface.ego_odom
        plot_names = self._get_expected_plot_names()
        plot_entries = self._get_plot_entries()
        self.plotter.configure_plots(self.plot_configs, plot_names)

        for plot_index, plot_config in enumerate(self.plot_configs):
            x_axis_selection = plot_config["x_axis"]
            y_axis_selection = plot_config["y_axis"]
            x_axis_fns = self.axis_options[x_axis_selection]
            y_axis_fns = self.axis_options[y_axis_selection]
            shift_x_data = "Arc Length" in x_axis_selection
            show_reference_arc = shift_x_data and "Curvature" in y_axis_selection
            self.plotter.set_reference_arc_visibility(plot_index, show_reference_arc, plot_names)

            y_data_list = []
            for topic, traj in plot_entries:
                if shift_x_data:
                    x_data = x_axis_fns.trajectory_fn(traj, ego_odom)
                else:
                    x_data = x_axis_fns.trajectory_fn(traj)
                y_data = y_axis_fns.trajectory_fn(traj)
                y_data_list.append(y_data)
                self.plotter.update_data(plot_index, topic, x_data, y_data)
                if show_reference_arc:
                    arc_x, arc_y = get_reference_arc_from_curvature(x_data, y_data)
                    self.plotter.update_reference_arc_data(plot_index, topic, arc_x, arc_y)

            if self.plotter.fixed_y_limits[plot_index] is None:
                self.plotter.update_fixed_y_limits(plot_index, y_data_list)
            if ego_odom is not None:
                self.plotter.update_ego_data(
                    plot_index,
                    x_axis_fns.ego_fn(ego_odom),
                    y_axis_fns.ego_fn(ego_odom),
                )
        self.plotter.replot()
        self.canvas.draw_idle()
        self.needs_replot = False

    def update_axis_labels(self, event=None):
        """Update plot axis labels when dropdown selection changes."""
        self._sync_active_plot_config()
        self.plotter.reset_y_limits(self.active_plot_index)
        self.plotter.update_labels(
            self.active_plot_index,
            self.current_x_axis_selection.get(),
            self.current_y_axis_selection.get(),
            self._get_expected_plot_names(),
        )
        self.needs_replot = True

    def update_y_zoom(self, value):
        zoom_factor = float(value)
        self.y_zoom_value_label.config(text=f"{zoom_factor:.1f}x")
        if not self.plot_configs:
            return

        self.plot_configs[self.active_plot_index]["y_zoom"] = max(zoom_factor, 1e-3)
        self.plotter.set_y_zoom_factor(self.active_plot_index, zoom_factor)
        self.needs_replot = True


if __name__ == "__main__":
    root = tk.Tk()
    app = TkinterApp(root)
    root.mainloop()

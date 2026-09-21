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
import time

import matplotlib.pyplot as plt
import numpy as np


class Plotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.plots = {}
        (self.ego_plot,) = self.ax.plot([], [], marker="x", label="ego", alpha=0.7, color="black")
        # Track last warning time for each plot to avoid spam
        self.last_warning_time = {}
        self.warning_interval = 3.0  # seconds

    def init_plot(self, x_label: str, y_label: str, plot_names: list[str]):
        cmap = "tab10" if len(plot_names) <= 10 else "tab20"
        colormap = plt.cm.get_cmap(cmap, len(plot_names))
        self.ax.set_prop_cycle(color=[colormap(i) for i in range(len(plot_names))])
        self.plots.clear()
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)

        # set aspect ratio to 1:1 when both axes show position (X value and Y value)
        if "X value" in x_label and "Y value" in y_label:
            self.ax.set_aspect("equal", adjustable="box")
        else:
            self.ax.set_aspect("auto")

        for name in plot_names:
            (self.plots[name],) = self.ax.plot(
                [], [], marker="o", linestyle="-", label=name, alpha=0.6
            )
        (self.ego_plot,) = self.ax.plot([], [], marker="x", label="ego", alpha=1.0, color="black")

        # Only create legend if there are labeled artists
        if plot_names or self.ego_plot.get_label():
            self.ax.legend()

    def update_data(self, name, x_data, y_data):
        if name not in self.plots:
            print(f"{name} plot not found")
            return

        x_data = np.asarray(x_data) if x_data is not None else np.array([])
        y_data = np.asarray(y_data) if y_data is not None else np.array([])

        # Check if data is empty or sizes don't match
        if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
            self.plots[name].set_data([], [])

            # Only print warning if enough time has passed since last warning for this plot
            current_time = time.time()
            if (
                name not in self.last_warning_time
                or (current_time - self.last_warning_time[name]) >= self.warning_interval
            ):
                print(
                    f"Warning: Data size mismatch for {name} - x: {len(x_data)}, y: {len(y_data)}. Setting empty data."
                )
                self.last_warning_time[name] = current_time
        else:
            self.plots[name].set_data(x_data, y_data)

    def update_ego_data(self, x_data, y_data):
        # Handle single value data for ego position
        if x_data is not None and y_data is not None:
            self.ego_plot.set_data([x_data], [y_data])
        else:
            self.ego_plot.set_data([], [])

    def update_labels(self, x_label: str, y_label: str):
        """Update axis labels without clearing the plot."""
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)

        # set aspect ratio to 1:1 when both axes show position (X value and Y value)
        if "X value" in x_label and "Y value" in y_label:
            self.ax.set_aspect("equal", adjustable="box")
        else:
            self.ax.set_aspect("auto")

    def replot(self):
        self.ax.relim()
        self.ax.autoscale_view()
        # Only update legend if there are labeled artists
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend()

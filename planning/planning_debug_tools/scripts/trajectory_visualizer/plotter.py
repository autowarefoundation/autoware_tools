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
import math

import matplotlib.pyplot as plt


class Plotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.plots = {}
        self.arc_ax = None
        self.arc_plots = {}
        self.y_zoom_factor = 1.0
        self.fixed_y_limits = None
        (self.ego_plot,) = self.ax.plot([], [], marker="x", label="ego", alpha=0.7, color="black")

    def _configure_axes(self, x_label: str, y_label: str):
        self.ax.grid(True)
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)

        # set aspect ratio to 1:1 when both axes show position (X value and Y value)
        if "X value" in x_label and "Y value" in y_label:
            self.ax.set_aspect("equal", adjustable="box")
        else:
            self.ax.set_aspect("auto")

    def _get_color_map(self, plot_names: list[str]):
        cmap = "tab10" if len(plot_names) <= 10 else "tab20"
        colormap = plt.cm.get_cmap(cmap, max(len(plot_names), 1))
        return {name: colormap(index) for index, name in enumerate(plot_names)}

    def init_plot(self, x_label: str, y_label: str, plot_names: list[str]):
        self.plots.clear()
        self.arc_plots.clear()
        self.arc_ax = None
        self.fixed_y_limits = None
        self.ax.clear()
        self._configure_axes(x_label, y_label)

        colors = self._get_color_map(plot_names)
        for name in plot_names:
            (self.plots[name],) = self.ax.plot(
                [], [], marker="o", linestyle="-", label=name, alpha=0.6, color=colors[name]
            )
        (self.ego_plot,) = self.ax.plot([], [], marker="x", label="ego", alpha=1.0, color="black")

        # Only create legend if there are labeled artists
        if plot_names or self.ego_plot.get_label():
            self.ax.legend()

    def reset_y_limits(self):
        self.fixed_y_limits = None

    def set_y_zoom_factor(self, zoom_factor: float):
        self.y_zoom_factor = max(zoom_factor, 1e-3)

    def update_fixed_y_limits(self, y_data_list):
        valid_values = []
        for y_data in y_data_list:
            for value in y_data:
                if value is None:
                    continue
                numeric_value = float(value)
                if math.isfinite(numeric_value):
                    valid_values.append(numeric_value)

        if not valid_values:
            return

        y_min = min(valid_values)
        y_max = max(valid_values)
        if y_min == y_max:
            padding = max(abs(y_min) * 0.1, 1.0)
            y_min -= padding
            y_max += padding
        self.fixed_y_limits = (y_min, y_max)

    def _apply_fixed_y_limits(self):
        if self.fixed_y_limits is None:
            return

        y_min, y_max = self.fixed_y_limits
        y_center = 0.5 * (y_min + y_max)
        half_range = 0.5 * (y_max - y_min)
        if half_range <= 0.0:
            half_range = 1.0
        zoomed_half_range = half_range / self.y_zoom_factor
        self.ax.set_ylim(y_center - zoomed_half_range, y_center + zoomed_half_range)

    def set_reference_arc_visibility(self, visible: bool):
        if visible:
            if self.arc_ax is None:
                self.arc_ax = self.ax.inset_axes([0.62, 0.08, 0.34, 0.34])
                self.arc_ax.grid(True)
                self.arc_ax.set_title("Arc from k-l", fontsize=9)
                self.arc_ax.set_xlabel("x [m]", fontsize=8)
                self.arc_ax.set_ylabel("y [m]", fontsize=8)
                self.arc_ax.tick_params(axis="both", labelsize=8)
                self.arc_ax.set_aspect("equal", adjustable="box")
            return

        if self.arc_ax is not None:
            self.arc_ax.remove()
            self.arc_ax = None
        self.arc_plots.clear()

    def sync_plots(self, plot_names: list[str]):
        removed_names = [name for name in self.plots if name not in plot_names]
        for name in removed_names:
            self.plots[name].remove()
            del self.plots[name]

        if self.arc_ax is not None:
            removed_arc_names = [name for name in self.arc_plots if name not in plot_names]
            for name in removed_arc_names:
                self.arc_plots[name].remove()
                del self.arc_plots[name]

        colors = self._get_color_map(plot_names)
        synced_plots = {}
        for name in plot_names:
            if name not in self.plots:
                (self.plots[name],) = self.ax.plot([], [], marker="o", linestyle="-", alpha=0.6)
            self.plots[name].set_label(name)
            self.plots[name].set_color(colors[name])
            synced_plots[name] = self.plots[name]
        self.plots = synced_plots

        if self.arc_ax is not None:
            synced_arc_plots = {}
            for name in plot_names:
                if name not in self.arc_plots:
                    (self.arc_plots[name],) = self.arc_ax.plot(
                        [], [], linestyle="-", alpha=0.8, linewidth=1.5
                    )
                self.arc_plots[name].set_color(colors[name])
                synced_arc_plots[name] = self.arc_plots[name]
            self.arc_plots = synced_arc_plots

    def update_data(self, name, x_data, y_data):
        if name not in self.plots:
            print(f"{name} plot not found")
            return
        self.plots[name].set_data(x_data, y_data)

    def update_reference_arc_data(self, name, x_data, y_data):
        if self.arc_ax is None or name not in self.arc_plots:
            return
        self.arc_plots[name].set_data(x_data, y_data)

    def update_ego_data(self, x_data, y_data):
        self.ego_plot.set_data([x_data], [y_data])

    def update_labels(self, x_label: str, y_label: str):
        """Update axis labels without clearing the plot."""
        self._configure_axes(x_label, y_label)

    def replot(self):
        self.ax.relim()
        self.ax.autoscale_view(scalex=True, scaley=False)
        self._apply_fixed_y_limits()
        if self.arc_ax is not None:
            self.arc_ax.relim()
            self.arc_ax.autoscale_view()
        # Only update legend if there are labeled artists
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend()

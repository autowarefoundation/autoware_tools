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
    REFERENCE_ARC_MIN_Y_PADDING = 0.5
    REFERENCE_ARC_Y_PADDING_RATIO = 0.1

    def __init__(self):
        self.fig = plt.figure()
        self.axes = []
        self.plots = []
        self.arc_axes = []
        self.arc_plots = []
        self.arc_y_limits = []
        self.ego_plots = []
        self.legend_dirty = []
        self.y_zoom_factors = []
        self.fixed_y_limits = []
        self.axis_labels = []
        self.plot_names = []

    def _configure_axes(self, ax, x_label: str, y_label: str):
        ax.grid(True)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)

        # set aspect ratio to 1:1 when both axes show position (X value and Y value)
        if "X value" in x_label and "Y value" in y_label:
            ax.set_aspect("equal", adjustable="box")
        else:
            ax.set_aspect("auto")

    def _get_color_map(self, plot_names: list[str]):
        cmap = "tab10" if len(plot_names) <= 10 else "tab20"
        colormap = plt.cm.get_cmap(cmap, max(len(plot_names), 1))
        return {name: colormap(index) for index, name in enumerate(plot_names)}

    def _build_layout(self, plot_configs: list[dict]):
        self.fig.clear()
        plot_count = max(len(plot_configs), 1)
        axes = self.fig.subplots(plot_count, 1, squeeze=False)
        self.axes = [ax for row in axes for ax in row]
        self.plots = [{} for _ in range(plot_count)]
        self.arc_axes = [None for _ in range(plot_count)]
        self.arc_plots = [{} for _ in range(plot_count)]
        self.arc_y_limits = [None for _ in range(plot_count)]
        self.ego_plots = [None for _ in range(plot_count)]
        self.legend_dirty = [True for _ in range(plot_count)]
        self.fixed_y_limits = [None for _ in range(plot_count)]
        self.y_zoom_factors = [1.0 for _ in range(plot_count)]
        self.axis_labels = [("", "") for _ in range(plot_count)]
        self.plot_names = []

    def _update_legend(self, plot_index: int):
        if not self.legend_dirty[plot_index]:
            return

        handles, labels = self.axes[plot_index].get_legend_handles_labels()
        if handles:
            self.axes[plot_index].legend()
        else:
            legend = self.axes[plot_index].get_legend()
            if legend is not None:
                legend.remove()
        self.legend_dirty[plot_index] = False

    def _sync_axis_plots(self, plot_index: int, plot_names: list[str], colors: dict[str, tuple]):
        ax = self.axes[plot_index]
        current_plots = self.plots[plot_index]
        removed_names = [name for name in current_plots if name not in plot_names]
        for name in removed_names:
            current_plots[name].remove()
            del current_plots[name]
        if removed_names:
            self.legend_dirty[plot_index] = True

        synced_plots = {}
        for name in plot_names:
            if name not in current_plots:
                (current_plots[name],) = ax.plot([], [], marker="o", linestyle="-", alpha=0.6)
                self.legend_dirty[plot_index] = True
            current_plots[name].set_label(name)
            current_plots[name].set_color(colors[name])
            synced_plots[name] = current_plots[name]
        self.plots[plot_index] = synced_plots

        if self.ego_plots[plot_index] is None:
            (self.ego_plots[plot_index],) = ax.plot(
                [], [], marker="x", label="ego", alpha=1.0, color="black"
            )
            self.legend_dirty[plot_index] = True

        arc_ax = self.arc_axes[plot_index]
        if arc_ax is None:
            return

        current_arc_plots = self.arc_plots[plot_index]
        removed_arc_names = [name for name in current_arc_plots if name not in plot_names]
        for name in removed_arc_names:
            current_arc_plots[name].remove()
            del current_arc_plots[name]

        synced_arc_plots = {}
        for name in plot_names:
            if name not in current_arc_plots:
                (current_arc_plots[name],) = arc_ax.plot(
                    [], [], linestyle="-", alpha=0.8, linewidth=1.5
                )
            current_arc_plots[name].set_color(colors[name])
            synced_arc_plots[name] = current_arc_plots[name]
        self.arc_plots[plot_index] = synced_arc_plots

    def _reset_axis(self, plot_index: int, x_label: str, y_label: str, plot_names: list[str]):
        if self.arc_axes[plot_index] is not None:
            self.arc_axes[plot_index].remove()
        self.axes[plot_index].clear()
        self.arc_axes[plot_index] = None
        self.arc_plots[plot_index] = {}
        self.arc_y_limits[plot_index] = None
        self.plots[plot_index] = {}
        self.ego_plots[plot_index] = None
        self.legend_dirty[plot_index] = True
        self.axis_labels[plot_index] = (x_label, y_label)
        self._configure_axes(self.axes[plot_index], x_label, y_label)
        colors = self._get_color_map(plot_names)
        self._sync_axis_plots(plot_index, plot_names, colors)
        self._update_legend(plot_index)

    def configure_plots(self, plot_configs: list[dict], plot_names: list[str]):
        if len(self.axes) != max(len(plot_configs), 1):
            self._build_layout(plot_configs)

        plot_names_changed = self.plot_names != list(plot_names)
        colors = self._get_color_map(plot_names)
        for plot_index, plot_config in enumerate(plot_configs):
            x_label = plot_config["x_axis"]
            y_label = plot_config["y_axis"]
            self.y_zoom_factors[plot_index] = max(plot_config.get("y_zoom", 1.0), 1e-3)

            if self.axis_labels[plot_index] != (x_label, y_label):
                self.fixed_y_limits[plot_index] = None
                self._reset_axis(plot_index, x_label, y_label, plot_names)
                continue

            if plot_names_changed:
                self._sync_axis_plots(plot_index, plot_names, colors)
                self._update_legend(plot_index)

        self.plot_names = list(plot_names)

    def init_plot(self, plot_configs: list[dict], plot_names: list[str]):
        self._build_layout(plot_configs)
        for plot_index in range(len(plot_configs)):
            self.fixed_y_limits[plot_index] = None
        self.configure_plots(plot_configs, plot_names)

    def reset_y_limits(self, plot_index: int | None = None):
        if plot_index is None:
            self.fixed_y_limits = [None for _ in self.fixed_y_limits]
            self.arc_y_limits = [None for _ in self.arc_y_limits]
            return
        self.fixed_y_limits[plot_index] = None
        self.arc_y_limits[plot_index] = None

    def set_y_zoom_factor(self, plot_index: int, zoom_factor: float):
        self.y_zoom_factors[plot_index] = max(zoom_factor, 1e-3)

    def _collect_valid_range(self, data_list):
        valid_values = []
        for data in data_list:
            for value in data:
                if value is None:
                    continue
                numeric_value = float(value)
                if math.isfinite(numeric_value):
                    valid_values.append(numeric_value)

        if not valid_values:
            return None

        return min(valid_values), max(valid_values)

    def _expand_range(self, current_limits, next_limits):
        if next_limits is None:
            return current_limits
        if current_limits is None:
            return next_limits
        return (
            min(current_limits[0], next_limits[0]),
            max(current_limits[1], next_limits[1]),
        )

    def _build_padded_reference_arc_limits(self, y_limits):
        if y_limits is None:
            return None

        y_min, y_max = y_limits
        y_range = y_max - y_min
        padding = max(
            y_range * self.REFERENCE_ARC_Y_PADDING_RATIO,
            self.REFERENCE_ARC_MIN_Y_PADDING,
        )
        if y_range <= 0.0:
            y_center = 0.5 * (y_min + y_max)
            return (y_center - padding, y_center + padding)
        return (y_min - padding, y_max + padding)

    def update_fixed_y_limits(self, plot_index: int, y_data_list):
        y_limits = self._collect_valid_range(y_data_list)
        if y_limits is None:
            return

        y_min, y_max = y_limits
        if y_min == y_max:
            padding = max(abs(y_min) * 0.1, 1.0)
            y_min -= padding
            y_max += padding
        self.fixed_y_limits[plot_index] = self._expand_range(
            self.fixed_y_limits[plot_index],
            (y_min, y_max),
        )

    def update_reference_arc_y_limits(self, plot_index: int, y_data_list):
        if self.arc_axes[plot_index] is None:
            return

        padded_limits = self._build_padded_reference_arc_limits(self._collect_valid_range(y_data_list))
        self.arc_y_limits[plot_index] = self._expand_range(
            self.arc_y_limits[plot_index],
            padded_limits,
        )

    def _apply_fixed_y_limits(self, plot_index: int):
        if self.fixed_y_limits[plot_index] is None:
            return

        y_min, y_max = self.fixed_y_limits[plot_index]
        y_center = 0.5 * (y_min + y_max)
        half_range = 0.5 * (y_max - y_min)
        if half_range <= 0.0:
            half_range = 1.0
        zoomed_half_range = half_range / self.y_zoom_factors[plot_index]
        self.axes[plot_index].set_ylim(y_center - zoomed_half_range, y_center + zoomed_half_range)

    def set_reference_arc_visibility(self, plot_index: int, visible: bool, plot_names: list[str]):
        if visible:
            if self.arc_axes[plot_index] is None:
                self.arc_axes[plot_index] = self.axes[plot_index].inset_axes([0.62, 0.08, 0.34, 0.34])
                self.arc_axes[plot_index].grid(True)
                self.arc_axes[plot_index].set_title("Arc from k-l", fontsize=9)
                self.arc_axes[plot_index].set_xlabel("x [m]", fontsize=8)
                self.arc_axes[plot_index].set_ylabel("y [m]", fontsize=8)
                self.arc_axes[plot_index].tick_params(axis="both", labelsize=8)
                self.arc_axes[plot_index].set_aspect("auto")
            if self.arc_axes[plot_index] is None or list(self.arc_plots[plot_index].keys()) != plot_names:
                colors = self._get_color_map(plot_names)
                self._sync_axis_plots(plot_index, plot_names, colors)
            return

        if self.arc_axes[plot_index] is not None:
            self.arc_axes[plot_index].remove()
            self.arc_axes[plot_index] = None
        self.arc_plots[plot_index] = {}
        self.arc_y_limits[plot_index] = None

    def update_data(self, plot_index: int, name, x_data, y_data):
        if name not in self.plots[plot_index]:
            print(f"{name} plot not found")
            return
        self.plots[plot_index][name].set_data(x_data, y_data)

    def update_reference_arc_data(self, plot_index: int, name, x_data, y_data):
        if self.arc_axes[plot_index] is None or name not in self.arc_plots[plot_index]:
            return
        self.arc_plots[plot_index][name].set_data(x_data, y_data)

    def update_ego_data(self, plot_index: int, x_data, y_data):
        if self.ego_plots[plot_index] is None:
            return
        self.ego_plots[plot_index].set_data([x_data], [y_data])

    def update_labels(self, plot_index: int, x_label: str, y_label: str, plot_names: list[str]):
        """Update axis labels without changing subplot count."""
        if self.axis_labels[plot_index] != (x_label, y_label):
            self.fixed_y_limits[plot_index] = None
            self._reset_axis(plot_index, x_label, y_label, plot_names)
            return
        self._configure_axes(self.axes[plot_index], x_label, y_label)
        self._update_legend(plot_index)

    def _apply_reference_arc_y_padding(self, plot_index: int):
        arc_ax = self.arc_axes[plot_index]
        if arc_ax is None:
            return

        if self.arc_y_limits[plot_index] is None:
            y_min, y_max = arc_ax.get_ylim()
            if not (math.isfinite(y_min) and math.isfinite(y_max)):
                return
            self.arc_y_limits[plot_index] = self._build_padded_reference_arc_limits((y_min, y_max))

        padded_y_limits = self.arc_y_limits[plot_index]
        if padded_y_limits is None:
            return

        arc_ax.set_ylim(*padded_y_limits)

    def replot(self):
        for plot_index, ax in enumerate(self.axes):
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=False)
            self._apply_fixed_y_limits(plot_index)
            if self.arc_axes[plot_index] is not None:
                self.arc_axes[plot_index].relim()
                self.arc_axes[plot_index].autoscale_view()
                self._apply_reference_arc_y_padding(plot_index)
            self._update_legend(plot_index)

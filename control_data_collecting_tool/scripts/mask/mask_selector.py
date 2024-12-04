import os
import signal
import threading
import time

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml


class DataCollectingMaskSelector:
    def __init__(
        self,
        window_title="Data Collecting Mask Selector",
        x_min=0,
        x_max=10,
        y_min=-5,
        y_max=5,
        num_bins_x=10,
        num_bins_y=10,
        default_number=1,
        xlabel="X Axis",
        ylabel="Y Axis",
        mask_xy=None,
    ):
        self.grid_update_time_interval = 5.0
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.num_bins_x, self.num_bins_y = num_bins_x, num_bins_y
        self.x_bins = np.linspace(self.x_min, self.x_max, self.num_bins_x + 1)
        self.y_bins = np.linspace(self.y_min, self.y_max, self.num_bins_y + 1)
        self.x_bin_centers = (self.x_bins[:-1] + self.x_bins[1:]) / 2
        self.y_bin_centers = (self.y_bins[:-1] + self.y_bins[1:]) / 2
        self.default_number = default_number
        self.xlabel = xlabel
        self.ylabel = ylabel

        # Initialize mask_xy with the provided array or default to ones
        self.mask_xy = (
            mask_xy
            if mask_xy is not None
            else np.ones((self.num_bins_x, self.num_bins_y), dtype=int)
        )

        self.dragging = False
        self.select_mode = True
        self.text_objects = {}

        # Create the plot and configure the axes
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title(window_title)
        self.ax.set_xlim([self.x_min, self.x_max])
        self.ax.set_ylim([self.y_max, self.y_min])

        # Set up grid lines and hide major tick labels
        self.ax.set_xticks(self.x_bins)
        self.ax.set_yticks(self.y_bins[::-1])
        self.ax.set_xticklabels([])  # Hide X-axis major tick labels
        self.ax.set_yticklabels([])  # Hide Y-axis major tick labels

        # Set minor tick labels at bin centers
        self.ax.set_xticks(self.x_bin_centers, minor=True)
        self.ax.set_yticks(self.y_bin_centers[::-1], minor=True)
        self.ax.set_xticklabels([f"{x:.2f}" for x in self.x_bin_centers], minor=True)
        self.ax.set_yticklabels([f"{y:.2f}" for y in self.y_bin_centers[::-1]], minor=True)

        # Configure grid lines and labels
        self.ax.grid(True, color="black", linestyle="-", linewidth=1.5)
        self.ax.set_title(window_title)
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)
        plt.ion()
        plt.show(block=False)

        # Initialize cells with provided mask_xy
        self.clicked_cells = self.initialize_cells()

        # Connect mouse events to handlers
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)

    def initialize_cells(self):
        clicked_cells = {}
        for x_bin in range(self.num_bins_x):
            for y_bin in range(self.num_bins_y):
                cell_x = self.x_bin_centers[x_bin]
                cell_y = self.y_bin_centers[y_bin]
                if self.mask_xy[x_bin, y_bin] == 1:
                    rect = patches.Rectangle(
                        (
                            cell_x - (self.x_bins[1] - self.x_bins[0]) / 2,
                            cell_y - (self.y_bins[1] - self.y_bins[0]) / 2,
                        ),
                        self.x_bins[1] - self.x_bins[0],
                        self.y_bins[1] - self.y_bins[0],
                        linewidth=1,
                        edgecolor="yellowgreen",
                        facecolor="yellowgreen",
                    )
                    self.ax.add_patch(rect)
                    clicked_cells[(cell_x, cell_y)] = rect

                    text = self.ax.text(
                        cell_x,
                        cell_y,
                        str(self.default_number),
                        ha="center",
                        va="center",
                        fontsize=10,
                        color="black",
                    )
                    self.text_objects[(cell_x, cell_y)] = text
                else:
                    rect = None

        return clicked_cells

    def on_press(self, event):
        self.dragging = True
        self.toggle_select_mode(event)
        self.update_cell(event)

    def on_release(self, event):
        self.dragging = False

    def on_motion(self, event):
        if self.dragging:
            self.update_cell(event)

    def toggle_select_mode(self, event):
        _, _, cell_center = self.get_cell_from_event(event)
        if cell_center is not None:
            self.select_mode = cell_center not in self.clicked_cells

    def get_cell_from_event(self, event):
        if event.xdata is None or event.ydata is None:
            return None, None, None

        x_bin = np.digitize(event.xdata, self.x_bins) - 1
        y_bin = np.digitize(event.ydata, self.y_bins) - 1

        if 0 <= x_bin < self.num_bins_x and 0 <= y_bin < self.num_bins_y:
            cell_x = self.x_bin_centers[x_bin]
            cell_y = self.y_bin_centers[y_bin]
            cell_center = (cell_x, cell_y)
            return x_bin, y_bin, cell_center
        else:
            return None, None, None

    def update_cell(self, event):
        x_bin, y_bin, cell_center = self.get_cell_from_event(event)
        if cell_center is None:
            return

        if self.select_mode:
            if cell_center not in self.clicked_cells:
                # Add the rectangle and text
                rect = patches.Rectangle(
                    (
                        cell_center[0] - (self.x_bins[1] - self.x_bins[0]) / 2,
                        cell_center[1] - (self.y_bins[1] - self.y_bins[0]) / 2,
                    ),
                    self.x_bins[1] - self.x_bins[0],
                    self.y_bins[1] - self.y_bins[0],
                    linewidth=1,
                    edgecolor="yellowgreen",
                    facecolor="yellowgreen",
                )
                self.ax.add_patch(rect)
                text = self.ax.text(
                    cell_center[0],
                    cell_center[1],
                    str(self.default_number),
                    ha="center",
                    va="center",
                    fontsize=10,
                    color="black",
                )
                self.text_objects[cell_center] = text
                self.clicked_cells[cell_center] = rect
                self.mask_xy[x_bin, y_bin] = 1
        else:
            if cell_center in self.clicked_cells:
                # Remove the rectangle
                self.clicked_cells[cell_center].remove()
                del self.clicked_cells[cell_center]

                # Remove the text
                # if cell_center in self.text_objects:
                self.text_objects[cell_center].remove()
                del self.text_objects[cell_center]

                # Update the mask
                self.mask_xy[x_bin, y_bin] = 0

        # Update the plot
        self.fig.canvas.draw_idle()


def load_config_from_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"YAML file '{file_path}' not found.")
    with open(file_path, "r") as file:
        try:
            return yaml.safe_load(file)
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file '{file_path}': {e}")


def load_mask_from_txt(file_path):
    try:
        mask = np.loadtxt(file_path, dtype=int)
        print(f"Loaded mask from {file_path}:")
        return mask
    except Exception as e:
        print(f"Error loading file {file_path}: {e}")
        return None


def main():
    shutdown_event = threading.Event()

    def handle_shutdown(sig_num, frame):
        shutdown_event.set()

    signal.signal(signal.SIGINT, handle_shutdown)

    # Load configuration from YAML
    config = load_config_from_yaml("../../config/common_param.yaml")["/**"]["ros__parameters"]

    MASK_NAME = config["MASK_NAME"]
    directory_path = "../../config/masks/" + MASK_NAME
    os.makedirs(directory_path, exist_ok=True)

    NUM_BINS_V = config["NUM_BINS_V"]
    NUM_BINS_STEER = config["NUM_BINS_STEER"]
    NUM_BINS_A = config["NUM_BINS_A"]
    NUM_BINS_STEER_RATE = config["NUM_BINS_ABS_STEER_RATE"]

    V_MIN = config["V_MIN"]
    V_MAX = config["V_MAX"]
    STEER_MIN = config["STEER_MIN"]
    STEER_MAX = config["STEER_MAX"]
    A_MIN = config["A_MIN"]
    A_MAX = config["A_MAX"]
    ABS_STEER_RATE_MIN = config["ABS_STEER_RATE_MIN"]
    ABS_STEER_RATE_MAX = config["ABS_STEER_RATE_MAX"]

    VEL_ACC_THRESHOLD = config["VEL_ACC_THRESHOLD"]
    VEL_STEER_THRESHOLD = config["VEL_STEER_THRESHOLD"]
    VEL_ABS_STEER_RATE_THRESHOLD = config["VEL_ABS_STEER_RATE_THRESHOLD"]

    # Create 3 instances of the selector with different configurations
    mask_velocity_acceleration_path = os.path.join(
        directory_path, f"{MASK_NAME}_Velocity_Acceleration.txt"
    )
    mask_velocity_acceleration = load_mask_from_txt(mask_velocity_acceleration_path)
    selector1 = DataCollectingMaskSelector(
        window_title="Velocity-Acceleration Mask",
        x_min=V_MIN,
        x_max=V_MAX,
        y_min=A_MIN,
        y_max=A_MAX,
        num_bins_x=NUM_BINS_V,
        num_bins_y=NUM_BINS_A,
        default_number=VEL_ACC_THRESHOLD,
        xlabel="Velocity (m/s)",
        ylabel="Acceleration (m/s^2)",
        mask_xy=mask_velocity_acceleration,
    )

    mask_velocity_steering_path = os.path.join(directory_path, f"{MASK_NAME}_Velocity_Steering.txt")
    mask_velocity_steer = load_mask_from_txt(mask_velocity_steering_path)
    selector2 = DataCollectingMaskSelector(
        window_title="Velocity-Steering Mask",
        x_min=V_MIN,
        x_max=V_MAX,
        y_min=STEER_MIN,
        y_max=STEER_MAX,
        num_bins_x=NUM_BINS_V,
        num_bins_y=NUM_BINS_STEER,
        default_number=VEL_STEER_THRESHOLD,
        xlabel="Velocity (m/s)",
        ylabel="Steering Angle (rad)",
        mask_xy=mask_velocity_steer,
    )

    mask_velocity_steering_rate_path = os.path.join(
        directory_path, f"{MASK_NAME}_Velocity_Steering_Rate.txt"
    )
    mask_velocity_steering_rate = load_mask_from_txt(mask_velocity_steering_rate_path)
    selector3 = DataCollectingMaskSelector(
        window_title="Velocity-Steering Rate Mask",
        x_min=V_MIN,
        x_max=V_MAX,
        y_min=ABS_STEER_RATE_MIN,
        y_max=ABS_STEER_RATE_MAX,
        num_bins_x=NUM_BINS_V,
        num_bins_y=NUM_BINS_STEER_RATE,
        default_number=VEL_ABS_STEER_RATE_THRESHOLD,
        xlabel="Velocity (m/s)",
        ylabel="Abs Steering Rate (rad/s)",
        mask_xy=mask_velocity_steering_rate,
    )

    print("Press Ctrl+C to save Masks and exit.")

    # Set window positions
    try:
        selector1.fig.canvas.manager.window.wm_geometry("+100+100")
        selector2.fig.canvas.manager.window.wm_geometry("+800+100")
        selector3.fig.canvas.manager.window.wm_geometry("+1500+100")
    except AttributeError:
        print("Warning: Unable to set window position. Backend may not support it.")

    def update_loop(selector):
        while not shutdown_event.is_set():
            time.sleep(selector.grid_update_time_interval)

    threads = [
        threading.Thread(target=update_loop, args=(selector,), daemon=True)
        for selector in [selector1, selector2, selector3]
    ]
    for thread in threads:
        thread.start()

    try:
        while not shutdown_event.is_set():
            plt.pause(0.01)
    finally:
        np.savetxt(mask_velocity_acceleration_path, selector1.mask_xy, fmt="%d")
        print("Saved Velocity-Acceleration Mask to", mask_velocity_acceleration_path)
        np.savetxt(
            os.path.join(directory_path, f"{MASK_NAME}_Velocity_Steering.txt"),
            selector2.mask_xy,
            fmt="%d",
        )
        print("Saved Velocity-Steering Mask to", mask_velocity_steering_path)
        np.savetxt(
            os.path.join(directory_path, f"{MASK_NAME}_Velocity_Steering_Rate.txt"),
            selector3.mask_xy,
            fmt="%d",
        )
        print("Saved Velocity-Steering Rate Mask to", mask_velocity_steering_rate_path)
        print("Exiting program.")


if __name__ == "__main__":
    main()

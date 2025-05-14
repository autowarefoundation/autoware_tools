import tkinter as tk
from tkinter import ttk

from autoware_planning_msgs.msg import Trajectory
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from plotter import Plotter
from ros2_interface import ROS2Interface
from trajectory_data import get_data_functions


class TkinterApp:
    def __init__(self, root, ros_interface_node: ROS2Interface):
        self.root = root
        self.root.title("Autoware Trajectory Visualizer")
        self.root.geometry("800x600")
        self.ros_interface = ros_interface_node
        self.topics = []

        # Predetermined list for the dropdown
        self.axis_options = get_data_functions()
        self.current_x_axis_selection = tk.StringVar()
        self.current_y_axis_selection = tk.StringVar()

        # --- Main Frames ---
        self.left_frame = ttk.Frame(self.root, padding="10")
        self.left_frame.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)
        self.right_frame = ttk.Frame(self.root, padding="10")
        self.right_frame.grid(row=0, column=1, sticky="nswe", padx=5, pady=5)

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=3)  # Give more weight to the plot area
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
        if self.axis_options:
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
        if self.axis_options:
            self.y_axis_dropdown.current(1)  # Set default selection

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
        self.listbox.bind("<<ListboxSelect>>", lambda event: self.plot(event.widget.curselection()))
        self.listbox_scrollbar_y.config(command=self.listbox.yview)
        self.listbox_scrollbar_x.config(command=self.listbox.xview)

        self.listbox_scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.left_frame.grid_rowconfigure(3, weight=1)  # Allow listbox to expand

        # Button to Refresh List
        self.refresh_button = ttk.Button(
            self.left_frame, text="Refresh List", command=self.refresh_topic_list
        )
        self.refresh_button.grid(row=4, column=0, columnspan=2, padx=5, pady=10, sticky="ew")
        # --- Right Frame Widgets ---
        # Matplotlib Plot Area
        self.plotter = Plotter()
        self.canvas = FigureCanvasTkAgg(self.plotter.fig, master=self.right_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.msg_per_topic = {}
        self.refresh_topic_list()

    def refresh_topic_list(self):
        self.listbox.delete(0, tk.END)  # Clear existing items
        self.topics = self.ros_interface.get_trajectory_topics()
        for topic in self.topics:
            self.listbox.insert(tk.END, topic)

    def update(self, topic, trajectory: Trajectory):
        self.msg_per_topic[topic] = trajectory

    def plot(self, topic_indexes):
        self.ros_interface.remove_callbacks()
        self.msg_per_topic.clear()
        x_axis_selection = self.current_x_axis_selection.get()
        y_axis_selection = self.current_y_axis_selection.get()
        selected_topics = [self.topics[i] for i in topic_indexes]
        self.plotter.init_plot(x_axis_selection, y_axis_selection, selected_topics)
        for topic in selected_topics:
            self.ros_interface.add_callback(
                topic,
                Trajectory,
                lambda msg, captured_topic=topic: self.update(captured_topic, msg),
            )

    def replot(self):
        x_axis_selection = self.current_x_axis_selection.get()
        y_axis_selection = self.current_y_axis_selection.get()
        x_axis_fn = self.axis_options[x_axis_selection]
        y_axis_fn = self.axis_options[y_axis_selection]
        shift_x_data = "Arc Length" in x_axis_selection
        for topic, traj in self.msg_per_topic.items():
            if shift_x_data:
                x_data = x_axis_fn(traj, self.ros_interface.ego_odom)
            else:
                x_data = x_axis_fn(traj)
            y_data = y_axis_fn(traj)
            self.plotter.update_data(topic, x_data, y_data)
        self.plotter.replot()
        self.canvas.draw_idle()


if __name__ == "__main__":
    root = tk.Tk()
    app = TkinterApp(root)
    root.mainloop()

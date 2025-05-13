import matplotlib.pyplot as plt


class Plotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.plots = {}

    def init_plot(self, x_label: str, y_label: str, plot_names: list[str]):
        cmap = "tab10" if len(plot_names) <= 10 else "tab20"
        colormap = plt.cm.get_cmap(cmap, len(plot_names))
        self.ax.set_prop_cycle(color=[colormap(i) for i in range(len(plot_names))])
        self.plots.clear()
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)
        self.ax.legend()
        for name in plot_names:
            (self.plots[name],) = self.ax.plot(
                [], [], marker="o", linestyle="-", label=name, alpha=0.6
            )

    def update_data(self, name, x_data, y_data):
        if name not in self.plots:
            print(f"{name} plot not found")
            return
        self.plots[name].set_data(x_data, y_data)

    def replot(self):
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()

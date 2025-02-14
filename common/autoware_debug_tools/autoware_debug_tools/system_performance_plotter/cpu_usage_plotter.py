#!/usr/bin/env python3

from tier4_debug_msgs.msg import SystemUsageArray

from .system_performance_plotter_base import PREDEFINED_COMPONENT_NAMES
from .system_performance_plotter_base import SystemPerformancePlotterBase
from .system_performance_plotter_base import create_common_argument


class CpuUsagePlotter(SystemPerformancePlotterBase):
    def check_topic(self, topic_name):
        if self.grep_topic_name is not None and self.grep_topic_name not in topic_name:
            return False
        if "/system_usage" not in topic_name:
            return False
        return True

    def update_metrics_func(self, topic_name, data, date_time):
        if not isinstance(data, SystemUsageArray):
            return

        for system_usage in data.system_usage:
            if self.component_name == "all":
                # all
                pass
            elif self.component_name not in PREDEFINED_COMPONENT_NAMES:
                # others (other than PREDEFINED_COMPONENT_NAMES)
                if system_usage.name.split("/")[0] in PREDEFINED_COMPONENT_NAMES:
                    continue
            elif system_usage.name.split("/")[0] != self.component_name:
                # specific component
                continue

            if system_usage.name not in self.stamp_and_metrics:
                self.stamp_and_metrics[system_usage.name] = []
                self.max_metrics[system_usage.name] = 0.0

            cpu_usage = system_usage.cpu_usage
            self.stamp_and_metrics[system_usage.name].append([date_time, cpu_usage])
            self.max_metrics[system_usage.name] = max(
                self.max_metrics[system_usage.name], cpu_usage
            )


def main():
    args = create_common_argument(100)
    plotter = CpuUsagePlotter(args, "CPU Usage [%]", "_cpu_usage")
    plotter.run()


if __name__ == "__main__":
    main()

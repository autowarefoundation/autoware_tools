#!/usr/bin/env python3

from autoware_internal_debug_msgs.msg import Float32Stamped
from autoware_internal_debug_msgs.msg import Float64Stamped
from tier4_debug_msgs.msg import Float32Stamped as Tier4Float32Stamped
from tier4_debug_msgs.msg import Float64Stamped as Tier4Float64Stamped

from .system_performance_plotter_base import PREDEFINED_COMPONENT_NAMES
from .system_performance_plotter_base import SystemPerformancePlotterBase
from .system_performance_plotter_base import create_common_argment


class ProcessingTimePlotter(SystemPerformancePlotterBase):
    def check_topic(self, topic_name):
        if self.grep_topic_name is not None and self.grep_topic_name not in topic_name:
            return False
        if "/processing_time_ms" not in topic_name and "/exe_time_ms" not in topic_name:
            return False

        if self.component_name == "all":
            # all
            pass
        elif self.component_name in PREDEFINED_COMPONENT_NAMES:
            # specific component
            if f"/{self.component_name}/" not in topic_name:
                return False
        else:
            # others (other than PREDEFINED_COMPONENT_NAMES)
            for predefined_component_name in PREDEFINED_COMPONENT_NAMES:
                if f"/{predefined_component_name}/" in topic_name:
                    return False
        return True

    def update_metrics_func(self, topic_name, data, date_time):
        if (
            not isinstance(data, Tier4Float64Stamped)
            and not isinstance(data, Tier4Float32Stamped)
            and not isinstance(data, Float64Stamped)
            and not isinstance(data, Float32Stamped)
        ):
            return

        if topic_name not in self.stamp_and_metrics:
            self.stamp_and_metrics[topic_name] = []
            self.max_metrics[topic_name] = 0.0

        processing_time_ms = data.data
        self.stamp_and_metrics[topic_name].append([date_time, processing_time_ms])
        self.max_metrics[topic_name] = max(self.max_metrics[topic_name], processing_time_ms)


def main():
    args = create_common_argment(100)
    plotter = ProcessingTimePlotter(args, "Processing Time [ms]", "_processing_time")
    plotter.run()


if __name__ == "__main__":
    main()

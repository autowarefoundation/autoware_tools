#!/usr/bin/env python3

import argparse
from datetime import datetime
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .utils import create_reader

plt.rcParams["font.size"] = 8

PREDEFINED_COMPONENT_NAMES = [
    "sensing",
    "localization",
    "perception",
    "planning",
    "control",
    "system",
]
LINESTYLES = ["solid", "dashed"]
NUM_LINESTYLES = len(LINESTYLES)
COLORS = plt.get_cmap("tab20")
NUM_COLORS = len(COLORS.colors)


class SystemPerformancePlotterBase:
    def __init__(self, args, ylabel, output_suffix):
        self.input_bag_dir = args.bag_file
        self.component_name = args.component_name
        self.grep_topic_name = args.grep_topic_name
        self.ymax = args.ymax
        self.number_of_plot = args.number_of_plot
        self.save_result = args.save_result
        self.ylabel = ylabel
        self.output_suffix = output_suffix

        self.stamp_and_metrics = {}
        self.max_metrics = {}

    def run(self):
        # read data from rosbag
        reader = create_reader(self.input_bag_dir)
        topic_type_list = {}
        for topic_type in reader.get_all_topics_and_types():
            topic_type_list[topic_type.name] = topic_type.type
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            if not self.check_topic(topic_name):
                continue

            data = deserialize_message(msg, get_message(topic_type_list[topic_name]))

            to_nanosec = 1e-9
            time_stamp = stamp * to_nanosec
            date_time = datetime.fromtimestamp(time_stamp)

            self.update_metrics_func(topic_name, data, date_time)

        # sort stamp_and_metrics by alphabetical order
        self.stamp_and_metrics = dict(sorted(self.stamp_and_metrics.items(), key=lambda x: x[0]))

        # pick up top N modules and sort by metrics if necessary
        if self.number_of_plot:
            sorted_max_metrics = dict(
                sorted(self.max_metrics.items(), key=lambda x: x[1], reverse=True)
            )

            tmp_stamp_and_metrics = {}
            for idx, key in enumerate(sorted_max_metrics):
                if idx < self.number_of_plot:
                    tmp_stamp_and_metrics[key] = self.stamp_and_metrics[key]

            self.stamp_and_metrics = tmp_stamp_and_metrics

        # initialize report data
        self.report_data = {}
        for n in self.stamp_and_metrics:
            self.report_data[n] = []

        # plot and output to csv
        fig = plt.figure(figsize=(10, 8), tight_layout=True)
        plt.subplots_adjust(left=0.08, right=0.97, bottom=0.08, top=0.97, wspace=0.5, hspace=0.1)

        ax = fig.add_subplot(1, 1, 1)
        ax.set_title(
            self.ylabel
            + (
                "   (sorted by metrics)"
                if self.number_of_plot
                else "   (sorted by alphabetical order)"
            )
        )
        ax.grid(which="major", axis="both", color="black", alpha=0.4, linestyle="-", linewidth=1)
        ax.grid(
            which="minor",
            axis="both",
            color="black",
            alpha=0.2,
            linestyle="--",
            linewidth=1,
        )
        ax.set_xlabel("time [s]", fontsize=10)
        ax.set_ylabel(self.ylabel, fontsize=10)

        merge_df1 = pd.DataFrame()
        merge_df2 = pd.DataFrame()
        for idx, name in enumerate(self.stamp_and_metrics):
            raw_data_arr = np.array(self.stamp_and_metrics[name])
            if len(raw_data_arr) == 0:
                print("no topics found in" + name)
                continue
            stamp = raw_data_arr[:, 0]
            rate = raw_data_arr[:, 1]
            color = COLORS(idx % NUM_COLORS)
            linestyle = LINESTYLES[(idx // NUM_COLORS) % NUM_LINESTYLES]
            ax.plot(stamp, rate, label=name, color=color, linestyle=linestyle)

            self.report_data[name].append(np.round(rate.mean(), 3))
            self.report_data[name].append(np.round(rate.std(), 3))
            self.report_data[name].append(np.round(rate.max(), 3))
            self.report_data[name].append(np.round(np.percentile(rate, 99), 3))
            report_arr = np.array(self.report_data[name])

            try:
                col_name = name.replace("/", "_")
                tmp_df1 = pd.DataFrame(raw_data_arr, columns=["DateTime", col_name])
                tmp_df1["DateTime"] = pd.to_datetime(tmp_df1["DateTime"])
                tmp_df1.set_index("DateTime", inplace=True)
                merge_df1 = pd.concat([merge_df1, tmp_df1], axis=1)

                tmp_df2 = pd.DataFrame(
                    report_arr,
                    columns=[name],
                    index=["ave [ms]", "std [ms]", "max [ms]", "99percentile [ms]"],
                )
                merge_df2 = pd.concat([merge_df2, tmp_df2], axis=1)

            except pd.errors.ParserError as e:
                print(f"Error parsing data: {e}")
            except pd.errors.EmptyDataError as e:
                print(f"Error: Empty data encountered: {e}")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")

        if self.ymax:
            plt.ylim(0.0, self.ymax)
        plt.legend(loc="upper left")

        if self.save_result:
            timestamp = datetime.now().strftime("%y-%m-%d-%H-%M-%S")
            output_name = f"{self.component_name}{self.output_suffix}"

            os.makedirs("./result", exist_ok=True) if not os.path.isdir("./result") else None

            # save csv
            try:
                merge_df1.reset_index(inplace=True)
                merge_df1.to_csv(f"./result/{output_name}-{timestamp}-raw-data.csv", index=True)
                merge_df2.to_csv(f"./result/{output_name}-{timestamp}-report.csv", index=True)
            except FileNotFoundError as e:
                print(f"Error: File not found: {e}")
            except PermissionError as e:
                print(f"Error: Permission denied: {e}")
            except pd.errors.EmptyDataError as e:
                print(f"Error: Empty data encountered: {e}")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")

            # save figure of plot
            plt.savefig(f"./result/{output_name}-{timestamp}.png")

        plt.show()


def create_common_argment(ymax=None):
    parser = argparse.ArgumentParser(description="report system performance from rosbag.")
    parser.add_argument("bag_file", help="input bagfile")
    parser.add_argument("-c", "--component-name", default="all", type=str, help="component name")
    parser.add_argument("-g", "--grep-topic-name", default=None, type=str, help="target topic name")
    parser.add_argument("-y", "--ymax", default=ymax, type=int, help="ymax of plot")
    parser.add_argument(
        "-n",
        "--number-of-plot",
        default=None,
        type=int,
        help="number of plot of top N highest metrics",
    )
    parser.add_argument(
        "-s", "--save-result", default=False, action="store_true", help="whether to save result"
    )

    args = parser.parse_args()

    if args.component_name not in PREDEFINED_COMPONENT_NAMES + ["others", "all"]:
        raise ValueError(f"Component {args.component} is not correct.")
    return args

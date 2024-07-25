#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

import os
import signal
import sys
import threading

import psutil
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import SystemUsage
from tier4_debug_msgs.msg import SystemUsageArray


def signal_handler(sig, frame):
    sys.exit(0)


def get_system_usage(pid, system_usages, interval):
    try:
        proc = psutil.Process(pid)
        cpu_usage = proc.cpu_percent(interval=interval)
        memory_usage = proc.memory_info().rss
        cmdline = proc.cmdline()
        process_name = " ".join(cmdline)
        component = (
            process_name.split("__ns:=/")[1].split("/")[0].split(" ")[0]
            if "__ns:=/" in process_name
            else ""
        )
        if component == "":
            component = "others"
        container = process_name.split("__node:=", 1)[1].split(" ")[0]
        system_usages[pid] = {
            "component": component,
            "container": container,
            "cpu_usage": cpu_usage,
            "memory_usage": memory_usage,
        }
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess, IndexError):
        pass


def print_system_usage(sorted_system_usages):
    # clear terminal
    os.system("clear")
    if not sorted_system_usages:
        print("No processes found with the specified name.")
        return

    print("|" + "-" * 202 + "|")
    print(
        "|"
        + "\033[1m"
        + "Process Information".center(72, " ")
        + "\033[0m"
        + "|"
        + "\033[1m"
        + "CPU Usage".center(62, " ")
        + "\033[0m"
        + "|"
        + "\033[1m"
        + "Memory Usage".center(66, " ")
        + "\033[0m"
        + "|"
    )
    print("|" + "-" * 202 + "|")

    last_component = None
    for pid, data in sorted_system_usages:
        component = data["component"]
        container = data["container"]
        cpu_usage = data["cpu_usage"]
        memory_usage = data["memory_usage"] / 1024**2
        cpu_bar = "#" * int(cpu_usage * 0.5)
        memory_bar = "#" * int(memory_usage * 0.06)

        if last_component and last_component != component:
            print(
                "|"
                + "-" * 16
                + "|"
                + "-" * 55
                + "|"
                + "-" * 9
                + "|"
                + "-" * 52
                + "|"
                + "-" * 13
                + "|"
                + "-" * 52
                + "|"
            )

        last_component = component
        process_info = f"| {component.split('/')[-1].ljust(14)} | {container.ljust(53)} | {cpu_usage:4.1f}[%] | {cpu_bar:<50} | {memory_usage:6.1f}[MiB] | {memory_bar:<50} |"
        print(process_info)

    print("|" + "-" * 202 + "|")


def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    node = Node("system_usage_monitor")

    pub_system_usage = node.create_publisher(
        SystemUsageArray,
        "~/system_usage",
        1,
    )

    system_usages = {}
    while True:
        # create thread to calculate cpu usage of each process since it takes time
        process_name_keyword = "__node:="
        threads = []
        for proc in psutil.process_iter(["pid", "name", "cpu_percent", "cmdline"]):
            try:
                if process_name_keyword in " ".join(proc.info["cmdline"]):
                    pid = proc.info["pid"]
                    thread = threading.Thread(
                        target=get_system_usage, args=(pid, system_usages, 1.0)
                    )
                    threads.append(thread)
                    thread.start()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

        # wait for all the thread to finish
        for thread in threads:
            thread.join()

        # sort in the order of component name
        sorted_system_usages = sorted(
            system_usages.items(), key=lambda x: x[1]["component"] + x[1]["container"]
        )

        # print system usage
        print_system_usage(sorted_system_usages)

        # publish system usage
        system_usage_array = SystemUsageArray()
        system_usage_array.stamp = node.get_clock().now().to_msg()
        for pid, data in sorted_system_usages:
            system_usage = SystemUsage()
            system_usage.pid = pid
            system_usage.name = data["component"] + "/" + data["container"]
            system_usage.cpu_usage = data["cpu_usage"]
            system_usage.memory_usage = float(data["memory_usage"])
            system_usage_array.system_usage.append(system_usage)
        pub_system_usage.publish(system_usage_array)


if __name__ == "__main__":
    main()

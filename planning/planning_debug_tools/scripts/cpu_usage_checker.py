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
import threading

import psutil


def get_cpu_usage(pid, cpu_usages, interval):
    try:
        proc = psutil.Process(pid)
        cpu_usage = proc.cpu_percent(interval=interval)
        cmdline = proc.cmdline()
        process_name = " ".join(cmdline)
        component = process_name.split("__ns:=/")[1].split("/")[0].split(" ")[0]
        if component == "":
            component = "others"
        container = process_name.split("__node:=", 1)[1].split(" ")[0]
        cpu_usages[pid] = {"component": component, "container": container, "usage": cpu_usage}
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess, IndexError):
        pass


def print_cpu_usage(cpu_usages):
    # clear terminal
    os.system("clear")
    if not cpu_usages:
        print("No processes found with the specified name.")
        return

    # sort in the order of component name
    sorted_cpu_usages = sorted(cpu_usages.items(), key=lambda x: x[1]["component"])

    print(" CPU Usage")
    print("-" * 185)

    last_component = None
    for pid, data in sorted_cpu_usages:
        component = data["component"]
        container = data["container"]
        usage = data["usage"]
        bar = "#" * int(usage)

        if last_component and last_component != component:
            print("|" + "-" * 17 + "|" + "-" * 57 + "|" + "-" * 7 + "|" + "-" * 100)

        last_component = component
        process_info = f"| {component.split('/')[-1].ljust(15)} | {container.ljust(55)} | {str(usage).ljust(4)}% | {bar}"
        print(process_info)

    print("-" * 185)


def main():
    cpu_usages = {}
    while True:
        # create thread to calculate cpu usage of each process since it takes time
        process_name_keyword = "__node:="
        threads = []
        for proc in psutil.process_iter(["pid", "name", "cpu_percent", "cmdline"]):
            try:
                if process_name_keyword in " ".join(proc.info["cmdline"]):
                    pid = proc.info["pid"]
                    thread = threading.Thread(target=get_cpu_usage, args=(pid, cpu_usages, 1.0))
                    threads.append(thread)
                    thread.start()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

        # wait for all the thread to finish
        for thread in threads:
            thread.join()

        # print cpu usage
        print_cpu_usage(cpu_usages)


if __name__ == "__main__":
    main()

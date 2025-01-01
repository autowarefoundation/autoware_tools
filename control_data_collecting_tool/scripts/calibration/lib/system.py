#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import subprocess
import sys


def get_active_nodes():
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        # Split the output into lines and return as a list
        return result.stdout.strip().split("\n")
    except subprocess.CalledProcessError as e:
        print(f"Error while running ros2 node list: {e.stderr}")
        return []


def check_node_active(node_list):
    print("Check if necessary nodes are activated.")
    active_nodes = get_active_nodes()
    is_all_nodes_active = True

    for node in node_list:
        if node not in active_nodes:
            print(f"Node {node} is inactive.")
            is_all_nodes_active = False

    if is_all_nodes_active:
        print("All nodes are activated.")
    else:
        print("Some nodes are inactive.")
        sys.exit(1)


def check_service_active(service_name):
    print(f"Check if the service '{service_name}' is active.")
    try:
        result = subprocess.run(
            ["systemctl", "status", service_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if result.returncode == 4:
            print(f"The service '{service_name}' was not found. Proceeding as OK.")
            return

        output = result.stdout
        if "Active: active (running)" in output:
            print(f"The service '{service_name}' is active and running. Exiting script.")
            sys.exit(1)
        else:
            print(f"The service '{service_name}' is NOT active.")
    except FileNotFoundError:
        print("The systemctl command is not available on this system.")
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

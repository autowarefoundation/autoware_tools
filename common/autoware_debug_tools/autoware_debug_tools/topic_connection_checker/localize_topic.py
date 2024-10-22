#!/usr/bin/env python3

<<<<<<< HEAD
import os
import re
import argparse
from typing import List, Dict

def find_topics_in_file(file_path: str, topics: List[str]) -> Dict[str, List[int]]:
    """
    Search for topics in a given file and return their line numbers.
    """
    topic_lines = {topic: [] for topic in topics}
    
    with open(file_path, 'r', encoding='utf-8') as file:
=======
import argparse
import os
from typing import Dict
from typing import List

from autoware_debug_tools.topic_connection_checker.launch_file_analyse import (
    launch_file_analyse_main,
)
from autoware_debug_tools.topic_connection_checker.launch_file_analyse import LaunchTree
from autoware_debug_tools.topic_connection_checker.launch_file_analyse import LaunchTreeNode
from autoware_debug_tools.topic_connection_checker.launch_file_analyse import find_cmake_projects
from autoware_debug_tools.topic_connection_checker.launch_file_analyse import find_package


def find_topics_in_file(file_path: str, topics: List[str]) -> Dict[str, List[int]]:
    """Search for topics in a given file and return their line numbers."""
    topic_lines = {topic: [] for topic in topics}

    with open(file_path, "r", encoding="utf-8") as file:
>>>>>>> ea4c35cdcf8e77d60d144306e242801446167729
        for line_num, line in enumerate(file, 1):
            for topic in topics:
                if topic in line:
                    topic_lines[topic].append(line_num)
<<<<<<< HEAD
    
    return {topic: lines for topic, lines in topic_lines.items() if lines}

def search_topics_in_directory(directory: str, topics: List[str]) -> Dict[str, Dict[str, List[int]]]:
    """
    Search for topics in all relevant files within the given directory and its subdirectories.
    """
    results = {}
    
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.hpp', '.h', 'launch.py')):
=======

    return {topic: lines for topic, lines in topic_lines.items() if lines}


def search_topics_in_directory(
    directory: str, topics: List[str]
) -> Dict[str, Dict[str, List[int]]]:
    """Search for topics in all relevant files within the given directory and its subdirectories."""
    results = {}

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith((".cpp", ".hpp", ".h", "launch.py")):
>>>>>>> ea4c35cdcf8e77d60d144306e242801446167729
                file_path = os.path.join(root, file)
                file_results = find_topics_in_file(file_path, topics)
                if file_results:
                    results[file_path] = file_results
<<<<<<< HEAD
    
    return results

def print_results(results: Dict[str, Dict[str, List[int]]]):
    """
    Print the search results in a formatted manner.
    """
=======

    return results


def search_topics_in_launch_tree(tree: LaunchTree, topics: List[str]) -> Dict[str, List[str]]:
    results = {}

    for node_name in tree.nodes_manager:
        node: LaunchTreeNode = tree.nodes_manager[node_name]
        name = node.name
        results[name] = {topic: [] for topic in topics}
        value_found = False

        if "__remapping__" in node.parameters:
            for remap in node.parameters["__remapping__"]:
                value = node.parameters["__remapping__"][remap]
                if value in topics:
                    if "input" in remap:
                        # it means that the topic is a subscriber; we aim to find the publisher
                        continue
                    results[name][value] = remap
                    value_found = True

        for param in node.parameters:
            value = node.parameters[param]
            if value in topics:
                if "input" in param:
                    # it means that the topic is a subscriber; we aim to find the publisher
                    continue
                results[name][value] = param
                value_found = True

        if value_found:
            if name.endswith(".launch.xml"):
                # topic name found in launch parameters
                print(f"\nFile: {node.parameters['path']}")
                for topic, param in results[name].items():
                    if len(param) == 0:
                        continue
                    print(f"  Topic '{topic}' found as parameter(s): {param}")

            else:
                # topic name found in node's parameters or remapping
                print(f"\nNode: {name}")
                for topic, param in results[name].items():
                    if len(param) == 0:
                        continue
                    print(f"  Topic '{topic}' found as parameter(s): {param}")


def print_results(results: Dict[str, Dict[str, List[int]]]):
    """Print the search results in a formatted manner."""
>>>>>>> ea4c35cdcf8e77d60d144306e242801446167729
    for file_path, file_results in results.items():
        print(f"\nFile: {file_path}")
        for topic, lines in file_results.items():
            print(f"  Topic '{topic}' found on line(s): {', '.join(map(str, lines))}")

<<<<<<< HEAD
def main():
    parser = argparse.ArgumentParser(description="Search for ROS topics in source files.")
    parser.add_argument("directory", help="The directory to search in")
    parser.add_argument("topics", help="Comma-separated list of topics to search for")
    
    args = parser.parse_args()
    
    directory = args.directory
    topics = [topic.strip() for topic in args.topics.split(',')]
    
    print(f"Searching for topics: {', '.join(topics)}")
    print(f"In directory: {directory}")
    
    results = search_topics_in_directory(directory, topics)
    
=======

def main():
    parser = argparse.ArgumentParser(description="Search for ROS topics in source files.")
    parser.add_argument("directory", help="The directory to search in", default=".")
    parser.add_argument("topics", help="Comma-separated list of topics to search for")

    args = parser.parse_args()

    directory = args.directory
    topics = [topic.strip() for topic in args.topics.split(",")]

    print(f"Searching for topics: {', '.join(topics)}")

    find_cmake_projects(directory)
    autoware_launch_directory = find_package("autoware_launch")

    autoware_launch_context_tree: LaunchTree = launch_file_analyse_main(
        os.path.join(autoware_launch_directory, "launch", "autoware.launch.xml"),
        context={"map_path": "dummy_map"},
    )["__tree__"]

    # Search in CPP, HPP, H, and python launch files
    results = search_topics_in_directory(directory, topics)

>>>>>>> ea4c35cdcf8e77d60d144306e242801446167729
    if results:
        print("\nResults:")
        print_results(results)
    else:
<<<<<<< HEAD
        print("\nNo matching topics found in the specified directory.")

if __name__ == "__main__":
    main()
=======
        print("\nNo matching topics found in the codes in the specified directory.")

    # Search in launch files
    search_topics_in_launch_tree(autoware_launch_context_tree, topics)


if __name__ == "__main__":
    main()
>>>>>>> ea4c35cdcf8e77d60d144306e242801446167729

#!/usr/bin/env python3

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
        for line_num, line in enumerate(file, 1):
            for topic in topics:
                if topic in line:
                    topic_lines[topic].append(line_num)
    
    return {topic: lines for topic, lines in topic_lines.items() if lines}

def search_topics_in_directory(directory: str, topics: List[str]) -> Dict[str, Dict[str, List[int]]]:
    """
    Search for topics in all relevant files within the given directory and its subdirectories.
    """
    results = {}
    
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(('.cpp', '.hpp', '.h', 'launch.py')):
                file_path = os.path.join(root, file)
                file_results = find_topics_in_file(file_path, topics)
                if file_results:
                    results[file_path] = file_results
    
    return results

def print_results(results: Dict[str, Dict[str, List[int]]]):
    """
    Print the search results in a formatted manner.
    """
    for file_path, file_results in results.items():
        print(f"\nFile: {file_path}")
        for topic, lines in file_results.items():
            print(f"  Topic '{topic}' found on line(s): {', '.join(map(str, lines))}")

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
    
    if results:
        print("\nResults:")
        print_results(results)
    else:
        print("\nNo matching topics found in the specified directory.")

if __name__ == "__main__":
    main()
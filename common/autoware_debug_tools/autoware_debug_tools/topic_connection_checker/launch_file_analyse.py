## Total Structure
"""
String Utils -> Launch Tree -> Launch Node Utils -> Launch Analyzer
"""

#### String Utils

import os
import re

patterns = {
    "var": r"\$\((var) ([^\)]+)\)",
    "env": r"\$\((env) ([^\s]+)(?:\s+([^\)]+))?\)",
    "eval": r"\$\((eval) ([^\)]+)\)",
    "find-pkg-share": r"\$\((find-pkg-share) ([^\)]+)\)",
}

BASE_PROJECT_MAPPING = {}
FLAG_CHECKING_SYSTEM_PROJECTS = False

def find_package(package_name) -> str:
    """
    Return the share directory of the given package.
    """
    if package_name in BASE_PROJECT_MAPPING:
        return BASE_PROJECT_MAPPING[package_name]
    else:
        if FLAG_CHECKING_SYSTEM_PROJECTS:
            from ament_index_python.packages import get_package_share_directory
            BASE_PROJECT_MAPPING[package_name] = get_package_share_directory(package_name)
        else:
            BASE_PROJECT_MAPPING[package_name] = f"/opt/ros/humble/share/{package_name}" # use this for temporal solution;
        return BASE_PROJECT_MAPPING[package_name]


def clean_eval_variables(string: str) -> str:
    """Remove quotes and spaces from a string, to obtain the 'value' of a variable."""
    string = string.replace("\\", "")
    if string.startswith('"') and string.endswith('"'):
        return string[1:-1]
    elif string.startswith("'") and string.endswith("'"):
        return string[1:-1]
    else:
        return string


def analyze_eval_string(input_string: str) -> str:
    """Evaluate the expression in the $(eval ...) tag."""
    list_of_strings = input_string.split(" ")
    if list_of_strings[0] == "$(eval":
        expression = " ".join(list_of_strings[1:])[:-1]  # remove the last ')'
        expression = clean_eval_variables(expression)
        result = str(eval(expression))  # remove the outer quotes
    else:
        result = input_string
    return result


def analyze_string(
    input_string: str, context: dict, local_context: dict, base_namespace: str
) -> str:
    """Resolve substitutions recursively in a given string.

    Args:
    context: The arguments and variables context of the current XML file, which is defined by the arg tag and will be passed to the included file
    local_context: The local variable context of the current XML file, which is defined by the let tag
    base_namespace: The current namespace of the XML file

    Returns:
    The string with all substitutions resolved.
    """

    def replace_match(match):
        # Determine type and execute corresponding logic
        if match.group(1) == "var":
            variable_name = analyze_string(
                match.group(2), context, local_context, base_namespace
            )  # Recursively resolve inner substitutions
            # Check if the variable is in the local context
            var_value = local_context.get(variable_name, None)
            if var_value is None:
                # Check if the variable is in the global context
                var_value = context.get(variable_name)
            return var_value
        elif match.group(1) == "env":
            var_name = analyze_string(
                match.group(2), context, local_context, base_namespace
            )  # Recursively resolve inner substitutions
            default_value = analyze_string(
                match.group(3) if match.group(3) is not None else "",
                context,
                local_context,
                base_namespace,
            )
            return os.getenv(var_name, default_value)

        elif match.group(1) == "find-pkg-share":
            package_name = analyze_string(
                match.group(2), context, local_context, base_namespace
            )  # Recursively resolve inner substitutions
            package_dir = find_package(package_name)
            return package_dir

        return ""

    # Loop to ensure all substitutions are resolved
    for key, pattern in patterns.items():
        """
        1. Solve all variables.
        2. Solve all environment variables.
        3. Solve all eval expressions.
        4. Solve all find-pkg-share expressions.
        """
        if key == "eval":
            input_string = analyze_eval_string(input_string)
        else:
            while True:
                old_string = input_string
                input_string = re.sub(pattern, replace_match, input_string)
                # Stop if no more changes are made
                if input_string == old_string:
                    break
    # solve for "\" in the string
    input_string = input_string.replace("\\", "")

    return input_string


def find_linked_path(path: str) -> str:
    """Find the linked path of a given path. If the path is not a link, return the path itself."""
    if os.path.islink(path):
        linked_path = os.readlink(path)
        return linked_path
    else:
        return path


#### Launch Tree

import json
from typing import List


class LaunchTreeNode:
    """Each node in the launch tree is a LaunchTreeNode. It represents a launch file or a ros node."""

    def __init__(self, name: str, **kwargs):
        self.name = name
        self.children: List[LaunchTreeNode] = []
        self.parameters = kwargs

    def add_child(self, child: "LaunchTreeNode"):
        self.children.append(child)

    def jsonify(self):
        return dict(
            name=self.name,
            children=[child.jsonify() for child in self.children],
            parameters=self.parameters,
        )


class LaunchTree:
    """Tree Structure to store the launch file structure."""

    def __init__(self):
        self.root = None
        self.edges_manager = []
        self.nodes_manager = {}

    def get_node(self, node_name):
        return self.nodes_manager[node_name]

    def add_root(self, root_name, **kwargs):
        if self.root is None:
            self.root = LaunchTreeNode(root_name)
            self.nodes_manager[root_name] = self.root
        else:
            print("Root already exists")

    def add_child(self, parent_name, child_name, **kwargs):
        if self.root is None:
            self.root = LaunchTreeNode(parent_name)
            self.nodes_manager[parent_name] = self.root

        if parent_name not in self.nodes_manager:
            # print(f"Parent node {parent_name} not found")
            return

        if child_name in self.nodes_manager:
            # print(f"Child node {child_name} already exists")
            return

        child = LaunchTreeNode(child_name, **kwargs)
        self.nodes_manager[child_name] = child
        self.nodes_manager[parent_name].add_child(child)
        self.edges_manager.append((parent_name, child_name))

    def add_argument(self, node_name, argument_name, argument_value):
        if node_name not in self.nodes_manager:
            print(f"Node {node_name} not found")
            return

        self.nodes_manager[node_name].arguments[argument_name] = argument_value

    def jsonify(self):
        json_object = self.root.jsonify()
        return json_object

    def __repr__(self) -> str:
        json_object = self.jsonify()
        return json.dumps(json_object, indent=4)


def find_unset_parameters(tree: LaunchTree):
    """Find all unset parameters in the launch tree."""
    unset_parameters = []
    for node_name, node in tree.nodes_manager.items():
        for parameter_name, parameter_value in node.parameters.items():
            if parameter_value == "ERROR_PARAMETER_NOT_SET":
                unset_parameters.append((node_name, parameter_name))
    return unset_parameters


#### Launch Node Utils

import xml.etree.ElementTree as ET

import yaml


def read_ros_yaml(file_path: str) -> dict:
    """Read and return the contents of a YAML file."""
    with open(file_path, "r") as file:
        # Using safe_load() to avoid potential security risks
        data = yaml.safe_load(file)

    data = data["/**"]["ros__parameters"]
    return data


def parse_node_tag(
    node_tag: ET.Element, base_namespace: str, context: dict, local_context: dict
):
    pkg = analyze_string(node_tag.get("pkg"), context, local_context, base_namespace)
    exec = analyze_string(node_tag.get("exec"), context, local_context, base_namespace)
    local_parameters = {}
    local_parameters["__param_files"] = []
    # print(context, base_namespace)
    for child in node_tag:
        if child.tag == "param":
            if child.get("name") is not None:
                local_parameters[child.get("name")] = analyze_string(
                    child.get("value"), context, local_context, base_namespace
                )
            if child.get("from") is not None:
                path = analyze_string(
                    child.get("from"), context, local_context, base_namespace
                )
                path = find_linked_path(path)
                if path.endswith("_empty.param.yaml"):
                    continue
                # print(path, child.get("from"))
                local_parameters["__param_files"].append(path)
                if path == "":
                    print("-----Node Parameter not Found------")
                    print(f"----package: {pkg}-----")
                    print(f"----exec: {exec}-----")
                    print(f"----parameter string: {child.get('from')}-----")
                else:
                    data = read_ros_yaml(path)
                    for key in data:
                        if isinstance(data[key], str) and data[key].startswith("$(var "):
                            local_parameters[key] = analyze_string(
                                data[key], context, local_context, base_namespace
                            )
                        else:
                            local_parameters[key] = data[key]
    context["__tree__"].add_child(
        context["__current_launch_name_"], f"{pkg}/{exec}", **local_parameters
    )


#### Launch Analyzer

import os
import xml.etree.ElementTree as ET
from copy import deepcopy
from typing import Any, Dict, List, Optional

import os


def find_cmake_projects(root_dir):
    for dirpath, _, filenames in os.walk(root_dir):
        if "CMakeLists.txt" in filenames:
            cmake_file_path = os.path.join(dirpath, "CMakeLists.txt")
            with open(cmake_file_path, "r") as file:
                for line in file:
                    if line.startswith("project("):
                        # Extract the project name
                        project_name = line.split("(")[1].split(")")[0].strip()
                        BASE_PROJECT_MAPPING[project_name] = dirpath
                        break


def check_if_run(tag: ET.Element, base_name: dict, context: dict, local_context: dict):
    """Many tag has a if and unless attribute, this function checks if the tag should be run or not."""
    if tag.get("if"):
        if_value = analyze_string(tag.get("if"), context, local_context, base_name)
        if_value = if_value.lower() == "true"
        if not if_value:
            return False
    if tag.get("unless"):
        unless_value = analyze_string(
            tag.get("unless"), context, local_context, base_name
        )
        unless_value = unless_value.lower() == "true"
        if unless_value:
            return False
    return True


def copy_context(context: dict):
    new_context = dict()
    for key in context:
        new_context[key] = context[key]
    return new_context


def process_include_tag(
    include_tag: ET.Element,
    context: dict,
    local_context: dict,
    base_namespace: str,
    group_base_namespace: Optional[str] = None,
):
    """Process the include tag, which includes another XML file.

    include_tag: The include XML node tag to process
    context: The arguments and variables context of the current XML file, which is defined by the arg tag and will be passed to the included file
    local_context: The local variable context of the current XML file, which is defined by the let tag
    base_namespace: The current namespace of the XML file
    group_base_namespace: The namespace of the current group tag (affect the namespace of the included file)
    """
    if group_base_namespace is None:
        group_base_namespace = base_namespace
    included_file = include_tag.get("file")
    included_file = analyze_string(
        included_file, context, local_context, base_namespace
    )
    included_file = find_linked_path(included_file)
    temp_context = copy_context(context)
    argument_dict = dict()
    for child in include_tag:
        if child.tag == "arg":
            value = analyze_string(
                child.get("value"), temp_context, local_context, base_namespace
            )
            name = analyze_string(
                child.get("name"),
                temp_context,
                local_context,
                base_namespace,
            )
            temp_context[name] = (
                value  # temp_context is used to pass arguments to the included file and updated on the fly for each argument
            )
    for key in argument_dict:
        temp_context[key] = argument_dict[key]
    if included_file:
        context["__tree__"].add_child(
            context["__current_launch_name_"],
            os.path.basename(included_file),
            path=included_file,
        )
        if included_file.startswith("/opt/ros/humble") and (not FLAG_CHECKING_SYSTEM_PROJECTS):
            # not checking system projects if the file is in /opt/ros/humble and FLAG_CHECKING_SYSTEM_PROJECTS is False
            pass
        elif included_file.endswith(".launch.xml"):
            # check if the file is a launch file
            return parse_xml(included_file, group_base_namespace, temp_context)
    return context


def parse_argument_tag(
    argument_tag: ET.Element, base_namespace: str, context: dict, local_context: dict
):
    # argument_name = os.path.join(base_namespace, argument_tag.get("name"))
    argument_name = argument_tag.get("name")
    if argument_tag.get("default"):
        if argument_name not in context:
            value = analyze_string(
                argument_tag.get("default"), context, local_context, base_namespace
            )
            context["__tree__"].get_node(context["__current_launch_name_"]).parameters[
                argument_name
            ] = value
            context[argument_name] = value
    else:
        if argument_name not in context:
            context["__tree__"].get_node(context["__current_launch_name_"]).parameters[
                argument_name
            ] = "ERROR_PARAMETER_NOT_SET"
            # print(f"Argument {argument_name} has no default value and is not set in context")
        else:
            context["__tree__"].get_node(context["__current_launch_name_"]).parameters[
                argument_name
            ] = context[argument_name]
    return context


def parse_let_tag(
    let_tag: ET.Element, base_namespace: str, context: dict, local_context: dict
):
    argument_name = let_tag.get("name")
    if let_tag.get("value"):
        local_context[argument_name] = analyze_string(
            let_tag.get("value"), context, local_context, base_namespace
        )
    return context


def parse_group_tag(
    group_tag: ET.Element,
    base_namespace: str,
    context: dict,
    local_context: dict,
    parent_file_space: Optional[str] = None,
):
    if parent_file_space is None:
        parent_file_space = base_namespace
    # find the push-ros-namespace tag inside the children
    group_base_namespace = deepcopy(base_namespace)
    for child in group_tag:
        if child.tag == "push-ros-namespace":
            if child.get("namespace").strip() == "/":
                continue
            group_base_namespace = (
                f"{base_namespace}/{child.get('namespace').strip('/')}"
            )
            # print(f"Setting ROS namespace to {group_base_namespace} inside group")

    # find all other children
    for child in group_tag:
        process_tag(
            child,
            base_namespace,
            context,
            local_context,
            group_base_namespace=group_base_namespace,
        )

    # if group_base_namespace != base_namespace:
    #     print(f"Exiting group with namespace {group_base_namespace}")
    return context


def process_tag(
    tag: ET.Element,
    base_namespace: str,
    context: dict,
    local_context: dict,
    group_base_namespace: Optional[str] = None,
):
    if group_base_namespace is None:
        group_base_namespace = base_namespace
    if not check_if_run(tag, base_namespace, context, local_context):
        return context

    if tag.tag == "arg":
        context = parse_argument_tag(tag, base_namespace, context, local_context)
    elif tag.tag == "let":
        context = parse_let_tag(tag, base_namespace, context, local_context)
    elif tag.tag == "group":
        context = parse_group_tag(tag, base_namespace, context, local_context)
    elif tag.tag == "include":
        context = process_include_tag(
            tag, context, local_context, base_namespace, group_base_namespace
        )
    elif tag.tag == "node":
        context = parse_node_tag(tag, base_namespace, context, local_context)
    return context


def parse_xml(file_path: str, namespace: str = "", context: dict = {}):
    """Recursively parse XML files, handling <include> tags. For each file, the namespace should be the same"""
    full_path = os.path.join(file_path)
    context["__current_launch_file__"] = full_path
    context["__current_launch_name_"] = os.path.basename(full_path)
    if "__tree__" not in context:
        context["__tree__"] = LaunchTree()
    if context["__tree__"].root is None:
        context["__tree__"].add_root(context["__current_launch_name_"], path=full_path)
    tree = ET.parse(full_path)
    root = tree.getroot()

    # Process each node in the XML
    local_context = dict()
    for tag in root:
        process_tag(tag, namespace, context, local_context)
    return context


def launch_file_analyse_main(launch_file, context={}, src_dir=None):
    if src_dir:
        find_cmake_projects(src_dir)
    context = parse_xml(launch_file, context=context)
    with open("output.json", "w") as f:
       f.write(str(context["__tree__"]))
    # print unused parameters
    unset_parameters = find_unset_parameters(context["__tree__"])
    if len(unset_parameters) > 0:
        for unset_parameter in unset_parameters:
            print(unset_parameter)
        raise Exception(f"Some parameters are not set; {unset_parameters}")
    return context


if __name__ == "__main__":
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument("--src_dir", type=str, default="src")
    args.add_argument("--launch_file", type=str, required=True)
    args.add_argument('--flag_check_system_file', action='store_true')
    # context dictionary
    args.add_argument('--context', '--parameters', nargs='+', help='Key=value pairs')

    args = args.parse_args()
    src_dir = args.src_dir
    launch_file = args.launch_file
    FLAG_CHECKING_SYSTEM_PROJECTS = args.flag_check_system_file
    context = {}
    for param in args.context:
        key, value = param.split('=')
        context[key] = value


    launch_file_analyse_main(launch_file, context=context, src_dir=src_dir)
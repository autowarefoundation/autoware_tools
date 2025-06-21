from copy import deepcopy
import os
from typing import Optional
import xml.etree.ElementTree as ET

from autoware_debug_tools.topic_connection_checker.launch_file_analyse.launch_tree import LaunchTree
from autoware_debug_tools.topic_connection_checker.launch_file_analyse.string_utils import (
    FLAG_CHECKING_SYSTEM_PROJECTS,
)
from autoware_debug_tools.topic_connection_checker.launch_file_analyse.string_utils import (
    analyze_string,
)
from autoware_debug_tools.topic_connection_checker.launch_file_analyse.string_utils import (
    find_linked_path,
)
import yaml


def read_ros_yaml(file_path: str) -> dict:
    """Read and return the contents of a YAML file."""
    with open(file_path, "r") as file:
        # Using safe_load() to avoid potential security risks
        data = yaml.safe_load(file)

    data = data["/**"]["ros__parameters"]
    return data


def include_ros_yaml(file_path: str) -> dict:
    """Read and return the contents of a YAML file."""
    with open(file_path, "r") as file:
        # Using safe_load() to avoid potential security risks
        data = yaml.safe_load(file)

    data = data["launch"]
    parameters = {}
    for argument_dict in data:
        name = argument_dict["arg"]["name"]
        default = argument_dict["arg"]["default"]
        parameters[name] = default
    return parameters


def parse_node_tag(node_tag: ET.Element, base_namespace: str, context: dict, local_context: dict):
    try:
        pkg = analyze_string(node_tag.get("pkg"), context, local_context, base_namespace)
        executable = analyze_string(node_tag.get("exec"), context, local_context, base_namespace)
    except Exception as e:
        print(f"Error in parsing node tag: {e}")
        print(
            node_tag.get("pkg"),
            node_tag.get("exec"),
            base_namespace,
            context["__current_launch_name_"],
        )
        raise Exception(f"Error in parsing node tag: {e}")
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
                path = analyze_string(child.get("from"), context, local_context, base_namespace)
                path = find_linked_path(path)
                if path.endswith("_empty.param.yaml"):
                    continue
                # print(path, child.get("from"))
                local_parameters["__param_files"].append(path)
                if path == "" or not os.path.exists(path):
                    print("-----Node Parameter not Found------")
                    print(f"----package: {pkg}-----")
                    print(f"----exec: {executable}-----")
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
        if child.tag == "remap":
            from_topic = analyze_string(child.get("from"), context, local_context, base_namespace)
            to_topic = analyze_string(child.get("to"), context, local_context, base_namespace)
            if "__remapping__" not in local_parameters:
                local_parameters["__remapping__"] = {}
            local_parameters["__remapping__"][from_topic] = to_topic

    context["__tree__"].add_child(
        context["__current_launch_name_"], f"{pkg}/{executable}", **local_parameters
    )


def parse_load_composable_node(
    load_composable_node_tag: ET.Element, base_namespace: str, context: dict, local_context: dict
):
    container_target = load_composable_node_tag.get("target")
    local_parameters = local_context.copy()
    local_parameters["__container_target__"] = container_target
    for child in load_composable_node_tag:
        process_tag(
            child,
            base_namespace,
            context,
            local_parameters,
        )
    return context


def parse_composable_node(
    composable_node_tag: ET.Element, base_namespace: str, context: dict, local_context: dict
):
    pkg = analyze_string(composable_node_tag.get("pkg"), context, local_context, base_namespace)
    executable = analyze_string(
        composable_node_tag.get("plugin"), context, local_context, base_namespace
    )
    local_parameters = {}
    local_parameters["__param_files"] = []
    # print(context, base_namespace)
    for child in composable_node_tag:
        if child.tag == "param":
            if child.get("name") is not None:
                local_parameters[child.get("name")] = analyze_string(
                    child.get("value"), context, local_context, base_namespace
                )
            if child.get("from") is not None:
                path = analyze_string(child.get("from"), context, local_context, base_namespace)
                path = find_linked_path(path)
                if path.endswith("_empty.param.yaml"):
                    continue
                # print(path, child.get("from"))
                local_parameters["__param_files"].append(path)
                if path == "":
                    print("-----Node Parameter not Found------")
                    print(f"----package: {pkg}-----")
                    print(f"----exec: {executable}-----")
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
        if child.tag == "remap":
            from_topic = analyze_string(child.get("from"), context, local_context, base_namespace)
            to_topic = analyze_string(child.get("to"), context, local_context, base_namespace)
            if "__remapping__" not in local_parameters:
                local_parameters["__remapping__"] = {}
            local_parameters["__remapping__"][from_topic] = to_topic

    context["__tree__"].add_child(
        context["__current_launch_name_"], f"{pkg}/{executable}", **local_parameters
    )


def check_if_run(tag: ET.Element, base_name: dict, context: dict, local_context: dict):
    """Many tag has a if and unless attribute, this function checks if the tag should be run or not."""
    if tag.get("if"):
        if_value = analyze_string(tag.get("if"), context, local_context, base_name)
        if_value = if_value.lower() == "true"
        if not if_value:
            return False
    if tag.get("unless"):
        unless_value = analyze_string(tag.get("unless"), context, local_context, base_name)
        unless_value = unless_value.lower() == "true"
        if unless_value:
            return False
    return True


def copy_context(context: dict):
    new_context = {}
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
    included_file = analyze_string(included_file, context, local_context, base_namespace)
    included_file = find_linked_path(included_file)

    if included_file.endswith(".yaml"):
        # this is a yaml file for parameters
        data = include_ros_yaml(included_file)
        for key in data:
            if key not in context:
                context[key] = data[key]
        return context

    temp_context = copy_context(context)
    argument_dict = {}
    for child in include_tag:
        if child.tag == "arg":
            value = analyze_string(child.get("value"), temp_context, local_context, base_namespace)
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
            f"{group_base_namespace}/{os.path.basename(included_file)}",
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


def parse_let_tag(let_tag: ET.Element, base_namespace: str, context: dict, local_context: dict):
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
    group_base_namespace = deepcopy(parent_file_space)
    for child in group_tag:
        if child.tag == "push-ros-namespace":
            if child.get("namespace").strip() == "/":
                continue
            group_base_namespace = f"{group_base_namespace}/{child.get('namespace').strip('/')}"
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
        context = parse_group_tag(tag, base_namespace, context, local_context, group_base_namespace)
    elif tag.tag == "include":
        context = process_include_tag(
            tag, context, local_context, base_namespace, group_base_namespace
        )
    elif tag.tag == "node":
        context = parse_node_tag(tag, base_namespace, context, local_context)
    elif tag.tag == "load_composable_node":
        context = parse_load_composable_node(tag, base_namespace, context, local_context)
    elif tag.tag == "composable_node":
        context = parse_composable_node(tag, base_namespace, context, local_context)
    return context


def parse_xml(file_path: str, namespace: str = "", context: dict = {}):
    """Recursively parse XML files, handling <include> tags. For each file, the namespace should be the same."""
    full_path = os.path.join(file_path)
    context["__current_launch_file__"] = full_path
    context["__current_launch_name_"] = f"{namespace}/{os.path.basename(full_path)}"
    if "__tree__" not in context:
        context["__tree__"] = LaunchTree()
    if context["__tree__"].root is None:
        context["__tree__"].add_root(context["__current_launch_name_"], path=full_path)
    tree = ET.parse(full_path)
    root = tree.getroot()

    # Process each node in the XML
    local_context = {}
    for tag in root:
        process_tag(tag, namespace, context, local_context)
    return context

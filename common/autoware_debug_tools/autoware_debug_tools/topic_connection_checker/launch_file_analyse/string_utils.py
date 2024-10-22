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
    """Return the share directory of the given package."""
    if package_name in BASE_PROJECT_MAPPING:
        return BASE_PROJECT_MAPPING[package_name]
    else:
        if FLAG_CHECKING_SYSTEM_PROJECTS:
            from ament_index_python.packages import get_package_share_directory

            BASE_PROJECT_MAPPING[package_name] = get_package_share_directory(package_name)
        else:
            BASE_PROJECT_MAPPING[
                package_name
            ] = f"/opt/ros/humble/share/{package_name}"  # use this for temporal solution;
        return BASE_PROJECT_MAPPING[package_name]

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

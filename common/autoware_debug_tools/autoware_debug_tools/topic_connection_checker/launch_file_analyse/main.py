from autoware_debug_tools.topic_connection_checker.launch_file_analyse.launch_xml_parser import parse_xml
from autoware_debug_tools.topic_connection_checker.launch_file_analyse.string_utils import find_unset_parameters, find_cmake_projects



def launch_file_analyse_main(launch_file, context={}, src_dir=None):
    if src_dir:
        find_cmake_projects(src_dir)
    context = parse_xml(launch_file, context=context)
    # with open("output.json", "w") as f:
    #     f.write(str(context["__tree__"]))
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
    args.add_argument("--flag_check_system_file", action="store_true")
    # context dictionary
    args.add_argument("--context", "--parameters", nargs="+", help="Key=value pairs")

    args = args.parse_args()
    src_dir = args.src_dir
    launch_file = args.launch_file
    FLAG_CHECKING_SYSTEM_PROJECTS = args.flag_check_system_file
    context = {}
    for param in args.context:
        key, value = param.split("=")
        context[key] = value

    launch_file_analyse_main(launch_file, context=context, src_dir=src_dir)

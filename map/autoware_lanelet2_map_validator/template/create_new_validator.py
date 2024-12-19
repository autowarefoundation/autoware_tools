import argparse
import os
import shutil


def create_files(
    base_directory, category_name, code_name, class_name, validator_name, check_function_name
):
    # Define directories
    template_directory = os.path.join(base_directory, "template")
    src_directory = os.path.join(base_directory, "src/validators", category_name)
    include_directory = os.path.join(
        base_directory, "src/include/lanelet2_map_validator/validators", category_name
    )
    docs_directory = os.path.join(base_directory, "docs", category_name)
    test_directory = os.path.join(base_directory, "test/src")

    # Define source and destination file paths
    cpp_template = os.path.join(template_directory, "validator_template.cpp")
    hpp_template = os.path.join(template_directory, "validator_template.hpp")
    test_template = os.path.join(template_directory, "test_validator_template.cpp")
    docs_template = os.path.join(template_directory, "validator_template.md")
    cpp_new = os.path.join(src_directory, f"{code_name}.cpp")
    print("Create " + cpp_new)
    hpp_new = os.path.join(include_directory, f"{code_name}.hpp")
    print("Create " + hpp_new)
    test_new = os.path.join(test_directory, f"test_{code_name}.cpp")
    print("Create " + test_new)
    docs_new = os.path.join(docs_directory, f"{code_name}.md")
    print("Create " + docs_new)

    # Copy template files
    shutil.copy(cpp_template, cpp_new)
    shutil.copy(hpp_template, hpp_new)
    shutil.copy(test_template, test_new)
    shutil.copy(docs_template, docs_new)

    # This is only for documents!!
    with open(docs_new, "r") as file:
        content = file.read()
    content = content.replace("validator_template", code_name)
    with open(docs_new, "w") as file:
        file.write(content)

    # Replace class name in the new files
    for file_path in [cpp_new, hpp_new, test_new, docs_new]:
        with open(file_path, "r") as file:
            content = file.read()
        # Replace the class name
        content = content.replace("ValidatorTemplate", class_name)
        content = content.replace("mapping.validator.template", validator_name)
        content = content.replace("checkFunction", check_function_name)
        content = content.replace(
            "validator_template.hpp",
            "lanelet2_map_validator/validators/" + category_name + "/" + code_name + ".hpp",
        )
        content = content.replace(
            "MAP__AUTOWARE_LANELET2_MAP_VALIDATOR__TEMPLATES__VALIDATOR_TEMPLATE_HPP_",
            "LANELET2_MAP_VALIDATOR__VALIDATORS__"
            + category_name.upper()
            + "__"
            + code_name.upper()
            + "_HPP_",
        )
        with open(file_path, "w") as file:
            file.write(content)


if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser(description="Generate files for a new validator.")
    parser.add_argument(
        "--base_directory", default="./", help="Path to the autoware_lanelet2_map_validator package"
    )
    parser.add_argument(
        "--category_name", default="enter_category", help="Category name of the validator"
    )
    parser.add_argument(
        "--code_name",
        default="enter_code_name",
        help="Code name for the validator files (e.g. code_name.cpp, code_name.hpp)",
    )
    parser.add_argument(
        "--class_name", default="EnterClassNameValidator", help="Class name for the validator"
    )
    parser.add_argument(
        "--validator_name",
        default="mapping.category.something",
        help="Full validator name (e. g. mapping.category.something)",
    )
    parser.add_argument(
        "--check_function_name",
        default="enter_function_name",
        help="Check function name for the validator",
    )

    args = parser.parse_args()

    # User-defined parameters
    directory_path = args.base_directory  # Replace with package directory path
    category_name = args.category_name
    code_name = args.code_name
    class_name = args.class_name
    validator_name = args.validator_name
    check_function_name = args.check_function_name

    create_files(
        directory_path, category_name, code_name, class_name, validator_name, check_function_name
    )
    print("Process complete.")

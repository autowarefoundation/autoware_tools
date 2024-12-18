import os
import shutil


def create_files(
    base_directory, category_name, code_name, class_name, validator_name, check_function_name
):
    # Define directories
    template_directory = os.path.join(base_directory, "src/validators")
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
    hpp_new = os.path.join(include_directory, f"{code_name}.hpp")
    test_new = os.path.join(test_directory, f"test_{code_name}.cpp")
    docs_new = os.path.join(docs_directory, f"{code_name}.md")

    # Copy template files
    shutil.copy(cpp_template, cpp_new)
    shutil.copy(hpp_template, hpp_new)
    shutil.copy(test_template, test_new)
    shutil.copy(docs_template, docs_new)

    # Replace class name in the new files
    for file_path in [cpp_new, hpp_new, test_new, docs_new]:
        with open(file_path, "r") as file:
            content = file.read()
        # Replace the class name
        content = content.replace("ValidatorTemplate", class_name)
        content = content.replace("mapping.validator.template", validator_name)
        content = content.replace("checkFunction", check_function_name)
        content = content.replace("- validator_template", "- " + code_name)
        content = content.replace(
            "validator_template.hpp",
            "lanelet2_map_validator/validators/" + category_name + "/" + code_name + ".hpp",
        )
        content = content.replace(
            "VALIDATORS__VALIDATOR_TEMPLATE_HPP_",
            "LANELET2_MAP_VALIDATOR__VALIDATORS__"
            + category_name.upper()
            + "__"
            + code_name.upper()
            + "_HPP_",
        )
        with open(file_path, "w") as file:
            file.write(content)


if __name__ == "__main__":
    # User-defined parameters
    directory_path = "./"  # Replace with package directory path
    category_name = "category"
    code_name = "enter_code_name"  #
    class_name = "EnterClassNameValidator"
    validator_name = "mapping.enter.validator_name"
    check_function_name = "enter_function_name"

    create_files(
        directory_path, category_name, code_name, class_name, validator_name, check_function_name
    )
    print("Process complete.")

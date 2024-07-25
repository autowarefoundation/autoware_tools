from setuptools import find_packages
from setuptools import setup

package_name = "autoware_debug_tools"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyperclip"],
    zip_safe=True,
    maintainer="Yukinari Hisaki",
    maintainer_email="yukinari.hisaki.2@tier4.jp",
    description="The autoware_debug_tools package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "processing_time_visualizer = autoware_debug_tools.processing_time_visualizer.node:main",
            "system_usage_monitor = autoware_debug_tools.system_usage_monitor:main",
        ],
    },
)

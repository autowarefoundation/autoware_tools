from setuptools import find_packages
from setuptools import setup

package_name = "autoware_lanelet2_divider"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config/lanelet2_divider.param.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ataparlar",
    maintainer="bzeren",
    maintainer_email="ataparlar@leodrive.ai",
    maintainer_email="baris@leodrive.ai",
    description="A tool to divide lanelet2 map into smaller pieces with respect to MGRS grid.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lanelet2_divider = autoware_lanelet2_divider.autoware_lanelet2_divider:main",
        ],
    },
)

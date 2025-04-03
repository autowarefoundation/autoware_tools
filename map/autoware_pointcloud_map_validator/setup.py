from setuptools import find_packages
from setuptools import setup

package_name = "autoware_pointcloud_map_validator"

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
    maintainer="Yuxuan Liu",
    maintainer_email="uken.ryu.2@tier4.jp",
    description="The autoware_pointcloud_map_validator package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_pointcloud_checker = autoware_pointcloud_map_validator.autoware_pointcloud_checker:main",
        ],
    },
)

# autoware_dependency_checker

This package provides a script for checking whether each package's dependencies listed in a package.xml are used or not.
Currently, it mainly checks packages that start with `autoware_`.

## Depedency Checking

The script will try to match the dependencies and the headers by reading the dependencies listed in package.xml and the included headers in the source files.

Some dependency in `package.xml` and the included header might differ.
The following table shows the matching between dependency names and headers:

| from               | to                 | description                       |
| ------------------ | ------------------ | --------------------------------- |
| autoware_pkg_name  | autoware/pkg_name  | Usually this style should be used |
| autoware\_\*\_msgs | autoware\_\*\_msgs | For messages                      |
| autoware_other_pkg | autoware_other_pkg | E.g. autoware_lanelet2_extension  |

## Usage

```Text
# build
$ cd to/autoware_tools
$ colcon build --symlink-install --cmake-args --packages-up-to autoware_dependency_checker
$ source

# run
$ cd to/your/autoware
$ ros2 run ros2 run autoware_dependency_checker dependency_checker.sh

# run in some package
$ cd to/some/package
$ ros2 run ros2 run autoware_dependency_checker dependency_checker.sh
```

#!/bin/bash
# This script finds recursively all package.xml in the src/ subdirectory
# Then check if all packages marked <depend> are effectively used somewhere in the package.
# Packages marked <depend> normally either provide shared libraries or custom messages

# Packages that should always be <buildtool_depend>, not <depend>
KNOWN_BUILDTOOLS="rosidl_default_generators;autoware_cmake;eigen3_cmake_module"

current_dir=$(pwd)

# Out of pattern packages
# These will get checked with `package_name/.*` pattern
# autoware_*_msg is handled in the following process, so you don't need to add here
EXCLUDE_PACKAGES=(
    "autoware_ad_api_specs"
    "autoware_lanelet2_extension"
    "autoware_raw_vehicle_cmd_converter"
)

# Find all package names under the current directory
mapfile -t ALL_PACKAGES < <(find "$current_dir" \
    -not \( -path "$current_dir/install" -prune \) \
    -not \( -path "$current_dir/build" -prune \) \
    -name "package.xml" -print0 | xargs -0 -n 1 dirname | xargs -n 1 basename | sort -u)

# Find all autoware packages starting with "autoware_"
# These packages will get checked with `autoware/pkg_name/.*`
BASE_RULE_TARGETS=()
for pkg_name in "${ALL_PACKAGES[@]}"; do
    if [[ $pkg_name == autoware_* ]]; then
        filter_flag=0

        for exclude_pkg_name in "${EXCLUDE_PACKAGES[@]}"; do
            if [[ $pkg_name == "$exclude_pkg_name" ]]; then
                filter_flag=1
                break
            fi
        done
        if [[ $filter_flag -eq 0 ]]; then
            BASE_RULE_TARGETS+=("$pkg_name")
        fi
    fi
done

pkgs=$(find "$current_dir" \
    -not \( -path "$current_dir/install" -prune \) \
    -not \( -path "$current_dir/build" -prune \) \
    -name "package.xml")

for pkg in $pkgs; do
    echo "--- Checking $pkg ---"
    # Get all packages marked <depend>. For example:
    #   <depend>dep_name</depend>
    deps=$(grep -d skip -oP '^\s*<depend>\K[^<]+' "$pkg")
    dir=$(dirname "$pkg")

    if [[ ! -f "$dir/CMakeLists.txt" ]]; then
        echo "Skipping package with no CMakeLists.txt (is python package?)"
        echo ""
        continue
    fi

    for dep in $deps; do
        # filter out buildtools (should not use <depend>)
        if grep -q "$dep" <<<"$KNOWN_BUILDTOOLS"; then
            echo "$dep should rather be marked as <buildtool_depend>, not <depend>"
            continue
        fi

        # filter out ament stuff (should not use <depend>)
        if [[ $dep =~ ament_.*$ ]]; then
            echo "$dep should rather be marked as either <build_depend> or <test_depend>, not <depend>"
            continue
        fi

        # filter out python stuff (should not use <depend>)
        if [[ $dep =~ python-.*$ ]]; then
            echo "$dep should rather be marked as either <build_depend> or <test_depend>, not <depend>"
            continue
        fi

        # By convention, dependency headers must be prefixed with the package name. For Example:
        # tier4_autoware_utils/geometry/geometry.hpp
        header_regex="$dep/.*"

        # System dependencies don't follow this rule. They all have custom names.
        [[ $dep =~ libpcl-.*$ ]] && header_regex="pcl/.*"
        [[ $dep =~ libboost-.*$ ]] && header_regex="boost/.*"
        [[ $dep =~ libqt5-.*$ ]] && header_regex="Q.*"
        [[ $dep == "qtbase5-dev" ]] && header_regex="Q.*"
        [[ $dep == "libopencv-dev" ]] && header_regex="opencv2/.*"
        [[ $dep == "pugixml-dev" ]] && header_regex="pugixml.hpp"
        [[ $dep == "yaml_cpp_vendor" ]] && header_regex="yaml-cpp/yaml.h"
        [[ $dep == "nlohmann-json-dev" ]] && header_regex="nlohmann/json.hpp"
        [[ $dep == "range-v3" ]] && header_regex="range/v3/.*"
        [[ $dep == "libcpprest-dev" ]] && header_regex="cpprest/.*"
        [[ $dep == "eigen" ]] && header_regex="Eigen/.*"
        [[ $dep == "libnl-3-dev" ]] && header_regex="netlink/.*"
        [[ $dep == "libpcap" ]] && header_regex="pcap.h"
        [[ $dep == "cgal" ]] && header_regex="CGAL/.*"
        [[ $dep == "osqp_vendor" ]] && header_regex="osqp/.*"
        [[ $dep == "magic_enum" ]] && header_regex="magic_enum.hpp"
        [[ $dep == "geographiclib" ]] && header_regex="GeographicLib/.*"
        # Some autoware packages don't follow the convention either...
        [[ $dep == "autoware_auto_common" ]] && header_regex="(common|helper_functions)/.*"
        # cspell: ignore multigrid
        [[ $dep == "ndt_omp" ]] && header_regex="(pclomp|multigrid_pclomp)/.*"
        [[ $dep == "planning_test_utils" ]] && header_regex="planning_interface_test_manager/.*"
        [[ $dep == "shape_estimation" ]] && header_regex="autoware/shape_estimation/.*"
        # Add more as needed...

        # Check the dependency with the including rule:
        #   autoware_pkg_name -> autoware/pkg_name/.*
        #   autoware_*_msgs   -> autoware_*_msgs/.*
        for autoware_pkg in "${BASE_RULE_TARGETS[@]}"; do
            if [[ $dep == "$autoware_pkg" ]]; then
                # for the autoware_*_msgs
                if [[ $autoware_pkg == *_msgs ]]; then
                    source_name="${autoware_pkg}"
                else
                    # replace `autoware_` with `autoware/`
                    source_name="${autoware_pkg/autoware_/autoware/}"
                fi

                header_regex="${source_name}/.*"
                break
            fi
        done

        # Look for C/C++ includes. For example:
        # #include "tier4_autoware_utils/geometry/geometry.hpp"
        # Note: whether these includes are actually useful is out-of-scope of this script
        # There are many great tools for that.
        include_regex="^#include [<\"]${header_regex}[>\"]"

        # Dependencies defining custom messages may also be re-used in other messages or services
        # By convention, these packages must be named "*_msgs", excepted for "builtin_interfaces"
        if [[ $dep =~ .*_msgs$ ]] || [[ $dep == "builtin_interfaces" ]]; then
            if grep -wIPrq "$dep" "$dir" --include \*.msg --include \*.srv --include \*.idl; then
                continue # found!
            fi
        fi

        # Check if the dependency is included anywhere in the source files
        if grep -wIPrq "$include_regex" "$dir" \
            --include \*.c --include \*.cc --include \*.cpp --include \*.h --include \*.hpp; then
            continue # found!
        fi

        echo "$dep seems not to be used"

        # /!\ WARNING - DANGEROUS /!\
        # Uncomment to delete unused dependency!
        # sed -i "/<depend>$dep<\/depend>/d" $pkg
    done
    echo ""
done

# Copyright (c) 2024 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from os.path import expandvars
from pathlib import Path

from autoware_msg_bag_converter.converter import convert_bag


def convert_bag_in_directory(input_dir: str, output_dir: str) -> None:
    input_root = Path(input_dir)
    output_root = Path(output_dir)

    bag_paths = input_root.glob("**/*.db3")  # Will mcap conversion be supported?
    for db3_path in bag_paths:
        input_bag_dir = db3_path.parent
        rel_path = input_bag_dir.relative_to(input_root)
        output_bag_dir = output_root.joinpath(rel_path)
        convert_bag(input_bag_dir.as_posix(), output_bag_dir.as_posix())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="path of input bag with autoware_auto_msgs")
    parser.add_argument("output", help="path of output bag with autoware_msgs")
    parser.add_argument(
        "--directory",
        "-d",
        action="store_true",
        help="If this option is specified, all rosbags under the directory specified in input will be converted",
    )
    args = parser.parse_args()
    if not args.directory:
        convert_bag(expandvars(args.input), expandvars(args.output))
    else:
        convert_bag_in_directory(expandvars(args.input), expandvars(args.output))


if __name__ == "__main__":
    main()

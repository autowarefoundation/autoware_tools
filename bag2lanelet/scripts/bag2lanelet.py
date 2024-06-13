#!/bin/env python3
import argparse

import os
import pathlib
from datetime import datetime

from bag2way import bag2pose, pose2line
from lanelet_xml import LaneletMap


def generate(input_path, output, width, mgrs, interval, offset, use_centerline=False):
    pose_array = bag2pose(input_path, interval)
    pose_array = pose_array[::10]
    left, right, center = pose2line(pose_array, width=width, offset=offset)

    m = LaneletMap(mgrs=mgrs)
    left_nodes = [m.add_node(*node) for node in left]
    right_nodes = [m.add_node(*node) for node in right]
    center_nodes = [m.add_node(*node) for node in center]

    left_line = m.add_way(left_nodes)
    right_line = m.add_way(right_nodes)
    center_line = m.add_way(center_nodes) if use_centerline else None

    m.add_relation(left_line, right_line, center_line)
    os.makedirs(output, exist_ok=True) if not os.path.isdir(output) else None
    m.save(
        str(output) + "/" + datetime.now().strftime("%y-%m-%d-%H-%M-%S") + "-" + "lanelet2_map.osm"
    )


def main():
    parser = argparse.ArgumentParser(description="Create lanelet2 file from rosbag2")
    parser.add_argument("input_bag", help="input bag stored path")
    parser.add_argument("output_lanelet", help="output lanelet2 save path")
    parser.add_argument("-l", "--width", type=float, default=2.0, help="lane width[m]")
    parser.add_argument("-m", "--mgrs", default="54SUE", help="MGRS code")
    parser.add_argument(
        "--interval",
        type=float,
        nargs=2,
        default=[0.1, 2.0],
        help="min and max interval between tf position",
    )
    parser.add_argument(
        "--offset", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="offset[m] from base_link"
    )
    parser.add_argument("--center", action="store_true", help="add centerline to lanelet")

    args = parser.parse_args()
    input_path = pathlib.Path(args.input_bag)
    if not input_path.exists():
        raise FileNotFoundError("Input bag folder '{}' is not found.".format(input_path))
    output_path = pathlib.Path(args.output_lanelet)

    print(args)
    generate(
        input_path, output_path, args.width, args.mgrs, args.interval, args.offset, args.center
    )


if __name__ == "__main__":
    main()

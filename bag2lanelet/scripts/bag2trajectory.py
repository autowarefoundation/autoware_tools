#!/bin/env python3
import argparse
import pathlib

import numpy as np
import tf_transformations
from bag2way import bag2pose


def genarate(input_path, output_path):
    pose_array = bag2pose(input_path)
    pose_array = pose_array[::50]

    trajectory = []
    for pose in pose_array:
        euler = tf_transformations.euler_from_quaternion(pose[3:])
        trajectory.append([*pose[:3], euler[2]])
    np.savetxt(
        str(output_path), trajectory, fmt="%.3f", delimiter=",", header="x,y,z,yaw", comments=""
    )


def main():
    parser = argparse.ArgumentParser(description="Create lanelet2 file from rosbag2")
    parser.add_argument("input_bag", help="input bag stored path")
    parser.add_argument("output_csv", help="output trajectory csv path")

    args = parser.parse_args()
    input_path = pathlib.Path(args.input_bag)
    if not input_path.exists():
        raise FileNotFoundError("Input bag folder '{}' is not found.".format(input_path))
    output_path = pathlib.Path(args.output_csv)

    print(input_path, output_path)
    genarate(input_path, output_path)


if __name__ == "__main__":
    main()

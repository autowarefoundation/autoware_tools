#!/bin/env python3
import argparse
import os
import pathlib
from datetime import datetime

import folium
from bag2way import bag2point_stamped, bag2pose, pose2line
from tools.bag2lanelet.scripts.lanelet_xml import LaneletMap


def genarate(input_path, output, mgrs):
    point_array = bag2point_stamped(input_path, 40.0, 500.0)
    m = LaneletMap(mgrs=mgrs)
    latlon = [{'lat': lat, 'lon': lon} for lat, lon in [m.get_latlon(*node) for node in point_array[:, 1:4]]]
    t = point_array[:, 0]

    f = folium.Figure(width=800, height=500)
    map = folium.Map(location=[latlon[0]["lat"], latlon[0]["lon"]], zoom_start=20).add_to(f)
    idx = 0
    # Note: latlon and t should be same index size
    for ll in latlon:
        print(ll["lat"], ll["lon"], t[idx])
        folium.Marker(location=[ll["lat"], ll["lon"]], popup=t[idx]).add_to(map)
        idx += 1
    os.makedirs(output, exist_ok=True) if not os.path.isdir(output) else None
    f.save(str(output) + "/" + datetime.now().strftime("%y-%m-%d-%H-%M-%S") + "-" + "map.html")
    map.show_in_browser()


def main():
    parser = argparse.ArgumentParser(description="Create lanelet2 file from rosbag2")
    parser.add_argument("input_bag", help="input bag stored path")
    parser.add_argument("output_dir", default="", help="output html store dir")
    parser.add_argument("-m", "--mgrs", default="54SUE", help="MGRS code")

    args = parser.parse_args()
    input_path = pathlib.Path(args.input_bag)
    if not input_path.exists():
        raise FileNotFoundError("Input bag folder '{}' is not found.".format(input_path))

    print(input_path)
    genarate(input_path, args.output_dir, args.mgrs)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import copy
import glob
import os
import os.path as osp
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata
from visualization_msgs.msg import MarkerArray

VECTOR_MAP_MARKER_TOPIC = "/map/vector_map_marker"
VECTOR_MAP_MARKER_TYPE = "visualization_msgs/msg/MarkerArray"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Launch tier4_map_component.launch.xml, capture one "
            "/map/vector_map_marker message, and insert it into a rosbag2 bag."
        )
    )
    parser.add_argument("input_bag", help="source rosbag directory")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help=("destination rosbag directory. " "If omitted, <input_bag>/map_added is used"),
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="rewrite the original bag directory in place",
    )
    parser.add_argument("--map-path", required=True, help="directory that contains map files")
    parser.add_argument(
        "--setup-script",
        help="setup script sourced before ros2 launch (default: <cwd>/install/setup.bash)",
    )
    parser.add_argument(
        "--launch-package",
        default="autoware_launch",
        help="ros2 launch package name (default: autoware_launch)",
    )
    parser.add_argument(
        "--launch-file",
        default="tier4_map_component.launch.xml",
        help="ros2 launch file name (default: tier4_map_component.launch.xml)",
    )
    parser.add_argument(
        "--lanelet2-map-file",
        default="lanelet2_map.osm",
        help="lanelet2 map filename under --map-path",
    )
    parser.add_argument(
        "--pointcloud-map-file",
        default="pointcloud_map.pcd",
        help=(
            "pointcloud map filename under --map-path. If this file does not exist and "
            "pointcloud_map_metadata.yaml exists, the tool falls back to '.'"
        ),
    )
    parser.add_argument(
        "--map-projector-info-file",
        default="map_projector_info.yaml",
        help="map projector info filename under --map-path",
    )
    parser.add_argument(
        "--pointcloud-map-metadata-file",
        default="pointcloud_map_metadata.yaml",
        help="pointcloud metadata filename under --map-path",
    )
    parser.add_argument(
        "--topic",
        default=VECTOR_MAP_MARKER_TOPIC,
        help=f"captured topic name (default: {VECTOR_MAP_MARKER_TOPIC})",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=60.0,
        help="timeout waiting for the marker topic after launch starts",
    )
    parser.add_argument(
        "--stamp-ns",
        type=int,
        help="explicit bag timestamp in nanoseconds for the inserted message",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        choices=["mcap", "sqlite3"],
        help="output rosbag storage id (default: mcap)",
    )
    return parser.parse_args()


def expand_path(path: str) -> str:
    return osp.abspath(osp.expandvars(osp.expanduser(path)))


def estimate_storage_id(bag_dir: str) -> str:
    storage_ids = {".db3": "sqlite3", ".mcap": "mcap"}
    for bag_file in sorted(glob.glob(osp.join(bag_dir, "*"))):
        extension = osp.splitext(bag_file)[1]
        if extension in storage_ids:
            return storage_ids[extension]
    raise FileNotFoundError(f"No .db3 or .mcap file found in {bag_dir}")


def list_bag_files(bag_dir: str, extension: str) -> list[str]:
    return sorted(glob.glob(osp.join(bag_dir, f"*{extension}")))


def get_converter_options() -> ConverterOptions:
    return ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )


def create_reader(bag_dir: str) -> SequentialReader:
    storage_id = estimate_storage_id(bag_dir)
    storage_options = StorageOptions(uri=bag_dir, storage_id=storage_id)
    reader = SequentialReader()
    try:
        reader.open(storage_options, get_converter_options())
    except RuntimeError as exc:
        error_message = str(exc)
        if storage_id != "mcap" or "unindexed MCAP file" not in error_message:
            raise
        print(f"[info] Reindexing input bag: {bag_dir}", file=sys.stderr)
        Reindexer().reindex(storage_options)
        reader = SequentialReader()
        reader.open(storage_options, get_converter_options())
    return reader


def get_first_bag_timestamp(bag_dir: str) -> int:
    reader = create_reader(bag_dir)
    if not reader.has_next():
        raise RuntimeError(f"Input bag is empty: {bag_dir}")
    _, _, stamp = reader.read_next()
    return stamp


def resolve_map_files(args) -> dict:
    map_path = expand_path(args.map_path)
    lanelet2_map_path = osp.join(map_path, args.lanelet2_map_file)
    pointcloud_map_metadata_path = osp.join(map_path, args.pointcloud_map_metadata_file)
    map_projector_info_path = osp.join(map_path, args.map_projector_info_file)

    pointcloud_candidate = osp.join(map_path, args.pointcloud_map_file)
    if osp.exists(pointcloud_candidate):
        pointcloud_map_file = args.pointcloud_map_file
    elif osp.exists(pointcloud_map_metadata_path):
        pointcloud_map_file = "."
    else:
        pointcloud_map_file = args.pointcloud_map_file

    pointcloud_map_path = osp.join(map_path, pointcloud_map_file)

    missing = []
    for required_path in [lanelet2_map_path, pointcloud_map_metadata_path, map_projector_info_path]:
        if not osp.exists(required_path):
            missing.append(required_path)
    if not osp.exists(pointcloud_map_path):
        missing.append(pointcloud_map_path)
    if missing:
        missing_str = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"Required map files are missing:\n{missing_str}")

    return {
        "map_path": map_path,
        "lanelet2_map_file": args.lanelet2_map_file,
        "pointcloud_map_file": pointcloud_map_file,
    }


def resolve_output_bag(input_bag: str, output_bag: Optional[str]) -> str:
    if output_bag is not None:
        return expand_path(output_bag)

    input_bag = expand_path(input_bag)
    return osp.join(input_bag, "map_added")


def remove_if_exists(path: str):
    if not osp.exists(path):
        return
    if osp.isdir(path) and not osp.islink(path):
        shutil.rmtree(path)
    else:
        os.remove(path)


def replace_bag_directory(src_dir: str, dst_dir: str):
    backup_dir = f"{dst_dir}.bak"
    remove_if_exists(backup_dir)
    os.rename(dst_dir, backup_dir)
    try:
        os.rename(src_dir, dst_dir)
    except Exception:
        os.rename(backup_dir, dst_dir)
        raise
    remove_if_exists(backup_dir)


def resolve_in_place_output_bag(input_bag: str) -> str:
    input_bag = expand_path(input_bag)
    parent_dir = osp.dirname(input_bag)
    bag_name = osp.basename(input_bag.rstrip("/"))
    temp_parent_dir = osp.join(parent_dir, f".{bag_name}.map_marker_tmp")
    return osp.join(temp_parent_dir, bag_name)


def align_output_mcap_filename(input_bag: str, output_bag: str):
    input_mcap_files = list_bag_files(input_bag, ".mcap")
    output_mcap_files = list_bag_files(output_bag, ".mcap")

    if len(input_mcap_files) != 1 or len(output_mcap_files) != 1:
        print(
            "[info] Skip mcap filename alignment because the bag does not contain exactly one .mcap file",
            file=sys.stderr,
        )
        return

    input_name = osp.basename(input_mcap_files[0])
    output_name = osp.basename(output_mcap_files[0])
    if input_name == output_name:
        return

    renamed_output = osp.join(output_bag, input_name)
    remove_if_exists(renamed_output)
    os.rename(output_mcap_files[0], renamed_output)
    Reindexer().reindex(StorageOptions(uri=output_bag, storage_id="mcap"))


class MarkerCaptureNode(Node):
    def __init__(self, topic_name: str):
        super().__init__("vector_map_marker_capture")
        self.message: Optional[MarkerArray] = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MarkerArray, topic_name, self.on_message, qos)

    def on_message(self, msg: MarkerArray):
        if self.message is None:
            self.message = copy.deepcopy(msg)
            self.get_logger().info(
                f"Captured {len(msg.markers)} markers from {VECTOR_MAP_MARKER_TOPIC}"
            )


def build_launch_command(args, map_files: dict) -> str:
    setup_script = args.setup_script
    if setup_script is None:
        setup_script = osp.join(os.getcwd(), "install", "setup.bash")
    setup_script = expand_path(setup_script)
    if not osp.exists(setup_script):
        raise FileNotFoundError(f"setup script not found: {setup_script}")

    ros_args = [
        "ros2",
        "launch",
        args.launch_package,
        args.launch_file,
        f"map_path:={map_files['map_path']}",
        f"lanelet2_map_file:={map_files['lanelet2_map_file']}",
        f"pointcloud_map_file:={map_files['pointcloud_map_file']}",
    ]
    quoted_args = " ".join(shlex.quote(arg) for arg in ros_args)
    return f"source {shlex.quote(setup_script)} && {quoted_args}"


def terminate_process_group(process: subprocess.Popen):
    if process.poll() is not None:
        return
    pgid = os.getpgid(process.pid)
    for sig, wait_sec in [(signal.SIGINT, 5.0), (signal.SIGTERM, 5.0)]:
        os.killpg(pgid, sig)
        try:
            process.wait(timeout=wait_sec)
            return
        except subprocess.TimeoutExpired:
            continue
    os.killpg(pgid, signal.SIGKILL)
    process.wait(timeout=5.0)


def capture_vector_map_marker(args, launch_command: str) -> MarkerArray:
    process = subprocess.Popen(["bash", "-lc", launch_command], preexec_fn=os.setsid)

    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    node = MarkerCaptureNode(args.topic)
    executor.add_node(node)

    deadline = time.monotonic() + args.timeout_sec
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    f"ros2 launch exited before {args.topic} was captured "
                    f"(exit code: {process.returncode})"
                )
            executor.spin_once(timeout_sec=0.2)
            if node.message is not None:
                return node.message
        raise TimeoutError(f"Timed out waiting for {args.topic} after {args.timeout_sec} seconds")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        terminate_process_group(process)


def create_output_writer(output_bag: str, storage_id: str) -> SequentialWriter:
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=output_bag, storage_id=storage_id), get_converter_options())
    return writer


def safe_create_topic(
    writer: SequentialWriter, topic_metadata: TopicMetadata, required: bool = False
) -> bool:
    try:
        writer.create_topic(topic_metadata)
        return True
    except Exception as exc:
        if required:
            raise RuntimeError(
                f"Failed to create required topic {topic_metadata.name} ({topic_metadata.type}): {exc}"
            ) from exc
        print(
            f"[warn] Skip topic {topic_metadata.name} ({topic_metadata.type}) "
            f"because topic registration failed: {exc}",
            file=sys.stderr,
        )
        return False


def insert_marker_topic(
    input_bag: str,
    output_bag: str,
    topic_name: str,
    marker_msg: MarkerArray,
    marker_stamp_ns: int,
    storage_id: str,
):
    reader = create_reader(input_bag)
    writer = create_output_writer(output_bag, storage_id)

    writable_topics = set()
    skipped_topics = set()
    for topic_metadata in reader.get_all_topics_and_types():
        if safe_create_topic(writer, topic_metadata):
            writable_topics.add(topic_metadata.name)
        else:
            skipped_topics.add(topic_metadata.name)

    if topic_name not in writable_topics:
        inserted = safe_create_topic(
            writer,
            TopicMetadata(
                name=topic_name,
                type=VECTOR_MAP_MARKER_TYPE,
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            required=True,
        )
        if inserted:
            writable_topics.add(topic_name)

    writer.write(topic_name, serialize_message(marker_msg), marker_stamp_ns)

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in writable_topics:
            continue
        writer.write(topic, data, stamp)

    del writer
    Reindexer().reindex(StorageOptions(uri=output_bag, storage_id=storage_id))
    if skipped_topics:
        print(
            f"[warn] Skipped {len(skipped_topics)} topics that could not be registered in the output bag",
            file=sys.stderr,
        )
        for skipped_topic in sorted(skipped_topics):
            print(f"[warn]   {skipped_topic}", file=sys.stderr)


def main():
    args = parse_args()
    input_bag = expand_path(args.input_bag)

    if not osp.isdir(input_bag):
        raise FileNotFoundError(f"Input bag directory not found: {input_bag}")
    if args.in_place and args.output_bag is not None:
        raise ValueError("--in-place and output_bag cannot be used together")

    if args.in_place:
        output_bag = resolve_in_place_output_bag(input_bag)
        remove_if_exists(output_bag)
        os.makedirs(osp.dirname(output_bag), exist_ok=True)
    else:
        output_bag = resolve_output_bag(input_bag, args.output_bag)
        if input_bag == output_bag:
            raise ValueError("output_bag must be different from input_bag")
        remove_if_exists(output_bag)
        os.makedirs(osp.dirname(output_bag), exist_ok=True)

    map_files = resolve_map_files(args)
    marker_stamp_ns = (
        args.stamp_ns if args.stamp_ns is not None else get_first_bag_timestamp(input_bag)
    )
    launch_command = build_launch_command(args, map_files)

    print(f"[1/3] Launching {args.launch_package}/{args.launch_file}")
    marker_msg = capture_vector_map_marker(args, launch_command)

    print(f"[2/3] Writing {args.topic} into {output_bag}")
    insert_marker_topic(
        input_bag=input_bag,
        output_bag=output_bag,
        topic_name=args.topic,
        marker_msg=marker_msg,
        marker_stamp_ns=marker_stamp_ns,
        storage_id=args.storage_id,
    )
    if args.storage_id == "mcap" and estimate_storage_id(input_bag) == "mcap":
        align_output_mcap_filename(input_bag, output_bag)

    if args.in_place:
        print("[3/4] Replacing original bag")
        replace_bag_directory(output_bag, input_bag)
        remove_if_exists(osp.dirname(output_bag))
        final_output_bag = input_bag
        print("[4/4] Completed")
    else:
        final_output_bag = output_bag
        print("[3/3] Completed")

    print(f"input_bag={input_bag}")
    print(f"output_bag={final_output_bag}")
    print(f"inserted_topic={args.topic}")
    print(f"inserted_stamp_ns={marker_stamp_ns}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)

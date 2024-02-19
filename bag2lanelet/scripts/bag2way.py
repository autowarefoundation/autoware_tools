from datetime import datetime

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message
import tf_transformations


def create_reader(bag_dir: str) -> SequentialReader:
    storage_options = StorageOptions(
        uri=bag_dir,
        storage_id="sqlite3",
    )
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


# too close & outlier pose filter
def is_close_pose(p0, p1, eps, thresh):
    dist = ((p0.x - p1.x) ** 2 + (p0.y - p1.y) ** 2 + (p0.z - p1.z) ** 2) ** 0.5
    if dist < eps or thresh < dist:
        return True
    else:
        return False


def bag2pose(input_path, interval=[0.1, 10000.0]):
    reader = create_reader(str(input_path))
    type_map = {}
    for topic_type in reader.get_all_topics_and_types():
        type_map[topic_type.name] = topic_type.type

    pose_list = []
    is_initial_pose = True
    prev_trans = None
    # read topic and fix timestamp if lidar, and write
    while reader.has_next():
        (topic, data, stamp) = reader.read_next()
        if topic == "/tf":
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            for transform in msg.transforms:
                if transform.child_frame_id != "base_link":
                    continue
                trans = transform.transform.translation
                rot = transform.transform.rotation
                if is_initial_pose:
                    is_initial_pose = False
                    prev_trans = trans
                elif is_close_pose(prev_trans, trans, interval[0], interval[1]):
                    # print("too close or too far")
                    continue
                pose_list.append(np.r_[trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w])
                prev_trans = trans

    return np.array(pose_list)


def bag2point_stamped(input_path, too_close, too_far):
    reader = create_reader(str(input_path))
    type_map = {}
    for topic_type in reader.get_all_topics_and_types():
        type_map[topic_type.name] = topic_type.type

    pose_list = []
    is_initial_pose = True
    prev_trans = None
    # read topic and fix timestamp if lidar, and write
    while reader.has_next():
        (topic, data, stamp) = reader.read_next()
        if topic == "/tf":
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            for transform in msg.transforms:
                if transform.child_frame_id != "base_link":
                    continue
                trans = transform.transform.translation
                date_time = datetime.fromtimestamp(stamp * 1e-9)
                if is_initial_pose:
                    is_initial_pose = False
                    prev_trans = trans
                elif is_close_pose(prev_trans, trans, too_close, too_far):
                    # print("too close or too far")
                    continue
                pose_list.append(np.r_[date_time, trans.x, trans.y, trans.z])
                prev_trans = trans

    return np.array(pose_list)


def pose2line(pose_array, width=3.0, offset=[0, 0, 0]):
    pos = pose_array[:, 0:3]
    rot = pose_array[:, 3:7]
    left = []
    right = []
    center = []
    for p, r in zip(pos, rot):
        lat = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(r, [0, width / 2.0, 0, 0]),
            tf_transformations.quaternion_conjugate(r),
        )[:3]

        off = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(r, [*offset, 0]),
            tf_transformations.quaternion_conjugate(r),
        )[:3]

        left.append(p + off + lat)
        right.append(p + off - lat)
        center.append(p + off)
    return left, right, center

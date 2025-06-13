"""A script to interpolate poses to match the timestamp in target_timestamp."""

import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp


def interpolate_pose(df_pose: pd.DataFrame, target_timestamp: pd.Series) -> pd.DataFrame:
    """Interpolate each pose in df_pose to match the timestamp in target_timestamp."""
    # check monotonicity
    assert df_pose["timestamp"].is_monotonic_increasing
    assert target_timestamp.is_monotonic_increasing

    # check length
    assert len(df_pose) > 0, f"{len(df_pose)=}"
    assert len(target_timestamp) > 0, f"{len(target_timestamp)=}"

    # check range
    assert df_pose.iloc[0]["timestamp"] <= target_timestamp.iloc[0]
    assert target_timestamp.iloc[-1] <= df_pose.iloc[-1]["timestamp"]

    position_keys = ["position.x", "position.y", "position.z"]
    orientation_keys = [
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]
    linear_velocity_keys = ["linear_velocity.x", "linear_velocity.y", "linear_velocity.z"]
    angular_velocity_keys = ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]

    df_pose = df_pose.reset_index(drop=True)
    target_timestamp = target_timestamp.reset_index(drop=True)

    target_index = 0
    df_index = 0
    data_dict = {
        "timestamp": [],
        **{key: [] for key in position_keys},
        **{key: [] for key in orientation_keys},
        **{key: [] for key in linear_velocity_keys},
        **{key: [] for key in angular_velocity_keys},
    }
    while df_index < len(df_pose) - 1 and target_index < len(target_timestamp):
        curr_time = df_pose.iloc[df_index]["timestamp"]
        next_time = df_pose.iloc[df_index + 1]["timestamp"]
        target_time = target_timestamp[target_index]

        # Find a df_index that includes target_time
        if not (curr_time <= target_time <= next_time):
            df_index += 1
            continue

        curr_weight = (next_time - target_time) / (next_time - curr_time)
        next_weight = 1.0 - curr_weight

        curr_position = df_pose.iloc[df_index][position_keys]
        next_position = df_pose.iloc[df_index + 1][position_keys]
        target_position = curr_position * curr_weight + next_position * next_weight

        curr_orientation = df_pose.iloc[df_index][orientation_keys]
        next_orientation = df_pose.iloc[df_index + 1][orientation_keys]
        curr_r = Rotation.from_quat(curr_orientation)
        next_r = Rotation.from_quat(next_orientation)
        slerp = Slerp([curr_time, next_time], Rotation.concatenate([curr_r, next_r]))
        target_orientation = slerp([target_time]).as_quat()[0]

        curr_linear_velocity = df_pose.iloc[df_index][linear_velocity_keys]
        next_linear_velocity = df_pose.iloc[df_index + 1][linear_velocity_keys]
        target_linear_velocity = curr_linear_velocity * curr_weight + next_linear_velocity * next_weight

        curr_angular_velocity = df_pose.iloc[df_index][angular_velocity_keys]
        next_angular_velocity = df_pose.iloc[df_index + 1][angular_velocity_keys]
        curr_r_ang_vel = Rotation.from_rotvec(curr_angular_velocity)
        next_r_ang_vel = Rotation.from_rotvec(next_angular_velocity)
        slerp_ang_vel = Slerp([curr_time, next_time], Rotation.concatenate([curr_r_ang_vel, next_r_ang_vel]))
        target_angular_velocity = slerp_ang_vel([target_time]).as_rotvec()[0]

        data_dict["timestamp"].append(target_time)
        data_dict[position_keys[0]].append(target_position[0])
        data_dict[position_keys[1]].append(target_position[1])
        data_dict[position_keys[2]].append(target_position[2])
        data_dict[orientation_keys[0]].append(target_orientation[0])
        data_dict[orientation_keys[1]].append(target_orientation[1])
        data_dict[orientation_keys[2]].append(target_orientation[2])
        data_dict[orientation_keys[3]].append(target_orientation[3])
        data_dict[linear_velocity_keys[0]].append(target_linear_velocity[0])
        data_dict[linear_velocity_keys[1]].append(target_linear_velocity[1])
        data_dict[linear_velocity_keys[2]].append(target_linear_velocity[2])
        data_dict[angular_velocity_keys[0]].append(target_angular_velocity[0])
        data_dict[angular_velocity_keys[1]].append(target_angular_velocity[1])
        data_dict[angular_velocity_keys[2]].append(target_angular_velocity[2])
        target_index += 1
    return pd.DataFrame(data_dict)

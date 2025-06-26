"""A script to compare two pose_lists."""

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

# cspell: ignore rotvec


def calc_relative_pose(df_prd: pd.DataFrame, df_ref: pd.DataFrame) -> pd.DataFrame:
    """Calculate the relative position and orientation of df_prd with respect to df_ref."""
    position_keys = ["position.x", "position.y", "position.z"]
    orientation_keys = [
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]
    linear_velocity_keys = ["linear_velocity.x", "linear_velocity.y", "linear_velocity.z"]
    angular_velocity_keys = ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]
    assert len(df_prd) == len(df_ref)

    df_relative = df_prd.copy()

    # Calculate the relative orientation
    rotation_prd = Rotation.from_quat(df_prd[orientation_keys].values)
    rotation_ref = Rotation.from_quat(df_ref[orientation_keys].values)
    df_relative[orientation_keys] = (rotation_prd * rotation_ref.inv()).as_quat()

    # Calculate the relative position
    df_relative[position_keys] = df_prd[position_keys].to_numpy() - df_ref[position_keys].to_numpy()
    # Rotate the relative position of each frame by rotation_true.inv()
    # This makes the relative position based on the pose of df_ref
    df_relative[position_keys] = rotation_ref.inv().apply(
        df_relative[position_keys].values,
    )

    # Add norm
    df_relative["position.norm"] = np.linalg.norm(
        df_relative[position_keys].values,
        axis=1,
    )

    # Add rpy angle
    r = Rotation.from_quat(
        df_relative[["orientation.x", "orientation.y", "orientation.z", "orientation.w"]],
    )
    euler = r.as_euler("xyz", degrees=True)
    df_relative["angle.x"] = euler[:, 0]
    df_relative["angle.y"] = euler[:, 1]
    df_relative["angle.z"] = euler[:, 2]
    df_relative["angle.norm"] = np.linalg.norm(r.as_rotvec(), axis=1)

    if "linear_velocity.x" not in df_prd.columns or "linear_velocity.x" not in df_ref.columns:
        # If linear_velocity is not in the DataFrame, return the relative pose without it
        return df_relative[
            [
                "timestamp",
                *position_keys,
                "position.norm",
                *orientation_keys,
                "angle.x",
                "angle.y",
                "angle.z",
                "angle.norm",
            ]
        ]

    # Add linear velocity
    df_relative[linear_velocity_keys] = (
        df_prd[linear_velocity_keys].to_numpy() - df_ref[linear_velocity_keys].to_numpy()
    )
    df_relative["linear_velocity.norm"] = np.linalg.norm(
        df_relative[linear_velocity_keys].values,
        axis=1,
    )
    # Add angular velocity
    df_relative[angular_velocity_keys] = (
        df_prd[angular_velocity_keys].to_numpy() - df_ref[angular_velocity_keys].to_numpy()
    )
    df_relative["angular_velocity.norm"] = np.linalg.norm(
        df_relative[angular_velocity_keys].values,
        axis=1,
    )

    # Arrange the order of columns
    return df_relative[
        [
            "timestamp",
            *position_keys,
            "position.norm",
            *orientation_keys,
            "angle.x",
            "angle.y",
            "angle.z",
            "angle.norm",
            *linear_velocity_keys,
            "linear_velocity.norm",
            *angular_velocity_keys,
            "angular_velocity.norm",
        ]
    ]

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d


def calc_acceleration_diff(df_prd: pd.DataFrame, df_ref: pd.DataFrame) -> pd.DataFrame:
    """Calculate the difference in linear acceleration between two DataFrames.

    Args:
        df_prd: DataFrame with ["timestamp", "linear.x", "linear.y", "linear.z"] (acceleration)
        df_ref: DataFrame with ["timestamp", "linear_velocity.x", "linear_velocity.y", "linear_velocity.z"]

    Returns:
        DataFrame with difference calculations including "diff.norm" column

    Raises:
        ValueError: If input data is insufficient or missing required columns
    """
    # Validate input data
    required_prd_cols = ["timestamp", "linear.x", "linear.y", "linear.z"]
    required_ref_cols = ["timestamp", "linear_velocity.x", "linear_velocity.y", "linear_velocity.z"]

    if not all(col in df_prd.columns for col in required_prd_cols):
        missing = [col for col in required_prd_cols if col not in df_prd.columns]
        raise ValueError(f"Missing required columns in predicted data: {missing}")

    if not all(col in df_ref.columns for col in required_ref_cols):
        missing = [col for col in required_ref_cols if col not in df_ref.columns]
        raise ValueError(f"Missing required columns in reference data: {missing}")

    df_prd_acc = df_prd[required_prd_cols].copy()
    df_ref_vel = df_ref[required_ref_cols].copy()

    # Calculate acceleration from reference velocity (numerical differentiation)
    df_ref_vel = df_ref_vel.sort_values("timestamp")
    dt = (df_ref_vel["timestamp"].diff() / 1e9)  # Convert to seconds

    # Protect against division by zero and very small time differences
    MIN_DT_THRESHOLD = 1e-6  # Minimum time difference threshold (1 microsecond)
    dt = dt.where(dt.abs() > MIN_DT_THRESHOLD, np.nan)

    df_ref_acc = df_ref_vel.copy()
    df_ref_acc["linear_velocity.x"] = df_ref_vel["linear_velocity.x"].diff() / dt
    df_ref_acc["linear_velocity.y"] = df_ref_vel["linear_velocity.y"].diff() / dt
    df_ref_acc["linear_velocity.z"] = df_ref_vel["linear_velocity.z"].diff() / dt
    df_ref_acc = df_ref_acc.dropna()  # Remove rows with NaN

    # Rename columns to match acceleration data
    df_ref_acc = df_ref_acc.rename(
        columns={
            "linear_velocity.x": "linear.x",
            "linear_velocity.y": "linear.y",
            "linear_velocity.z": "linear.z",
        }
    )

    # Interpolate reference acceleration to match timestamp of measured acceleration

    # Remove duplicate timestamps and sort
    df_ref_acc = df_ref_acc.drop_duplicates(subset=["timestamp"]).sort_values("timestamp")
    df_prd_acc = df_prd_acc.drop_duplicates(subset=["timestamp"]).sort_values("timestamp")

    min_timestamp = max(df_ref_acc["timestamp"].min(), df_prd_acc["timestamp"].min())
    max_timestamp = min(df_ref_acc["timestamp"].max(), df_prd_acc["timestamp"].max())

    df_ref_acc = df_ref_acc[
        (min_timestamp <= df_ref_acc["timestamp"]) & (df_ref_acc["timestamp"] <= max_timestamp)
    ]
    df_prd_acc = df_prd_acc[
        (min_timestamp <= df_prd_acc["timestamp"]) & (df_prd_acc["timestamp"] <= max_timestamp)
    ]

    # Validate data availability for interpolation
    if len(df_ref_acc) <= 1:
        raise ValueError(
            f"Insufficient reference acceleration data points: {len(df_ref_acc)}. "
            "Need at least 2 points for interpolation."
        )
    if len(df_prd_acc) <= 1:
        raise ValueError(
            f"Insufficient predicted acceleration data points: {len(df_prd_acc)}. "
            "Need at least 2 points for interpolation."
        )

    interp_x = interp1d(
        df_ref_acc["timestamp"],
        df_ref_acc["linear.x"],
        kind="linear",
        fill_value="extrapolate",
    )
    interp_y = interp1d(
        df_ref_acc["timestamp"],
        df_ref_acc["linear.y"],
        kind="linear",
        fill_value="extrapolate",
    )
    interp_z = interp1d(
        df_ref_acc["timestamp"],
        df_ref_acc["linear.z"],
        kind="linear",
        fill_value="extrapolate",
    )

    # Interpolate reference acceleration at measured timestamps
    df_result = df_prd_acc.copy()
    df_result["reference.linear.x"] = interp_x(df_result["timestamp"])
    df_result["reference.linear.y"] = interp_y(df_result["timestamp"])
    df_result["reference.linear.z"] = interp_z(df_result["timestamp"])

    # Calculate differences between measured and reference acceleration
    df_result["diff.x"] = df_prd_acc["linear.x"] - df_result["reference.linear.x"]
    df_result["diff.y"] = df_prd_acc["linear.y"] - df_result["reference.linear.y"]
    df_result["diff.z"] = df_prd_acc["linear.z"] - df_result["reference.linear.z"]
    df_result["diff.norm"] = np.sqrt(
        df_result["diff.x"] ** 2 + df_result["diff.y"] ** 2 + df_result["diff.z"] ** 2
    )
    return df_result

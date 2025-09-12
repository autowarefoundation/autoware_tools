#!/usr/bin/env python3
"""A script to check rise/fall of diagnostics flags."""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import pandas as pd
import yaml


@dataclass
class CriteriaConfig:
    tsv_path: Path
    diagnostics_key: str
    target_time_ns: int
    flag: str = "rise"
    time_window_ns: int = 200_000_000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_file", type=Path, default=None)
    parser.add_argument("diagnostics_result_dir", type=Path, default=None)
    return parser.parse_args()


def load_diagnostics_flag_check(yaml_path: Path):
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        conditions = data["Evaluation"]["Conditions"]
    except Exception:
        return None

    return conditions.get("DiagnosticsFlagCheck", None)


def check_pose_no_update_count(cfg: CriteriaConfig) -> bool:
    if not cfg.tsv_path.exists():
        print(f"TSV file not found for {cfg.diagnostics_key}: {cfg.tsv_path}")
        return False
    df = pd.read_csv(cfg.tsv_path, sep="\t")

    required_cols = [
        "pose_no_update_count",
        "pose_no_update_count_threshold_error",
        "timestamp_header",
        "message",
    ]
    error_message = "[ERROR]pose is not updated"

    for col in required_cols:
        if col not in df.columns:
            print(f"Missing required column: {col}")
            return False

    # Convert timestamps into integer nanoseconds
    df["timestamp_ns"] = pd.to_numeric(df["timestamp_header"], errors="coerce")

    # Define ranges
    start = cfg.target_time_ns - cfg.time_window_ns
    end = cfg.target_time_ns + cfg.time_window_ns

    df_numeric = df.dropna(subset=required_cols)

    # Split into pre-window, in-window
    df_pre = df_numeric[df_numeric["timestamp_ns"] < start]
    df_window = df_numeric[
        (df_numeric["timestamp_ns"] >= start) & (df_numeric["timestamp_ns"] <= end)
    ]

    # Define conditions
    violation = (
        df_numeric["pose_no_update_count"] >= df_numeric["pose_no_update_count_threshold_error"]
    ) & (df_numeric["message"].str.contains(error_message, na=False, regex=False))
    ok = ~violation

    if cfg.flag == "rise":
        # Expect violation inside the window, but *no violation* before it
        if violation[df_pre.index].any():
            print(f"{cfg.diagnostics_key}: Unexpected violation before window.")
            return False
        df_condition = df_window[violation[df_window.index]]
    elif cfg.flag == "fall":
        # Expect OK inside the window, but *only violation* before it
        if ok[df_pre.index].any():
            print(f"{cfg.diagnostics_key}: Unexpected OK state before window.")
            return False
        df_condition = df_window[ok[df_window.index]]
    else:
        print("Invalid flag is set")
        return False

    if df_condition.empty:
        print(f"{cfg.diagnostics_key}: No matching rows found in window.")
        return False

    print(f"{cfg.diagnostics_key}: Matching rows from {cfg.tsv_path}:")
    print(df_condition["pose_no_update_count"].iloc[0])
    return True


def check_pose_is_passed_delay_gate(cfg: CriteriaConfig) -> bool:
    if not cfg.tsv_path.exists():
        print(f"TSV file not found for {cfg.diagnostics_key}: {cfg.tsv_path}")
        return False
    df = pd.read_csv(cfg.tsv_path, sep="\t")

    # Ensure columns exist
    required_cols = [
        "pose_is_passed_delay_gate",
        "timestamp_header",
        "message",
    ]

    error_message = "[WARN]pose topic is delay"

    for col in required_cols:
        if col not in df.columns:
            print(f"Missing required column: {col}")
            return False

    # Convert timestamps into integer nanoseconds
    df["pose_is_passed_delay_gate"] = df["pose_is_passed_delay_gate"].astype(bool)
    df["timestamp_ns"] = pd.to_numeric(df["timestamp_header"], errors="coerce")

    # Convert target into window range
    start = cfg.target_time_ns - cfg.time_window_ns
    end = cfg.target_time_ns + cfg.time_window_ns

    df_pre = df[df["timestamp_ns"] < start]
    df_window = df[(df["timestamp_ns"] >= start) & (df["timestamp_ns"] <= end)]

    if cfg.flag == "rise":
        # Expect False + no False before window
        if (~df_pre["pose_is_passed_delay_gate"]).any():
            print(f"{cfg.diagnostics_key}: Unexpected False before window.")
            return False
        df_condition = df_window[
            (~df_window["pose_is_passed_delay_gate"])
            & (df_window["message"].str.contains(error_message, na=False, regex=False))
        ]
    elif cfg.flag == "fall":
        # Expect True + no True before window
        if (df_pre["pose_is_passed_delay_gate"]).any():
            print(f"{cfg.diagnostics_key}: Unexpected True before window.")
            return False
        df_condition = df_window[
            (df_window["pose_is_passed_delay_gate"])
            & (~df_window["message"].str.contains(error_message, na=True, regex=False))
        ]
    else:
        print("Invalid flag is set")
        return False

    if df_condition.empty:
        print(f"{cfg.diagnostics_key}: No matching rows found.")
        return False

    print(f"{cfg.diagnostics_key}: Matching rows from {cfg.tsv_path} at:")
    print(df_condition["timestamp_header"].iloc[0])
    return True


def check_is_initial_pose_reliable(cfg: CriteriaConfig) -> bool:
    if not cfg.tsv_path.exists():
        print(f"TSV file not found for {cfg.diagnostics_key}: {cfg.tsv_path}")
        return False
    df = pd.read_csv(cfg.tsv_path, sep="\t", dtype={"is_initial_pose_reliable": "boolean"})

    # We will just see the final row for pose_initializer diagnostics. In other words, the timing is not considered

    is_initial_pose_reliable = df.iloc[-1]["is_initial_pose_reliable"]  # get the last result
    if (cfg.flag == "rise" and is_initial_pose_reliable) or (
        cfg.flag == "fall" and not is_initial_pose_reliable
    ):
        return True

    print("pose_initializer didn't work as expected.")
    return False


def main(scenario_file: Path, diagnostics_result_dir: Path) -> Dict[str, bool]:
    diagnostics_flag_condition = load_diagnostics_flag_check(scenario_file)

    if not diagnostics_flag_condition:
        print("No DiagnosticsFlagCheck found, exiting.")
        return {}

    results = {key: False for key in diagnostics_flag_condition}
    for key, criteria in diagnostics_flag_condition.items():
        print(f"\nProcessing criteria: {key}")

        flag = criteria.get("flag", "").strip().lower()
        at_sec = int(float(criteria.get("at_sec", 0)))
        at_nanosec = int(float(criteria.get("at_nanosec", 0)))
        target_time = int(at_sec * 1_000_000_000 + at_nanosec)

        if key == "pose_no_update_count":
            tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            criteria_cfg = CriteriaConfig(
                tsv_path=tsv_path, diagnostics_key=key, flag=flag, target_time_ns=target_time
            )
            results[key] = check_pose_no_update_count(criteria_cfg)
        elif key == "pose_is_passed_delay_gate":
            tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            criteria_cfg = CriteriaConfig(
                tsv_path=tsv_path, diagnostics_key=key, flag=flag, target_time_ns=target_time
            )
            results[key] = check_pose_is_passed_delay_gate(criteria_cfg)
        elif key == "is_initial_pose_reliable":
            tsv_path = diagnostics_result_dir / "pose_initializer__pose_initializer_status.tsv"
            criteria_cfg = CriteriaConfig(
                tsv_path=tsv_path, diagnostics_key=key, flag=flag, target_time_ns=target_time
            )
            results[key] = check_is_initial_pose_reliable(criteria_cfg)
        else:
            print(f"Flag checking for {key} not defined!!")
    return results


if __name__ == "__main__":
    args = parse_args()
    scenario_file = args.scenario_file
    diagnostics_result_dir = args.diagnostics_result_dir

    main(scenario_file, diagnostics_result_dir)

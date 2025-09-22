#!/usr/bin/env python3
"""A script to check rise/fall of diagnostics flags."""

from abc import abstractmethod
import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict
from typing import Optional

import pandas as pd
import yaml


@dataclass
class CriteriaConfig:
    tsv_path: Path = "default.tsv"
    diagnostics_key: str = ""
    flag: str = "rise"
    target_time_ns: Optional[int] = None
    timestamp_label = "timestamp_header"
    time_window_ns: int = 500_000_000


class BaseChecker:
    def __init__(self, cfg: CriteriaConfig):
        self.cfg = cfg
        if not self.cfg.tsv_path.exists():
            print(f"TSV file not found for {self.cfg.diagnostics_key}: {self.cfg.tsv_path}")
            return False
        self.df = pd.read_csv(cfg.tsv_path, sep="\t")

    @abstractmethod
    def is_positive(self, series: pd.Series) -> bool:
        pass

    def any_positive_in_window(self) -> bool:
        start = self.cfg.target_time_ns - self.cfg.time_window_ns
        end = self.cfg.target_time_ns + self.cfg.time_window_ns

        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[(timestamp_ns >= start) & (timestamp_ns <= end)]

        for _, row in df_window.iterrows():
            if self.is_positive(row):
                return True

        return False

    def any_negative_in_window(self) -> bool:
        start = self.cfg.target_time_ns - self.cfg.time_window_ns
        end = self.cfg.target_time_ns + self.cfg.time_window_ns

        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[(timestamp_ns >= start) & (timestamp_ns <= end)]

        for _, row in df_window.iterrows():
            if not self.is_positive(row):
                return True

        return False

    def all_positive_before_window(self) -> bool:
        start = self.cfg.target_time_ns - self.cfg.time_window_ns
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[(timestamp_ns >= 0) & (timestamp_ns <= start)]

        for _, row in df_window.iterrows():
            if not self.is_positive(row):
                return False

        return True

    def all_negative_before_window(self) -> bool:
        start = self.cfg.target_time_ns - self.cfg.time_window_ns
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[(timestamp_ns >= 0) & (timestamp_ns <= start)]

        for _, row in df_window.iterrows():
            if self.is_positive(row):
                return False

        return True

    def check(self) -> bool:
        if self.cfg.flag == "rise":
            if not self.any_positive_in_window():
                print("    Not positive around target time.")
                return False
            if not self.all_negative_before_window():
                print("    Found positive before target time.")
                return False
            return True
        else:
            if not self.any_negative_in_window():
                print("    Not negative around target time.")
                return False
            if not self.all_positive_before_window():
                print("    Found negative before target time.")
                return False
            return True


class ImuTimeStampDtChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        stamp_label = "imu_time_stamp_dt"
        if level_label in series and stamp_label in series:
            return series[level_label] == 2 and series[stamp_label] >= 0.2
        else:
            return False


class VehicleTwistTimeStampDtChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        stamp_label = "vehicle_twist_time_stamp_dt"
        if level_label in series and stamp_label in series:
            return series[level_label] == 2 and series[stamp_label] >= 0.2
        else:
            return False


class PoseNoUpdateCountChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        count_label = "pose_no_update_count"
        threshold_label = "pose_no_update_count_threshold_error"
        if level_label in series and count_label in series and threshold_label in series:
            return series[level_label] == 2 and series[count_label] >= series[threshold_label]
        else:
            return False


class PoseIsPassedDelayGateChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        is_passed_label = "pose_is_passed_delay_gate"
        if level_label in series and is_passed_label in series:
            return series[level_label] == 2 and not series[is_passed_label]
        else:
            return False


class IsInitialPoseReliableChecker(BaseChecker):
    # Override check() since this diagnostics is special
    def check(self):
        is_initial_pose_reliable = self.df.iloc[-1]["is_initial_pose_reliable"]
        if (self.cfg.flag == "rise" and not is_initial_pose_reliable) or (
            self.cfg.flag == "fall" and is_initial_pose_reliable
        ):
            return True
        else:
            return False


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


def main(scenario_file: Path, diagnostics_result_dir: Path) -> Dict[str, bool]:
    diagnostics_flag_condition = load_diagnostics_flag_check(scenario_file)

    if not diagnostics_flag_condition:
        print("No DiagnosticsFlagCheck found, exiting.")
        return {}

    results = {key: False for key in diagnostics_flag_condition}
    for key, criteria in diagnostics_flag_condition.items():
        print(f"Processing diagnostics: {key}")

        criteria_cfg = CriteriaConfig()

        criteria_cfg.key = key
        criteria_cfg.flag = criteria.get("flag", "").strip().lower()

        at_sec = int(float(criteria.get("at_sec", 0)))
        at_nanosec = int(float(criteria.get("at_nanosec", 0)))
        criteria_cfg.target_time_ns = int(at_sec * 1_000_000_000 + at_nanosec)

        if not (criteria_cfg.flag == "rise" or criteria_cfg.flag == "fall"):
            print("    Invalid flag found!")
            results[key] = False
        elif key == "pose_no_update_count":
            criteria_cfg.tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            results[key] = PoseNoUpdateCountChecker(criteria_cfg).check()
            # results[key] = check_pose_no_update_count(criteria_cfg)
        elif key == "pose_is_passed_delay_gate":
            criteria_cfg.tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            results[key] = PoseIsPassedDelayGateChecker(criteria_cfg).check()
            # results[key] = check_pose_is_passed_delay_gate(criteria_cfg)
        elif key == "is_initial_pose_reliable":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "pose_initializer__pose_initializer_status.tsv"
            )
            results[key] = IsInitialPoseReliableChecker(criteria_cfg).check()
            # results[key] = check_is_initial_pose_reliable(criteria_cfg)
        elif key == "imu_time_stamp_dt":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "gyro_odometer__gyro_odometer_status.tsv"
            )
            results[key] = ImuTimeStampDtChecker(criteria_cfg).check()
        elif key == "vehicle_twist_time_stamp_dt":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "gyro_odometer__gyro_odometer_status.tsv"
            )
            results[key] = VehicleTwistTimeStampDtChecker(criteria_cfg).check()
        else:
            print(f"Flag checking for {key} not defined!!")

        print(f"    {results[key]}")
    return results


if __name__ == "__main__":
    args = parse_args()
    scenario_file = args.scenario_file
    diagnostics_result_dir = args.diagnostics_result_dir

    main(scenario_file, diagnostics_result_dir)

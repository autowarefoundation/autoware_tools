#!/usr/bin/env python3
"""A script to check rise/fall of diagnostics flags."""

from abc import abstractmethod
import argparse
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable
from typing import Dict
from typing import List

import pandas as pd
import yaml


def window_condition_before(t, target_time):
    return t <= target_time


def window_condition_after(t, target_time):
    return t >= target_time


def window_condition_at(t, start_time, end_time):
    return t >= start_time & t <= end_time


class TimingType(Enum):
    BEFORE = "before"
    AFTER = "after"
    AT = "at"


class FlagType(Enum):
    POSITIVE = "positive"
    NEGATIVE = "negative"


@dataclass
class Flag:
    flag_type: FlagType
    timing: TimingType
    sec: int
    nanosec: int
    window: int
    entirely: bool


@dataclass
class CriteriaConfig:
    diagnostics_key: str
    flags: List[Flag]
    tsv_path: Path = "default.tsv"
    timestamp_label = "timestamp_header"


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

    def is_any_positive_in_window(self, window_condition: Callable[[pd.Series], pd.Series]) -> bool:
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[window_condition(timestamp_ns)]

        for _, row in df_window.iterrows():
            if self.is_positive(row):
                return True

        return False

    def is_any_negative_in_window(self, window_condition: Callable[[pd.Series], pd.Series]) -> bool:
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[window_condition(timestamp_ns)]

        for _, row in df_window.iterrows():
            if not self.is_positive(row):
                return True

        return False

    def is_all_positive_in_window(self, window_condition: Callable[[pd.Series], pd.Series]) -> bool:
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[window_condition(timestamp_ns)]

        for _, row in df_window.iterrows():
            if not self.is_positive(row):
                return False

        return True

    def is_all_negative_in_window(self, window_condition: Callable[[pd.Series], pd.Series]) -> bool:
        df_valid = self.df.dropna(subset=[self.cfg.timestamp_label])
        df_valid = df_valid.sort_values(by=self.cfg.timestamp_label)
        timestamp_ns = pd.to_numeric(df_valid[self.cfg.timestamp_label], errors="coerce")
        df_window = df_valid[window_condition(timestamp_ns)]

        for _, row in df_window.iterrows():
            if self.is_positive(row):
                return False

        return True

    def check(self) -> bool:
        entire_result = True
        for flag in self.cfg.flags:
            target_time = flag.sec * 1_000_000_000 + flag.nanosec
            if flag.timing == TimingType.BEFORE.value:

                def window_condition(t: int) -> bool:
                    return t <= target_time

            elif flag.timing == TimingType.AFTER.value:

                def window_condition(t: int) -> bool:
                    return t >= target_time

            elif flag.timing == TimingType.AT.value:
                start_time = target_time - flag.window
                end_time = target_time + flag.window

                def window_condition(t: int) -> bool:
                    return (t >= start_time) & (t <= end_time)

            else:
                print("Invalid timing type found!!")
                return False

            if flag.flag_type == FlagType.POSITIVE.value and flag.entirely:
                partial_result = self.is_all_positive_in_window(window_condition)
            elif flag.flag_type == FlagType.POSITIVE.value and not flag.entirely:
                partial_result = self.is_any_positive_in_window(window_condition)
            elif flag.flag_type == FlagType.NEGATIVE.value and flag.entirely:
                partial_result = self.is_all_negative_in_window(window_condition)
            elif flag.flag_type == FlagType.NEGATIVE.value and not flag.entirely:
                partial_result = self.is_any_negative_in_window(window_condition)
            else:
                print("Invalid flag type found!!")
                return False

            if not partial_result:
                print(
                    f'{flag.flag_type} flag of {self.cfg.diagnostics_key} with timing "{flag.timing}" of {flag.sec}.{flag.nanosec} failed.'
                )

            entire_result &= partial_result

        return entire_result


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
    # Override check() since this diagnostics is an one shot
    def check(self):
        is_initial_pose_reliable = self.df.iloc[-1]["is_initial_pose_reliable"] & (
            self.df.iloc[-1]["level"] != 2
        )
        if (
            self.cfg.flags[0].flag_type == FlagType.POSITIVE.value and not is_initial_pose_reliable
        ) or (self.cfg.flags[0].flag_type == FlagType.NEGATIVE.value and is_initial_pose_reliable):
            return True
        else:
            return False


class NearestVoxelTransformationLikelihoodChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        nvtl_label = "nearest_voxel_transformation_likelihood"
        if level_label in series and nvtl_label in series:
            return series[level_label] >= 1 and series[nvtl_label] < 2.3
        else:
            return False


class LocalizationErrorEllipseChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        error_ellipse_label = "localization_error_ellipse"
        lateral_error_ellipse_label = "localization_error_ellipse_lateral_direction"
        if level_label in series and error_ellipse_label in series:
            return (series[level_label] == 2 and series[error_ellipse_label] >= 1.5) or (
                series[level_label] == 2 and series[lateral_error_ellipse_label] >= 0.3
            )
        else:
            return False


class PoseInstabilityChecker(BaseChecker):
    def is_positive(self, series):
        level_label = "level"
        status_label = [
            "diff_position_x:status",
            "diff_position_y:status",
            "diff_position_z:status",
            "diff_angle_x:status",
            "diff_angle_y:status",
            "diff_angle_z:status",
        ]
        validation_label = [
            "diff_position_x:validation_enabled",
            "diff_position_y:validation_enabled",
            "diff_position_z:validation_enabled",
            "diff_angle_x:validation_enabled",
            "diff_angle_y:validation_enabled",
            "diff_angle_z:validation_enabled",
        ]

        if (
            level_label in series
            and set(status_label).issubset(series.index)
            and set(validation_label).issubset(series.index)
        ):
            all_ok = True
            for s, v in zip(status_label, validation_label):
                if series[v]:  # validation_enabled == True
                    if series[s] != "OK":
                        all_ok = False
                        break
            return not all_ok and series[level_label] == 2
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
    except Exception as e:
        print(type(e).__name__, e)
        return None

    print("Read scenario file.")
    return conditions.get("DiagnosticsFlagCheck", None)


def parse_flags(flags_yaml: Dict) -> List[Flag]:
    flags: List[Flag] = []
    for name, flag_yaml in flags_yaml.items():
        flag = Flag(
            flag_type=flag_yaml.get("type", ""),
            timing=flag_yaml.get("timing", ""),
            sec=int(float(flag_yaml.get("sec", 0))),
            nanosec=int(float(flag_yaml.get("nanosec", 0))),
            window=int(float(flag_yaml.get("window", 500_000_000))),
            entirely=flag_yaml.get("entirely", False),
        )
        flags.append(flag)

    return flags


def main(scenario_file: Path, diagnostics_result_dir: Path) -> Dict[str, bool]:
    diagnostics_flag_condition = load_diagnostics_flag_check(scenario_file)

    if not diagnostics_flag_condition:
        print("No DiagnosticsFlagCheck found, exiting.")
        return {}

    results = {key: False for key in diagnostics_flag_condition}
    for key, flags_yaml in diagnostics_flag_condition.items():
        print(f"Processing diagnostics: {key}")
        criteria_cfg = CriteriaConfig(diagnostics_key=key, flags=parse_flags(flags_yaml))
        if key == "pose_no_update_count":
            criteria_cfg.tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            results[key] = PoseNoUpdateCountChecker(criteria_cfg).check()
        elif key == "pose_is_passed_delay_gate":
            criteria_cfg.tsv_path = diagnostics_result_dir / "localization__ekf_localizer.tsv"
            results[key] = PoseIsPassedDelayGateChecker(criteria_cfg).check()
        elif key == "is_initial_pose_reliable":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "pose_initializer__pose_initializer_status.tsv"
            )
            results[key] = IsInitialPoseReliableChecker(criteria_cfg).check()
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
        elif key == "nearest_voxel_transformation_likelihood":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "ndt_scan_matcher__scan_matching_status.tsv"
            )
            results[key] = NearestVoxelTransformationLikelihoodChecker(criteria_cfg).check()
        elif key == "localization_error_ellipse":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "localization_error_monitor__ellipse_error_status.tsv"
            )
            results[key] = LocalizationErrorEllipseChecker(criteria_cfg).check()
        elif key == "pose_instability":
            criteria_cfg.tsv_path = (
                diagnostics_result_dir / "localization__pose_instability_detector.tsv"
            )
            results[key] = PoseInstabilityChecker(criteria_cfg).check()
        else:
            print(f"Flag checking for {key} not defined!!")

        print(f"    {results[key]}")
    return results


if __name__ == "__main__":
    args = parse_args()
    scenario_file = args.scenario_file
    diagnostics_result_dir = args.diagnostics_result_dir

    main(scenario_file, diagnostics_result_dir)

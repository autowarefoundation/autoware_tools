# NDT Evaluation

This directory contains tools for evaluating the performance of the NDT localization.

## Step1: Record rosbag

```bash
ros2 bag record \
  /localization/kinematic_state \
  /localization/util/downsample/pointcloud \
  /sensing/vehicle_velocity_converter/twist_with_covariance \
  /sensing/imu/imu_data \
  /tf_static \
  /sensing/gnss/pose_with_covariance \
  /initialpose
```

## Step2: Apply `convert_rosbag_for_ndt_evaluation.py`

- Check whether the necessary topics are available
- Extract only the necessary topics
- Change the topic name of the reference trajectory

```bash
python3 convert_rosbag_for_ndt_evaluation.py /path/to/recorded_rosbag
```

## Step3: Execute DrivingLogReplayer

Setup [DrivingLogReplayer](https://tier4.github.io/driving_log_replayer/quick_start/installation/) and copy the converted rosbag (input_bad) to `~/driving_log_replayer_data/localization`

```bash
~/driving_log_replayer_data/localization$ tree -L 2
.
├── evaluation_sample
│   ├── input_bag
│   └── scenario.yaml
└── sample
    ├── input_bag
    └── scenario.yaml

4 directories, 2 files
```

Then execute.

```bash
dlr simulation run -p localization -l "play_rate:=0.5"
```

```bash
<< show test result >>
test case 1 / 2 : use case: evaluation_sample
--------------------------------------------------
TestResult: Passed
Passed: Convergence (Success): 964 / 964 -> 100.00%, Reliability (Success): NVTL Sequential NG Count: 0 (Total Test: 974, Average: 3.07964, StdDev: 0.09657), NDT Availability (Success): NDT available
test case 2 / 2 : use case: sample
--------------------------------------------------
TestResult: Passed
Passed: Convergence (Success): 294 / 295 -> 99.66%, Reliability (Success): NVTL Sequential NG Count: 0 (Total Test: 295, Average: 2.47750, StdDev: 0.04174), NDT Availability (Success): NDT available
```
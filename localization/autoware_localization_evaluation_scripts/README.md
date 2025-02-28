# autoware_localization_evaluation_scripts

This package contains some scripts for evaluating the localization of Autoware.

Ths scripts are used for the result rosbags of localization, particularly the result_bag from `driving_log_replayer_v2`.

<https://tier4.github.io/driving_log_replayer_v2/quick_start/setup/>

As a test, execute `driving_log_replayer_v2` with the following command:

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
    scenario_path:=$HOME/driving_log_replayer_v2/localization.yaml
```

Then, use the scripts for processing the result_bag:

```bash
$HOME/driving_log_replayer_v2/out/latest/result_bag
```

## plot_diagnostics.py

```bash
ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py \
   <rosbag_path> \
   --save_dir=/your/path (default:rosbag_path/../)
```

[Example]

```bash
$ ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py \
    $HOME/driving_log_replayer_v2/out/latest/result_bag
[INFO] [1740561002.740748735] [rosbag2_storage]: Opened database '$HOME/driving_log_replayer_v2/out/latest/result_bag/result_bag_0.db3' for READ_ONLY.
save_dir=PosixPath('$HOME/driving_log_replayer_v2/out/latest/diagnostics_result')
```

This command outputs each diagnostic (tsv) and plot result (png).

```bash
$ tree $HOME/driving_log_replayer_v2/out/latest/diagnostics_result
$HOME/driving_log_replayer_v2/out/latest/diagnostics_result
├── gyro_bias_validator__gyro_bias_validator.png
├── gyro_bias_validator__gyro_bias_validator.tsv
├── localization__ekf_localizer.png
├── localization__ekf_localizer.tsv
├── localization__pose_instability_detector.png
├── localization__pose_instability_detector.tsv
├── localization_error_monitor__ellipse_error_status.png
├── localization_error_monitor__ellipse_error_status.tsv
├── ndt_scan_matcher__scan_matching_status.png
└── ndt_scan_matcher__scan_matching_status.tsv

0 directories, 10 files
```

[Example : ndt_scan_matcher__scan_matching_status.png]

![ndt_scan_matcher__scan_matching_status.png](./media/ndt_scan_matcher__scan_matching_status.png)

## extract_pose_from_rosbag.py

```bash
ros2 run autoware_localization_evaluation_scripts extract_pose_from_rosbag.py \
   <rosbag_path> \
   --save_dir=/your/path (default:rosbag_path/../)
```

[Example]

```bash
$ ros2 run autoware_localization_evaluation_scripts extract_pose_from_rosbag.py \
    $HOME/driving_log_replayer_v2/out/latest/result_bag \
    --target_topics "/localization/kinematic_state" \
                    "/localization/pose_estimator/pose_with_covariance"
```

This command outputs tsv files for each pose.

The file names are the topic names with “/” replaced with “\_\_”.

```bash
$ tree $HOME/driving_log_replayer_v2/out/latest/pose_tsv
$HOME/driving_log_replayer_v2/out/latest/pose_tsv
├── localization__kinematic_state.tsv
└── localization__pose_estimator__pose_with_covariance.tsv

0 directories, 2 files
```

## compare_trajectories.py

```bash
ros2 run autoware_localization_evaluation_scripts compare_trajectories.py \
   <tsv_file_subject> <tsv_file_reference>
```

[Example]

```bash
$ ros2 run autoware_localization_evaluation_scripts compare_trajectories.py \
    $HOME/driving_log_replayer_v2/out/latest/pose_tsv/localization__kinematic_state.tsv \
    $HOME/driving_log_replayer_v2/out/latest/pose_tsv/localization__pose_estimator__pose_with_covariance.tsv
```

This command outputs tsv files for each pose.

```bash
$ tree $HOME/driving_log_replayer_v2/out/latest/pose_tsv/localization__kinematic_state_result
$HOME/driving_log_replayer_v2/out/latest/pose_tsv/localization__kinematic_state_result
├── compare_trajectories.png
├── relative_pose.png
├── relative_pose.tsv
└── relative_pose_summary.tsv

0 directories, 4 files
```

## analyze_rosbags_parallel.py

```bash
ros2 run autoware_localization_evaluation_scripts analyze_rosbags_parallel.py \
   <result_dir> --parallel_num 2
```

[Example]

```bash
$ ros2 run autoware_localization_evaluation_scripts analyze_rosbags_parallel.py \
    $HOME/driving_log_replayer_v2/out/ \
    --parallel_num 2 \
    --topic_subject "/localization/kinematic_state" \
    --topic_reference "/localization/pose_estimator/pose_with_covariance"
```

This command performs the above three analyses on the subdirectories of the target directory.

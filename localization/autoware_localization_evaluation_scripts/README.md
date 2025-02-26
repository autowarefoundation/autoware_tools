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
ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py <rosbag_path>
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

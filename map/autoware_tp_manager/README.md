# autoware_tp_manager

Here are some tools for collecting average TPs of PCD maps. Currently, we consider the decrease of TPs as a sign of map decay. However, we don't know what TPs are 'abnormal', e.g. in some areas the TPs range around 2.0 ~ 3.0, while in others TPs float around 5.0. This package provides some tools to check it, including:

- TP collector: collect the average TPs per segment of a PCD map
- TP checker: compare a rosbag's TPs with a map's TPs and highlight the map areas where the rosbag's TPs differ significantly from the map's TPs.

## Installation

```bash
cd <PATH_TO_pilot-auto.*> # OR <PATH_TO_autoware>
cd src/
git clone git@github.com:autowarefoundation/autoware_tools.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --symlink-install --packages-up-to autoware_pointcloud_merger
```

## Usage

- Collect the average TPs per segment from a map by TP_collector

  ```bash
  ros2 run autoware_tp_manager tp_collector.py <path_to_pcd_dir> <path_to_rosbag> <path_to_output_dir> [--resolution <resolution>] [--pose_topic <topic_of_poses>] [--tp_topic <topic_of_TPs>] [--scan_topic <topic_of_scans>]
  ```

  | Name               | Description                                                                  |
  | ------------------ | ---------------------------------------------------------------------------- |
  | path_to_pcd_dir    | Directory that contains the input PCD files                                  |
  | path_to_rosbag     | Path to the input rosbag                                                     |
  | path_to_output_dir | Path to the output directory                                                 |
  | resolution         | Resolution to segment the input PCD. The TPs are collected on these segments |
  | topic_of_poses     | Topic of poses messages in the input rosbag                                  |
  | topic_of_TPs       | Topic of TPs in the input rosbag                                             |
  | topic_of_scans     | Topic of downsampled scans in the input rosbag                               |

  Paths to folders should be specified as **absolute paths**.

  The rosbag should contain the following topics

  - /localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias
  - /localization/pose_estimator/transform_probability
  - /localization/util/downsample/pointcloud

  The average TPs can be visualized on Rviz2 by running the following command

  ```bash
  python3 install/autoware_tp_manager/lib/autoware_tp_manager/tp_visualizer.py <path_to_output_dir>
  ```

  | Name               | Description                                  |
  | ------------------ | -------------------------------------------- |
  | path_to_output_dir | Path to the output directory of TP_collector |

  then open another terminal to launch Rviz2 and add the topic /autoware_tp_visualizer.

- Compare a rosbags' TPs with a map's TPs by TP_checker

  ```bash
  ros2 run autoware_tp_manager tp_checker.py <path_to_score_dir> <path_to_rosbag> [--pose_topic <topic_of_poses>] [--tp_topic <topic_of_TPs>] [--scan_topic <topic_of_scans>] [--radius <radius>] [--drop_num <drop_num>]
  ```

  | Name              | Description                                                                                                               |
  | ----------------- | ------------------------------------------------------------------------------------------------------------------------- |
  | path_to_score_dir | Directory that contains the TP file (.csv) and the downsampled PCD map. This is the output directory of the tp_collector. |
  | path_to_rosbag    | Path to the input rosbag to be evaluated                                                                                  |
  | topic_of_poses    | Topic of poses in the evaluation rosbag                                                                                   |
  | topic_of_TPs      | Topic of TPs in the evaluation rosbag                                                                                     |
  | topic_of_scans    | Topic of scans in the evaluation rosbag                                                                                   |
  | radius            | The radius to query map segments in vicinity of poses. This is used when scan data is not available in the rosbag.       |
  | drop_num          | When the number of continuous low-TP poses exceeds this number, the tool stops checking because the localization is not reliable anymore |

  The results of checking are published to the topic /autoware_tp_checker, and can also be displayed on Rviz2. The red points

- The rosbags used for both TP collector and TP checker is created by running Autoware's logging simulator and record the following three topics:
  - /localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias [optional]
  - /localization/pose_estimator/transform_probability
  - /localization/util/downsample/pointcloud

## Parameter

{{ json_to_markdown("map/autoware_tp_manager/schema/tp_manager.schema.json") }}

## LICENSE

This package is under [Apache License 2.0](../../LICENSE)

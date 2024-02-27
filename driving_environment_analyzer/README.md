# Driving Environment Analyzer

このツールはROSBAGに含まれる走行履歴を元に走行環境のODDを解析するツールです。

## How to use

現在以下の情報が出力可能です。

- 走行経路の長さ
- 走行経路の車線情報
- 走行経路の最大・最小勾配
- 走行経路の最大曲率
- 走行経路の最大・最小車線幅
- 交差点の有無
- 信号機の有無
- 横断歩道の有無

起動時に`bag_path`オプションで解析したいROSBAGを指定してください。（ディレクトリの指定も.db3ファイルの直接指定もサポートしています。）

解析に必要なtopicは以下のとおりです。（今後増える可能性もあります。）

- `/planning/mission_planning/route`
- `/map/vector_map`

以下のようにlaunchすることでODDの解析結果が得られます。

`ros2 launch driving_environment_analyzer driving_environment_analyzer.launch.xml use_map_in_bag:=true bag_path:=<ROSBAG>`

```bash
[component_container-1] [INFO 1708999777.768870564] [driving_environment_analyzer]: ======================================
[component_container-1] [INFO 1708999777.768922452] [driving_environment_analyzer]:  data is ready. start ODD analysis...
[component_container-1] [INFO 1708999777.768933574] [driving_environment_analyzer]: ======================================
[component_container-1] [INFO 1708999777.768967412] [driving_environment_analyzer]: - Length of total lanes : 2357.50 [m]
[component_container-1] [INFO 1708999777.769031174] [driving_environment_analyzer]: - Length of lane that has adjacent lane : 2080.43 [m]
[component_container-1] [INFO 1708999777.769076141] [driving_environment_analyzer]: - Length of lane that has opposite lane : 0.00 [m]
[component_container-1] [INFO 1708999777.769101793] [driving_environment_analyzer]: - Length of lane that has no adjacent lane : 277.07 [m]
[component_container-1] [INFO 1708999777.769225729] [driving_environment_analyzer]: - Min lane width: 3.14 [m] Max lane width: 4.94 [m]
[component_container-1] [INFO 1708999777.769278698] [driving_environment_analyzer]: - Max curvature: 0.007967 [1/m]
[component_container-1] [INFO 1708999777.769293161] [driving_environment_analyzer]: - Min curve radius: 125.52 [m]
[component_container-1] [INFO 1708999777.769336094] [driving_environment_analyzer]: - Min elevation angle: -0.033037 [rad] Max elevation angle: 0.026073 [rad]
[component_container-1] [INFO 1708999777.769403870] [driving_environment_analyzer]: - Min speed limit: 13.89 [m/s] Max speed limit: 16.67 [m/s]
[component_container-1] [INFO 1708999777.769424648] [driving_environment_analyzer]: - Exist traffic light: true
[component_container-1] [INFO 1708999777.769435813] [driving_environment_analyzer]: - Exist intersection: true
[component_container-1] [INFO 1708999777.769620035] [driving_environment_analyzer]: - Exist crosswalk: true
[component_container-1] [INFO 1708999777.769634980] [driving_environment_analyzer]: ======================================
[component_container-1] [INFO 1708999777.769642769] [driving_environment_analyzer]:  complete ODD analysis. shutdown.
[component_container-1] [INFO 1708999777.769650034] [driving_environment_analyzer]: ======================================
```

ただし、`map/vector_map`に関しては`use_map_in_bag`を`false`にすることでローカル環境に保存されている地図を使用してODD解析を行うことも可能です。その場合、`map_path`オプションで地図のパスを指定してください。

`ros2 launch driving_environment_analyzer driving_environment_analyzer.launch.xml use_map_in_bag:=false map_path:=<MAP> bag_path:=<ROSBAG>`

以上のようにオプションを指定することでROSBAGに地図情報が保存されていなくてもODD解析が可能です。

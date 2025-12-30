^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package planning_debug_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2025-12-30)
------------------
* feat(perception_replayer): cpp implementation of perception replayer (`#321 <https://github.com/autowarefoundation/autoware_tools/issues/321>`_)
  * feat cpp version perception_replayer/reproducer
  * delete comment
  * fix pre-commit error
  * fix spell
  * fix package.xml depends
  * update readme and fix options
  * add error ouptut
  * add noise and pub_route option
  * fix option behavior
  * delete unnecessary depend
  * fix typo
  * throttling warn message
  * throttling warn message
  ---------
* Contributors: Kotakku

0.4.0 (2025-11-16)
------------------
* feat(trajectory_visualizer): sort topic list (`#307 <https://github.com/autowarefoundation/autoware_tools/issues/307>`_)
  * sort topic list
  * fix pre-commit error
  * add README topic display ordering section
  * fix inital topics selection
  * config path
  * fix pre-commit error
  * add copylight
  ---------
* feat(planning_debug_tools): improve trajectory visualizer usability (`#300 <https://github.com/autowarefoundation/autoware_tools/issues/300>`_)
  * feat equal aspect ratio when XY-plot
  * add unwrapped yaw plot
  * fix pre-commit
  ---------
* fix: change to detailed library reference (`#295 <https://github.com/autowarefoundation/autoware_tools/issues/295>`_)
* feat(planning_debug_tools): support V2I (`#292 <https://github.com/autowarefoundation/autoware_tools/issues/292>`_)
* fix(planning_debug_tools): fix InitializeLocalization msg import (`#291 <https://github.com/autowarefoundation/autoware_tools/issues/291>`_)
* fix: show motion velocity planner output by default (`#290 <https://github.com/autowarefoundation/autoware_tools/issues/290>`_)
* Contributors: Kotakku, Satoshi OTA, Xiaoyu WANG, Yukinari Hisaki

0.3.0 (2025-08-11)
------------------
* feat(trajectory_visualizer): add initial config and better layout (`#273 <https://github.com/autowarefoundation/autoware_tools/issues/273>`_)
* feat(trajectory_visualizer): support for new message types (`#272 <https://github.com/autowarefoundation/autoware_tools/issues/272>`_)
  * fix(trajectory_data): handle early return for empty or single element trajectory in get_arc_lengths
  * feat(trajectory_visualizer): support multiple message types in trajectory visualization and data processing
  * feat(trajectory_data): add index_values function to return indices of trajectory points
  * apply pre-commit
  * remove unused imports
  * fix(package.xml): add missing maintainer entry for Maxime CLEMENT
  ---------
* style(pre-commit): autofix (`#275 <https://github.com/autowarefoundation/autoware_tools/issues/275>`_)
  * apply pre-commit
  * fix: add maintainer in planning_debug tools package.xml
  ---------
* feat(trajectory_visualizer): add visualization of X, Y, and Yaw values (`#262 <https://github.com/autowarefoundation/autoware_tools/issues/262>`_)
* fix(planning): bug of loading the db3 rosbag for the perception replayer (`#269 <https://github.com/autowarefoundation/autoware_tools/issues/269>`_)
  * fix loading rosbag bug
  * remove formatting chanes
  ---------
* feat(planning/perception_reproducer): add mcap bag format support (`#246 <https://github.com/autowarefoundation/autoware_tools/issues/246>`_)
* fix: skip ureadable msg (`#259 <https://github.com/autowarefoundation/autoware_tools/issues/259>`_)
  fix error check
* feat(planning_debug_tools): add trajectory visualizer tool (`#251 <https://github.com/autowarefoundation/autoware_tools/issues/251>`_)
  * implement initial script
  * improvements
  * add todo
  * improve script and add arc length shift on ego pose
  * remove "plot" button (now we replot when topics selection is changed)
  * Add README
  * disable cspell for python code
  * add copyright
  ---------
* Contributors: Kem (TiankuiXian), Kyoichi Sugahara, Maxime CLEMENT, yukage-oya

0.2.0 (2025-03-24)
------------------
* feat(planning_debug_tools)!: replace VelocityLimit messages to autoware_internal_planning_msgs (`#223 <https://github.com/autowarefoundation/autoware_tools/issues/223>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#206 <https://github.com/autowarefoundation/autoware_tools/issues/206>`_)
  * feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId
* Contributors: Ryohsuke Mitsudome

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* feat(planning_debug_tools)!: replace tier4_debug_msgs with tier4_internal_debug_msgs (`#200 <https://github.com/autowarefoundation/autoware_tools/issues/200>`_)
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(perception_reproducer): fix a small bug (`#155 <https://github.com/autowarefoundation/autoware_tools/issues/155>`_)
  * fix bug.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(perception_reproducer): add publish route option (`#154 <https://github.com/autowarefoundation/autoware_tools/issues/154>`_)
  * feat(perception_reproducer): publish route created from the initial pose and goal pose retrieved from the beginning and end of the rosbag
  * add readme
  ---------
* fix(planning_debug_tools): output engage status properly (`#146 <https://github.com/autowarefoundation/autoware_tools/issues/146>`_)
* fix a perception reproducer bug. (`#129 <https://github.com/autowarefoundation/autoware_tools/issues/129>`_)
* chore: remove the support to auto-msgs. (`#115 <https://github.com/autowarefoundation/autoware_tools/issues/115>`_)
  * remove the support to auto-msgs.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: add percpetion msgs to dependance (`#109 <https://github.com/autowarefoundation/autoware_tools/issues/109>`_)
  * add percpetion msgs to dependance.
  * style(pre-commit): autofix
  * update .repos
  * update .repos
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(path_optimizer): rename package name (`#98 <https://github.com/autowarefoundation/autoware_tools/issues/98>`_)
  * refactor(path_optimizer): rename package name
  * rename in graph
  ---------
* feat(autoware_debug_tools): add system_usage_monitor.py (`#85 <https://github.com/autowarefoundation/autoware_tools/issues/85>`_)
  * feat(autoware_debug_tools): add system_usage_monitor.py
  * publish system usage
  ---------
* feat(perception reproducer): some improvements in perception reproducer. (`#72 <https://github.com/autowarefoundation/autoware_tools/issues/72>`_)
  * fix(perception reproducer): improve perception reproducer (`#69 <https://github.com/autowarefoundation/autoware_tools/issues/69>`_)
  * temp save new parameters.
  * update parameter
  * update commit
  * feat(metrics): make it possible to hide graph for each metrics (`#64 <https://github.com/autowarefoundation/autoware_tools/issues/64>`_)
  * feat(planning_debug_tools): add cpu_usage_checker.py (`#66 <https://github.com/autowarefoundation/autoware_tools/issues/66>`_)
  * feat(planning_debug_tools): add cpu_usage_checker.py
  * update README
  ---------
  * feat(route_client): add route_client (`#67 <https://github.com/autowarefoundation/autoware_tools/issues/67>`_)
  * fix: autoware_msgs (`#68 <https://github.com/autowarefoundation/autoware_tools/issues/68>`_)
  * feat(control_data_collecting_tool): add control_data_collecting_tool (`#33 <https://github.com/autowarefoundation/autoware_tools/issues/33>`_)
  * Add control data collecting tool for learning-based control
  * Implement basic part
  * Disable data collection area selection during automatic driving
  * Remove unnecessary code
  * Add rviz_config for data collection
  * Update rviz_config for data collection
  * Changed to start driving by pressing LOCAL
  * Fix hard coding rviz config path when launching
  * Add a launch file in this package
  * Add missing changes
  * Claen up arguments to use rviz_config
  * Updated pre-commit-hooks-ros version from v0.8.0 to v0.9.0 (`#24 <https://github.com/autowarefoundation/autoware_tools/issues/24>`_)
  * Add 8-figure target route
  * Avoid errors when data collecting area cannot be selected by clicking properly.
  * Change to launch another launch for data collection after launching the standard autoware_launch.
  * chore(tools): move system and evaluation tools to this repo from autoware.universe (`#26 <https://github.com/autowarefoundation/autoware_tools/issues/26>`_)
  * chore(tools): move system tools
  * chore(evaluator): move evaluators
  ---------
  * update
  * update trajectory publisher
  * update
  * feat(driving_environment_analyzer): add rviz plugin (`#23 <https://github.com/autowarefoundation/autoware_tools/issues/23>`_)
  * feat(driving_environment_analyzer): add rviz plugin
  * feat(driving_environment_analyzer): output csv file
  ---------
  * add naive pure pursuit
  * update
  * clean up naive pure pursuit
  * smoothing trajectory position
  * refactor
  * chore(rviz_plugin): move peripheral rviz plugin (`#27 <https://github.com/autowarefoundation/autoware_tools/issues/27>`_)
  * feat:  calibrator tier4 pkg name (`#200 <https://github.com/autowarefoundation/autoware_tools/issues/200>`_)
  * feat: change names
  * feat: move pkg to common
  * feat: change pkg name
  * fix: library path (`#225 <https://github.com/autowarefoundation/autoware_tools/issues/225>`_)
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  * feat: add rviz plugin to publish and control the simulated clock (`#349 <https://github.com/autowarefoundation/autoware_tools/issues/349>`_)
  * Add tier4_clock_rviz_plugin to publish&control the sim clock in rviz
  * Add step control
  * Fix precommit
  * Update documentation
  * Fix spellcheck
  * Update plugin description and icon
  * Rename package
  * Fix bug with long duration jumps (high speed + low rate)
  * ci: check include guard (`#438 <https://github.com/autowarefoundation/autoware_tools/issues/438>`_)
  * ci: check include guard
  * apply pre-commit
  * Update .pre-commit-config.yaml
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * fix: pre-commit
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * chore: sync files (`#629 <https://github.com/autowarefoundation/autoware_tools/issues/629>`_)
  * chore: sync files
  * ci(pre-commit): autofix
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * style: fix format of package.xml (`#844 <https://github.com/autowarefoundation/autoware_tools/issues/844>`_)
  * refactor: use autoware cmake (`#849 <https://github.com/autowarefoundation/autoware_tools/issues/849>`_)
  * remove autoware_auto_cmake
  * add build_depend of autoware_cmake
  * use autoware_cmake in CMakeLists.txt
  * fix bugs
  * fix cmake lint errors
  * chore: upgrade cmake_minimum_required to 3.14 (`#856 <https://github.com/autowarefoundation/autoware_tools/issues/856>`_)
  * fix(accel_brake_map_calibrator): rviz panel type (`#895 <https://github.com/autowarefoundation/autoware_tools/issues/895>`_)
  * fixed panel type
  * modified instruction for rosbag replay case
  * modified update_map_dir service name
  * feat: add manual controller (`#989 <https://github.com/autowarefoundation/autoware_tools/issues/989>`_)
  * feat: add manual controller
  * ci(pre-commit): autofix
  * fix : typo
  * chore: minor update
  * chore : add minor update
  * docs: add image for readme
  * docs: update docs
  * ci(pre-commit): autofix
  * ci(pre-commit): autofix
  * Update common/tier4_control_rviz_plugin/src/tools/manual_controller.cpp
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  * Update common/tier4_control_rviz_plugin/src/tools/manual_controller.hpp
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  * feat(manual_controller): support backward driving (`#1119 <https://github.com/autowarefoundation/autoware_tools/issues/1119>`_)
  * feat(manual_controller): support backward driving
  * chore: unite variable
  * feat: add screen capture package (`#1177 <https://github.com/autowarefoundation/autoware_tools/issues/1177>`_)
  * feat: add screen capture package
  * feat: add state to current process
  * style: refactor codes
  * fix: time
  * feat: add mp4 writer and finalize folders
  * feat: add trigger by service
  * feat: update recorder
  * fix: change codec of mp4 to h264 (`#1262 <https://github.com/autowarefoundation/autoware_tools/issues/1262>`_)
  * feat: optimize screen capture panel (`#1470 <https://github.com/autowarefoundation/autoware_tools/issues/1470>`_)
  * feat: optimize screen capture panel
  * apply pre-commit
  * fix: remove unused check of rviz plugin version (`#1474 <https://github.com/autowarefoundation/autoware_tools/issues/1474>`_)
  * fix(rviz_plugin): fix screen capture plugin (`#1492 <https://github.com/autowarefoundation/autoware_tools/issues/1492>`_)
  * refactor(tier4_calibration_rviz_plugin): apply clang-tidy (`#1596 <https://github.com/autowarefoundation/autoware_tools/issues/1596>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy (`#1597 <https://github.com/autowarefoundation/autoware_tools/issues/1597>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy
  * revert: readability-identifier-naming
  * revert(tier4_calibration_rviz_plugin): readability-identifier-naming (`#1618 <https://github.com/autowarefoundation/autoware_tools/issues/1618>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix release process to handle video writer correctly (`#1622 <https://github.com/autowarefoundation/autoware_tools/issues/1622>`_)
  * refactor(tier4_screen_capture_rviz_plugin):apply clang-tidy (`#1649 <https://github.com/autowarefoundation/autoware_tools/issues/1649>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check (`#1790 <https://github.com/autowarefoundation/autoware_tools/issues/1790>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: add rtc  manager rviz plugin (`#1936 <https://github.com/autowarefoundation/autoware_tools/issues/1936>`_)
  * feat: add rtc  manager rviz plugin
  * chore: cosmetic change
  * chore: remove indent
  * feat: add rtc safe unsafe color
  * fix: typo
  * chore: simplify layout
  * feat: update rtc panel
  * feat(rtc_manager_panel): add rtc all approval (`#2009 <https://github.com/autowarefoundation/autoware_tools/issues/2009>`_)
  * feat(rtc_manager_panel): add rtc all approval
  * chore: cosmetic change
  * fix: static cast
  * chore: update text
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * chore: update text
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * doc: update documents
  * doc: update
  * doc: small update
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * feat(rtc_manager_rviz_plugin): add_indivisual_exe (`#2021 <https://github.com/autowarefoundation/autoware_tools/issues/2021>`_)
  * feat(tier4_screen_capture_rviz_plugin): add prefix to video name (`#2038 <https://github.com/autowarefoundation/autoware_tools/issues/2038>`_)
  feat: add  prefix
  * feat(tier4_debug_rviz_plugin): pie chart with float32 multi array stamped (`#2055 <https://github.com/autowarefoundation/autoware_tools/issues/2055>`_)
  * feat(tier4_debug_rviz_plugin): add ros2 pie chart with Float32MultiArrayStamped
  * update README.md
  * fix typo
  * fixed license
  * fix
  * removed unnecessary include
  * fix
  * fix
  * fix(tier4_control_rviz_plugin): add time stamp for control command (`#2154 <https://github.com/autowarefoundation/autoware_tools/issues/2154>`_)
  * fix(rtc_manager_rviz_plugin): size check (`#2163 <https://github.com/autowarefoundation/autoware_tools/issues/2163>`_)
  * feat(behavior_path_planner): external request lane change (`#2442 <https://github.com/autowarefoundation/autoware_tools/issues/2442>`_)
  * feature(behavior_path_planner): add external request lane change module
  feature(behavior_path_planner): fix for RTC
  feature(behavior_path_planner): fix decision logic
  feat(behavior_path_planner): fix behavior_path_planner_tree.xml
  feat(behavior_path_planner): fix for rebase
  feat(behavior_path_planner): output multiple candidate paths
  feat(behavior_path_planner): get path candidate in behavior tree manager
  feat(behavior_path_planner): fix for multiple candidate path
  feat(behavior_path_planner): separate external request lane change module
  feature(behavior_path_planner): add create publisher method
  feature(behavior_path_planner): move publishers to node
  feature(behavior_path_planner): remove unnecessary publisher
  feat(behavior_path_planner): move reset path candidate function to behavior tree manager
  feat(behavior_path_planner): add external request lane change path candidate publisher
  feat(behavior_path_planner): apply abort lane change
  * fix(behavior_path_planner): remove unnecessary change
  * feat(behavior_path_planner): fix getLaneChangePaths()
  * feat(behavior_path_planner): disable external request lane change in default tree
  * Update rtc_auto_mode_manager.param.yaml
  * fix(route_handler): remove redundant code
  * fix(behavior_path_planner): fix for turn signal
  * chore(rtc_manager_rviz_plugin): add code owner (`#2792 <https://github.com/autowarefoundation/autoware_tools/issues/2792>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status (`#2791 <https://github.com/autowarefoundation/autoware_tools/issues/2791>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status
  * chore: simplify layout
  ---------
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  * feat(automatic_goal): add automatic goal rviz plugin (`#3031 <https://github.com/autowarefoundation/autoware_tools/issues/3031>`_)
  * add first version automatic_goal plugin
  * feat(automatic_goal): extract automatic_goal_sender, add logging achieved goals
  * doc(automatic_goal): append README
  * ref(automatic_goal): apply pre-commity, fix depend
  * fix(automatic_goal): fix warnings - treated as errors
  * ref(automatic_goal): add author, apply clang-tidy hints
  * ref(automatic_goal): add maintainer, change  year
  * ref(automatic_goal): fix package.xml order
  * ref(automatic_goal): names, initializations, main except
  * fix(automatic_goal): change path home->tmp
  * fix(automatic_goal): fix bad string init, expand readme
  * fix(automatic_goal): fix name
  ---------
  * feat(rtc_manager_rviz_plugin): add avoidance by lc (`#3118 <https://github.com/autowarefoundation/autoware_tools/issues/3118>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix extra/missing naming components (`#3207 <https://github.com/autowarefoundation/autoware_tools/issues/3207>`_)
  * chore: sync files (`#3227 <https://github.com/autowarefoundation/autoware_tools/issues/3227>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: change external lane change rtc name (`#3259 <https://github.com/autowarefoundation/autoware_tools/issues/3259>`_)
  * feat: change external lane change rtc name
  * update config
  ---------
  * feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#3458 <https://github.com/autowarefoundation/autoware_tools/issues/3458>`_)
  * migrated
  * WIP
  * IntersectionModuleManager own one rtc_interface for intersection_occlusion
  * divide occlusion safety and activated
  * fixed to update occlusion cooperate status at construction
  * fixed getOcclusionFirstStopSafety()
  * fixed not to show both intersection and intersection_occlusion  after passed 1st stop line
  * fixed the intersection_occlusion/inersection stop position afte r CLEARED
  * if occlusion cleared when eog is before 1st stop line, set stop line to 1st stop line and clear prev_occlusion_stop_line_pose\_
  * (misc) fix viz, sync param
  * fixed vehicle footprint offset calculation
  * add occcupancy_grid_map method/param var to launcher
  * migrated latest
  * use static pass judge line
  * removed some params
  * organized param
  * add occlusion enable flag
  * revert occupancy grid settings in this PR
  * remove comment
  * fixed pass judge line collision detection to original
  * style(pre-commit): autofix
  * use vehicle_length for static pass judge line
  * fixed virtual wall marker
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * fix(rtc_manager_rviz_plugin): update panel visualization properly (`#3517 <https://github.com/autowarefoundation/autoware_tools/issues/3517>`_)
  * refactor(behavior_path_planner): rename pull_over to goal_planner (`#3501 <https://github.com/autowarefoundation/autoware_tools/issues/3501>`_)
  * build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_tools/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  * build(iron): remove rmw_qos_profile_t (`#3809 <https://github.com/autowarefoundation/autoware_tools/issues/3809>`_)
  * refactor(start_planner): rename pull out to start planner (`#3908 <https://github.com/autowarefoundation/autoware_tools/issues/3908>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name (`#4539 <https://github.com/autowarefoundation/autoware_tools/issues/4539>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name
  * misc
  ---------
  * feat(rviz_plugin): add target object type display (`#4855 <https://github.com/autowarefoundation/autoware_tools/issues/4855>`_)
  * add common/tier4_target_object_type_rviz_plugin
  * fix format
  * update color
  * update
  * add readme
  * Update common/tier4_target_object_type_rviz_plugin/src/target_object_type_panel.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update common/tier4_target_object_type_rviz_plugin/src/target_object_type_panel.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * remove unused depend
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix(rtc_manager_panel): fix panel chattering (`#4988 <https://github.com/autowarefoundation/autoware_tools/issues/4988>`_)
  * build(tier4_target_object_type_rviz_plugin): add missing cv_bridge dependency (`#5000 <https://github.com/autowarefoundation/autoware_tools/issues/5000>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level (`#5112 <https://github.com/autowarefoundation/autoware_tools/issues/5112>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level
  * change file names
  * move initialization code from constructor to onInitialize
  * add maintainer
  * add maintainer
  * fix include
  ---------
  * feat(logger_level_reconfigure_plugin): use node interface and some cosmetic updates (`#5204 <https://github.com/autowarefoundation/autoware_tools/issues/5204>`_)
  * use node service
  * enable yaml configuration
  * update yaml loading
  * make it scrollable
  * change function order
  * change color for level
  * fix depend
  * Update common/tier4_logging_level_configure_rviz_plugin/src/logging_level_configure.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update common/tier4_logging_level_configure_rviz_plugin/src/logging_level_configure.cpp
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * refactor(lane_change): add debug log (`#5308 <https://github.com/autowarefoundation/autoware_tools/issues/5308>`_)
  * docs(logger_level_reconfigure): update readme (`#5471 <https://github.com/autowarefoundation/autoware_tools/issues/5471>`_)
  * feat(localization): enable logging_level_configure (`#5487 <https://github.com/autowarefoundation/autoware_tools/issues/5487>`_)
  * feat(localization): enable logging_level_configure
  * style(pre-commit): autofix
  * update logger config
  * fix pre-commit
  * add tier4_autoware_utils in dependency
  * add tier4_autoware_utils in dependency
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * Logger level update (`#5494 <https://github.com/autowarefoundation/autoware_tools/issues/5494>`_)
  * address ordering
  * add grouping
  * remove unused comment
  ---------
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button (`#5666 <https://github.com/autowarefoundation/autoware_tools/issues/5666>`_)
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button
  * add for control
  ---------
  * refactor(lane_change): standardizing lane change logger name (`#5899 <https://github.com/autowarefoundation/autoware_tools/issues/5899>`_)
  * feat(tier4_logging_level_configure_rviz_plugin): add goal/start planner (`#5978 <https://github.com/autowarefoundation/autoware_tools/issues/5978>`_)
  * chore: set log level of debug printing in rviz plugin to DEBUG (`#5996 <https://github.com/autowarefoundation/autoware_tools/issues/5996>`_)
  * feat(rviz_plugin): add string visualization plugin (`#6100 <https://github.com/autowarefoundation/autoware_tools/issues/6100>`_)
  * feat(tier4_automatic_goal_rviz_plugin): make it possible to register checkpoints (`#6153 <https://github.com/autowarefoundation/autoware_tools/issues/6153>`_)
  * chore(object_type_panel): update rosparam name config (`#6347 <https://github.com/autowarefoundation/autoware_tools/issues/6347>`_)
  * style(update): autoware tools icons (`#6351 <https://github.com/autowarefoundation/autoware_tools/issues/6351>`_)
  * fix(readme): add acknowledgement for material icons in tool plugins (`#6354 <https://github.com/autowarefoundation/autoware_tools/issues/6354>`_)
  * feat(mission_planner_rviz_plugin): create mission planner tool (`#6362 <https://github.com/autowarefoundation/autoware_tools/issues/6362>`_)
  * feat(mission_planner_rviz_plugin): create package
  * fix copyright
  * add interrupted state
  * use full license text instead of spdx
  ---------
  * fix(manual_controller): set PARK gear when going from reverse to drive (`#6230 <https://github.com/autowarefoundation/autoware_tools/issues/6230>`_)
  * feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/autowarefoundation/autoware_tools/issues/6823>`_)
  * feat(launch): add logging_demo::LoggerConfig into container
  * fix(logger_level_reconfigure_plugin): fix yaml
  * feat(logging_level_configure): add composable node
  ---------
  * revert: "feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/autowarefoundation/autoware_tools/issues/6823>`_)" (`#6842 <https://github.com/autowarefoundation/autoware_tools/issues/6842>`_)
  This reverts commit 9d045335d8e3763984bce8dea92f63de3423ebde.
  * docs(tier4_logging_level_configure_rviz_plugin): update document (`#6720 <https://github.com/autowarefoundation/autoware_tools/issues/6720>`_)
  * docs(tier4_logging_level_configure_rviz_plugin): update document
  * fix spell check
  * fix Warning
  ---------
  * refactor(bpp, avoidance): remove unnecessary verbose flag (`#6822 <https://github.com/autowarefoundation/autoware_tools/issues/6822>`_)
  * refactor(avoidance): logger small change
  * refactor(bpp): remove verbose flag
  ---------
  * feat(tier4_screen_capture_panel): add new service to capture screen shot (`#6867 <https://github.com/autowarefoundation/autoware_tools/issues/6867>`_)
  * feat(tier4_screen_capture_panel): add new service to capture screen shot
  * docs(tier4_screen_capture_rviz_plugin): update readme
  ---------
  * refactor(lane_change): fix logger (`#6873 <https://github.com/autowarefoundation/autoware_tools/issues/6873>`_)
  * fix(route_handler): add logger (`#6888 <https://github.com/autowarefoundation/autoware_tools/issues/6888>`_)
  * fix(route_handler): add logger
  * fix indent
  ---------
  * docs(tier4_simulated_clock_rviz_plugin): update how to use (`#6914 <https://github.com/autowarefoundation/autoware_tools/issues/6914>`_)
  * docs(tier4_simulated_clock_rviz_plugin): update how to use
  * fixed tabbed warning
  * fix warning not working
  * Fix bullet list
  ---------
  * refactor(bpp): path shifter clang tidy and logging level configuration (`#6917 <https://github.com/autowarefoundation/autoware_tools/issues/6917>`_)
  * fix(accel_brake_calibrator): fix to set service name and exception failure (`#6973 <https://github.com/autowarefoundation/autoware_tools/issues/6973>`_)
  * add service
  * fix exception
  * fix style
  * refactor(motion_utils): supress log message with rclcpp logging (`#6955 <https://github.com/autowarefoundation/autoware_tools/issues/6955>`_)
  * refactor(motion_utils): supress log message with rclcpp logging
  * remove std string
  ---------
  ---------
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  * update smoothing of start/end-point
  * Add simplified lateral acc limit
  * update debug plot
  * add gif
  * Update README.md
  * Update README.md
  * use ros param
  * ros param
  * update
  * update image
  * Update README.md
  * Add velocity noise
  * Update acc and steer noises
  * update rviz
  * update resource
  * Update README.md
  * Update README.md
  * Update control_data_collecting_tool/README.md
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * update readme
  * Update bag2lanelet/scripts/bag2map.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * modify sentence
  * chore(rtc_manager_panel): update module name (`#29 <https://github.com/autowarefoundation/autoware_tools/issues/29>`_)
  * chore(rtc_manager_panel): update module name
  * chore(gitignore): remove py cache
  ---------
  * improve pure pursuit
  * Update control_data_collecting_tool/scripts/data_collecting_trajectory_publisher.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * remove deprecated linearized pure pursuit
  * clean up traj pub
  * Reduce speed for large lateral error
  * Improve how to obtain the trajectory closest point
  * update readme
  * rename variables
  * compute trajectory only when receive data collecting area info
  * smoothing target yaw
  * add yaw error threshold
  * remove rviz config
  * renew png and gif
  * add arrow on png
  * feat(driving_environment_analyzer): remove dependency to autoware_auto_tf2 (`#31 <https://github.com/autowarefoundation/autoware_tools/issues/31>`_)
  ---------
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  * fix markdownlint in README
  * add maitainers
  * update msgs
  * add linearized pure pursuit control
  * change default params
  * add stop automatic driving
  * update control cmd limit
  * [bug fix] recompute trajectory when changing rosparam online
  * fix dead link
  * run pre-commit
  * fix spell
  * Update bag2lanelet/scripts/bag2map.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: masayukiaino <101699734+masayukiaino@users.noreply.github.com>
  * refresh cool down for setting initial pose in psim.
  * style(pre-commit): autofix
  * parameterize publishing_speed_factor
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Toru Hishinuma <92719065+th1991-01@users.noreply.github.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: masayukiaino <101699734+masayukiaino@users.noreply.github.com>
  * fix a bug in converting old traffic liht message. (`#70 <https://github.com/autowarefoundation/autoware_tools/issues/70>`_)
  * refactored perception_replayer/reproducer.
  * refactor perception_reproducer in order to speed up publishing speed. delete some unnessesary parameters.
  * add noise to perception objects when repeat publishing.
  * update speed constraint
  * fix spell err
  * style(pre-commit): autofix
  * fix spell err
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Toru Hishinuma <92719065+th1991-01@users.noreply.github.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: masayukiaino <101699734+masayukiaino@users.noreply.github.com>
* fix a bug in converting old traffic liht message. (`#70 <https://github.com/autowarefoundation/autoware_tools/issues/70>`_)
* fix(perception reproducer): improve perception reproducer (`#69 <https://github.com/autowarefoundation/autoware_tools/issues/69>`_)
  * temp save new parameters.
  * update parameter
  * update commit
  * feat(metrics): make it possible to hide graph for each metrics (`#64 <https://github.com/autowarefoundation/autoware_tools/issues/64>`_)
  * feat(planning_debug_tools): add cpu_usage_checker.py (`#66 <https://github.com/autowarefoundation/autoware_tools/issues/66>`_)
  * feat(planning_debug_tools): add cpu_usage_checker.py
  * update README
  ---------
  * feat(route_client): add route_client (`#67 <https://github.com/autowarefoundation/autoware_tools/issues/67>`_)
  * fix: autoware_msgs (`#68 <https://github.com/autowarefoundation/autoware_tools/issues/68>`_)
  * feat(control_data_collecting_tool): add control_data_collecting_tool (`#33 <https://github.com/autowarefoundation/autoware_tools/issues/33>`_)
  * Add control data collecting tool for learning-based control
  * Implement basic part
  * Disable data collection area selection during automatic driving
  * Remove unnecessary code
  * Add rviz_config for data collection
  * Update rviz_config for data collection
  * Changed to start driving by pressing LOCAL
  * Fix hard coding rviz config path when launching
  * Add a launch file in this package
  * Add missing changes
  * Claen up arguments to use rviz_config
  * Updated pre-commit-hooks-ros version from v0.8.0 to v0.9.0 (`#24 <https://github.com/autowarefoundation/autoware_tools/issues/24>`_)
  * Add 8-figure target route
  * Avoid errors when data collecting area cannot be selected by clicking properly.
  * Change to launch another launch for data collection after launching the standard autoware_launch.
  * chore(tools): move system and evaluation tools to this repo from autoware.universe (`#26 <https://github.com/autowarefoundation/autoware_tools/issues/26>`_)
  * chore(tools): move system tools
  * chore(evaluator): move evaluators
  ---------
  * update
  * update trajectory publisher
  * update
  * feat(driving_environment_analyzer): add rviz plugin (`#23 <https://github.com/autowarefoundation/autoware_tools/issues/23>`_)
  * feat(driving_environment_analyzer): add rviz plugin
  * feat(driving_environment_analyzer): output csv file
  ---------
  * add naive pure pursuit
  * update
  * clean up naive pure pursuit
  * smoothing trajectory position
  * refactor
  * chore(rviz_plugin): move peripheral rviz plugin (`#27 <https://github.com/autowarefoundation/autoware_tools/issues/27>`_)
  * feat:  calibrator tier4 pkg name (`#200 <https://github.com/autowarefoundation/autoware_tools/issues/200>`_)
  * feat: change names
  * feat: move pkg to common
  * feat: change pkg name
  * fix: library path (`#225 <https://github.com/autowarefoundation/autoware_tools/issues/225>`_)
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  * feat: add rviz plugin to publish and control the simulated clock (`#349 <https://github.com/autowarefoundation/autoware_tools/issues/349>`_)
  * Add tier4_clock_rviz_plugin to publish&control the sim clock in rviz
  * Add step control
  * Fix precommit
  * Update documentation
  * Fix spellcheck
  * Update plugin description and icon
  * Rename package
  * Fix bug with long duration jumps (high speed + low rate)
  * ci: check include guard (`#438 <https://github.com/autowarefoundation/autoware_tools/issues/438>`_)
  * ci: check include guard
  * apply pre-commit
  * Update .pre-commit-config.yaml
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * fix: pre-commit
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * chore: sync files (`#629 <https://github.com/autowarefoundation/autoware_tools/issues/629>`_)
  * chore: sync files
  * ci(pre-commit): autofix
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * style: fix format of package.xml (`#844 <https://github.com/autowarefoundation/autoware_tools/issues/844>`_)
  * refactor: use autoware cmake (`#849 <https://github.com/autowarefoundation/autoware_tools/issues/849>`_)
  * remove autoware_auto_cmake
  * add build_depend of autoware_cmake
  * use autoware_cmake in CMakeLists.txt
  * fix bugs
  * fix cmake lint errors
  * chore: upgrade cmake_minimum_required to 3.14 (`#856 <https://github.com/autowarefoundation/autoware_tools/issues/856>`_)
  * fix(accel_brake_map_calibrator): rviz panel type (`#895 <https://github.com/autowarefoundation/autoware_tools/issues/895>`_)
  * fixed panel type
  * modified instruction for rosbag replay case
  * modified update_map_dir service name
  * feat: add manual controller (`#989 <https://github.com/autowarefoundation/autoware_tools/issues/989>`_)
  * feat: add manual controller
  * ci(pre-commit): autofix
  * fix : typo
  * chore: minor update
  * chore : add minor update
  * docs: add image for readme
  * docs: update docs
  * ci(pre-commit): autofix
  * ci(pre-commit): autofix
  * Update common/tier4_control_rviz_plugin/src/tools/manual_controller.cpp
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  * Update common/tier4_control_rviz_plugin/src/tools/manual_controller.hpp
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  * feat(manual_controller): support backward driving (`#1119 <https://github.com/autowarefoundation/autoware_tools/issues/1119>`_)
  * feat(manual_controller): support backward driving
  * chore: unite variable
  * feat: add screen capture package (`#1177 <https://github.com/autowarefoundation/autoware_tools/issues/1177>`_)
  * feat: add screen capture package
  * feat: add state to current process
  * style: refactor codes
  * fix: time
  * feat: add mp4 writer and finalize folders
  * feat: add trigger by service
  * feat: update recorder
  * fix: change codec of mp4 to h264 (`#1262 <https://github.com/autowarefoundation/autoware_tools/issues/1262>`_)
  * feat: optimize screen capture panel (`#1470 <https://github.com/autowarefoundation/autoware_tools/issues/1470>`_)
  * feat: optimize screen capture panel
  * apply pre-commit
  * fix: remove unused check of rviz plugin version (`#1474 <https://github.com/autowarefoundation/autoware_tools/issues/1474>`_)
  * fix(rviz_plugin): fix screen capture plugin (`#1492 <https://github.com/autowarefoundation/autoware_tools/issues/1492>`_)
  * refactor(tier4_calibration_rviz_plugin): apply clang-tidy (`#1596 <https://github.com/autowarefoundation/autoware_tools/issues/1596>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy (`#1597 <https://github.com/autowarefoundation/autoware_tools/issues/1597>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy
  * revert: readability-identifier-naming
  * revert(tier4_calibration_rviz_plugin): readability-identifier-naming (`#1618 <https://github.com/autowarefoundation/autoware_tools/issues/1618>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix release process to handle video writer correctly (`#1622 <https://github.com/autowarefoundation/autoware_tools/issues/1622>`_)
  * refactor(tier4_screen_capture_rviz_plugin):apply clang-tidy (`#1649 <https://github.com/autowarefoundation/autoware_tools/issues/1649>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check (`#1790 <https://github.com/autowarefoundation/autoware_tools/issues/1790>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: add rtc  manager rviz plugin (`#1936 <https://github.com/autowarefoundation/autoware_tools/issues/1936>`_)
  * feat: add rtc  manager rviz plugin
  * chore: cosmetic change
  * chore: remove indent
  * feat: add rtc safe unsafe color
  * fix: typo
  * chore: simplify layout
  * feat: update rtc panel
  * feat(rtc_manager_panel): add rtc all approval (`#2009 <https://github.com/autowarefoundation/autoware_tools/issues/2009>`_)
  * feat(rtc_manager_panel): add rtc all approval
  * chore: cosmetic change
  * fix: static cast
  * chore: update text
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * chore: update text
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * doc: update documents
  * doc: update
  * doc: small update
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  * feat(rtc_manager_rviz_plugin): add_indivisual_exe (`#2021 <https://github.com/autowarefoundation/autoware_tools/issues/2021>`_)
  * feat(tier4_screen_capture_rviz_plugin): add prefix to video name (`#2038 <https://github.com/autowarefoundation/autoware_tools/issues/2038>`_)
  feat: add  prefix
  * feat(tier4_debug_rviz_plugin): pie chart with float32 multi array stamped (`#2055 <https://github.com/autowarefoundation/autoware_tools/issues/2055>`_)
  * feat(tier4_debug_rviz_plugin): add ros2 pie chart with Float32MultiArrayStamped
  * update README.md
  * fix typo
  * fixed license
  * fix
  * removed unnecessary include
  * fix
  * fix
  * fix(tier4_control_rviz_plugin): add time stamp for control command (`#2154 <https://github.com/autowarefoundation/autoware_tools/issues/2154>`_)
  * fix(rtc_manager_rviz_plugin): size check (`#2163 <https://github.com/autowarefoundation/autoware_tools/issues/2163>`_)
  * feat(behavior_path_planner): external request lane change (`#2442 <https://github.com/autowarefoundation/autoware_tools/issues/2442>`_)
  * feature(behavior_path_planner): add external request lane change module
  feature(behavior_path_planner): fix for RTC
  feature(behavior_path_planner): fix decision logic
  feat(behavior_path_planner): fix behavior_path_planner_tree.xml
  feat(behavior_path_planner): fix for rebase
  feat(behavior_path_planner): output multiple candidate paths
  feat(behavior_path_planner): get path candidate in behavior tree manager
  feat(behavior_path_planner): fix for multiple candidate path
  feat(behavior_path_planner): separate external request lane change module
  feature(behavior_path_planner): add create publisher method
  feature(behavior_path_planner): move publishers to node
  feature(behavior_path_planner): remove unnecessary publisher
  feat(behavior_path_planner): move reset path candidate function to behavior tree manager
  feat(behavior_path_planner): add external request lane change path candidate publisher
  feat(behavior_path_planner): apply abort lane change
  * fix(behavior_path_planner): remove unnecessary change
  * feat(behavior_path_planner): fix getLaneChangePaths()
  * feat(behavior_path_planner): disable external request lane change in default tree
  * Update rtc_auto_mode_manager.param.yaml
  * fix(route_handler): remove redundant code
  * fix(behavior_path_planner): fix for turn signal
  * chore(rtc_manager_rviz_plugin): add code owner (`#2792 <https://github.com/autowarefoundation/autoware_tools/issues/2792>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status (`#2791 <https://github.com/autowarefoundation/autoware_tools/issues/2791>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status
  * chore: simplify layout
  ---------
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  * feat(automatic_goal): add automatic goal rviz plugin (`#3031 <https://github.com/autowarefoundation/autoware_tools/issues/3031>`_)
  * add first version automatic_goal plugin
  * feat(automatic_goal): extract automatic_goal_sender, add logging achieved goals
  * doc(automatic_goal): append README
  * ref(automatic_goal): apply pre-commity, fix depend
  * fix(automatic_goal): fix warnings - treated as errors
  * ref(automatic_goal): add author, apply clang-tidy hints
  * ref(automatic_goal): add maintainer, change  year
  * ref(automatic_goal): fix package.xml order
  * ref(automatic_goal): names, initializations, main except
  * fix(automatic_goal): change path home->tmp
  * fix(automatic_goal): fix bad string init, expand readme
  * fix(automatic_goal): fix name
  ---------
  * feat(rtc_manager_rviz_plugin): add avoidance by lc (`#3118 <https://github.com/autowarefoundation/autoware_tools/issues/3118>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix extra/missing naming components (`#3207 <https://github.com/autowarefoundation/autoware_tools/issues/3207>`_)
  * chore: sync files (`#3227 <https://github.com/autowarefoundation/autoware_tools/issues/3227>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: change external lane change rtc name (`#3259 <https://github.com/autowarefoundation/autoware_tools/issues/3259>`_)
  * feat: change external lane change rtc name
  * update config
  ---------
  * feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#3458 <https://github.com/autowarefoundation/autoware_tools/issues/3458>`_)
  * migrated
  * WIP
  * IntersectionModuleManager own one rtc_interface for intersection_occlusion
  * divide occlusion safety and activated
  * fixed to update occlusion cooperate status at construction
  * fixed getOcclusionFirstStopSafety()
  * fixed not to show both intersection and intersection_occlusion  after passed 1st stop line
  * fixed the intersection_occlusion/inersection stop position afte r CLEARED
  * if occlusion cleared when eog is before 1st stop line, set stop line to 1st stop line and clear prev_occlusion_stop_line_pose\_
  * (misc) fix viz, sync param
  * fixed vehicle footprint offset calculation
  * add occcupancy_grid_map method/param var to launcher
  * migrated latest
  * use static pass judge line
  * removed some params
  * organized param
  * add occlusion enable flag
  * revert occupancy grid settings in this PR
  * remove comment
  * fixed pass judge line collision detection to original
  * style(pre-commit): autofix
  * use vehicle_length for static pass judge line
  * fixed virtual wall marker
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * fix(rtc_manager_rviz_plugin): update panel visualization properly (`#3517 <https://github.com/autowarefoundation/autoware_tools/issues/3517>`_)
  * refactor(behavior_path_planner): rename pull_over to goal_planner (`#3501 <https://github.com/autowarefoundation/autoware_tools/issues/3501>`_)
  * build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_tools/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  * build(iron): remove rmw_qos_profile_t (`#3809 <https://github.com/autowarefoundation/autoware_tools/issues/3809>`_)
  * refactor(start_planner): rename pull out to start planner (`#3908 <https://github.com/autowarefoundation/autoware_tools/issues/3908>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name (`#4539 <https://github.com/autowarefoundation/autoware_tools/issues/4539>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name
  * misc
  ---------
  * feat(rviz_plugin): add target object type display (`#4855 <https://github.com/autowarefoundation/autoware_tools/issues/4855>`_)
  * add common/tier4_target_object_type_rviz_plugin
  * fix format
  * update color
  * update
  * add readme
  * Update common/tier4_target_object_type_rviz_plugin/src/target_object_type_panel.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update common/tier4_target_object_type_rviz_plugin/src/target_object_type_panel.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * remove unused depend
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix(rtc_manager_panel): fix panel chattering (`#4988 <https://github.com/autowarefoundation/autoware_tools/issues/4988>`_)
  * build(tier4_target_object_type_rviz_plugin): add missing cv_bridge dependency (`#5000 <https://github.com/autowarefoundation/autoware_tools/issues/5000>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level (`#5112 <https://github.com/autowarefoundation/autoware_tools/issues/5112>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level
  * change file names
  * move initialization code from constructor to onInitialize
  * add maintainer
  * add maintainer
  * fix include
  ---------
  * feat(logger_level_reconfigure_plugin): use node interface and some cosmetic updates (`#5204 <https://github.com/autowarefoundation/autoware_tools/issues/5204>`_)
  * use node service
  * enable yaml configuration
  * update yaml loading
  * make it scrollable
  * change function order
  * change color for level
  * fix depend
  * Update common/tier4_logging_level_configure_rviz_plugin/src/logging_level_configure.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update common/tier4_logging_level_configure_rviz_plugin/src/logging_level_configure.cpp
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * refactor(lane_change): add debug log (`#5308 <https://github.com/autowarefoundation/autoware_tools/issues/5308>`_)
  * docs(logger_level_reconfigure): update readme (`#5471 <https://github.com/autowarefoundation/autoware_tools/issues/5471>`_)
  * feat(localization): enable logging_level_configure (`#5487 <https://github.com/autowarefoundation/autoware_tools/issues/5487>`_)
  * feat(localization): enable logging_level_configure
  * style(pre-commit): autofix
  * update logger config
  * fix pre-commit
  * add tier4_autoware_utils in dependency
  * add tier4_autoware_utils in dependency
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * Logger level update (`#5494 <https://github.com/autowarefoundation/autoware_tools/issues/5494>`_)
  * address ordering
  * add grouping
  * remove unused comment
  ---------
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button (`#5666 <https://github.com/autowarefoundation/autoware_tools/issues/5666>`_)
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button
  * add for control
  ---------
  * refactor(lane_change): standardizing lane change logger name (`#5899 <https://github.com/autowarefoundation/autoware_tools/issues/5899>`_)
  * feat(tier4_logging_level_configure_rviz_plugin): add goal/start planner (`#5978 <https://github.com/autowarefoundation/autoware_tools/issues/5978>`_)
  * chore: set log level of debug printing in rviz plugin to DEBUG (`#5996 <https://github.com/autowarefoundation/autoware_tools/issues/5996>`_)
  * feat(rviz_plugin): add string visualization plugin (`#6100 <https://github.com/autowarefoundation/autoware_tools/issues/6100>`_)
  * feat(tier4_automatic_goal_rviz_plugin): make it possible to register checkpoints (`#6153 <https://github.com/autowarefoundation/autoware_tools/issues/6153>`_)
  * chore(object_type_panel): update rosparam name config (`#6347 <https://github.com/autowarefoundation/autoware_tools/issues/6347>`_)
  * style(update): autoware tools icons (`#6351 <https://github.com/autowarefoundation/autoware_tools/issues/6351>`_)
  * fix(readme): add acknowledgement for material icons in tool plugins (`#6354 <https://github.com/autowarefoundation/autoware_tools/issues/6354>`_)
  * feat(mission_planner_rviz_plugin): create mission planner tool (`#6362 <https://github.com/autowarefoundation/autoware_tools/issues/6362>`_)
  * feat(mission_planner_rviz_plugin): create package
  * fix copyright
  * add interrupted state
  * use full license text instead of spdx
  ---------
  * fix(manual_controller): set PARK gear when going from reverse to drive (`#6230 <https://github.com/autowarefoundation/autoware_tools/issues/6230>`_)
  * feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/autowarefoundation/autoware_tools/issues/6823>`_)
  * feat(launch): add logging_demo::LoggerConfig into container
  * fix(logger_level_reconfigure_plugin): fix yaml
  * feat(logging_level_configure): add composable node
  ---------
  * revert: "feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/autowarefoundation/autoware_tools/issues/6823>`_)" (`#6842 <https://github.com/autowarefoundation/autoware_tools/issues/6842>`_)
  This reverts commit 9d045335d8e3763984bce8dea92f63de3423ebde.
  * docs(tier4_logging_level_configure_rviz_plugin): update document (`#6720 <https://github.com/autowarefoundation/autoware_tools/issues/6720>`_)
  * docs(tier4_logging_level_configure_rviz_plugin): update document
  * fix spell check
  * fix Warning
  ---------
  * refactor(bpp, avoidance): remove unnecessary verbose flag (`#6822 <https://github.com/autowarefoundation/autoware_tools/issues/6822>`_)
  * refactor(avoidance): logger small change
  * refactor(bpp): remove verbose flag
  ---------
  * feat(tier4_screen_capture_panel): add new service to capture screen shot (`#6867 <https://github.com/autowarefoundation/autoware_tools/issues/6867>`_)
  * feat(tier4_screen_capture_panel): add new service to capture screen shot
  * docs(tier4_screen_capture_rviz_plugin): update readme
  ---------
  * refactor(lane_change): fix logger (`#6873 <https://github.com/autowarefoundation/autoware_tools/issues/6873>`_)
  * fix(route_handler): add logger (`#6888 <https://github.com/autowarefoundation/autoware_tools/issues/6888>`_)
  * fix(route_handler): add logger
  * fix indent
  ---------
  * docs(tier4_simulated_clock_rviz_plugin): update how to use (`#6914 <https://github.com/autowarefoundation/autoware_tools/issues/6914>`_)
  * docs(tier4_simulated_clock_rviz_plugin): update how to use
  * fixed tabbed warning
  * fix warning not working
  * Fix bullet list
  ---------
  * refactor(bpp): path shifter clang tidy and logging level configuration (`#6917 <https://github.com/autowarefoundation/autoware_tools/issues/6917>`_)
  * fix(accel_brake_calibrator): fix to set service name and exception failure (`#6973 <https://github.com/autowarefoundation/autoware_tools/issues/6973>`_)
  * add service
  * fix exception
  * fix style
  * refactor(motion_utils): supress log message with rclcpp logging (`#6955 <https://github.com/autowarefoundation/autoware_tools/issues/6955>`_)
  * refactor(motion_utils): supress log message with rclcpp logging
  * remove std string
  ---------
  ---------
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  * update smoothing of start/end-point
  * Add simplified lateral acc limit
  * update debug plot
  * add gif
  * Update README.md
  * Update README.md
  * use ros param
  * ros param
  * update
  * update image
  * Update README.md
  * Add velocity noise
  * Update acc and steer noises
  * update rviz
  * update resource
  * Update README.md
  * Update README.md
  * Update control_data_collecting_tool/README.md
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * update readme
  * Update bag2lanelet/scripts/bag2map.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * modify sentence
  * chore(rtc_manager_panel): update module name (`#29 <https://github.com/autowarefoundation/autoware_tools/issues/29>`_)
  * chore(rtc_manager_panel): update module name
  * chore(gitignore): remove py cache
  ---------
  * improve pure pursuit
  * Update control_data_collecting_tool/scripts/data_collecting_trajectory_publisher.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * remove deprecated linearized pure pursuit
  * clean up traj pub
  * Reduce speed for large lateral error
  * Improve how to obtain the trajectory closest point
  * update readme
  * rename variables
  * compute trajectory only when receive data collecting area info
  * smoothing target yaw
  * add yaw error threshold
  * remove rviz config
  * renew png and gif
  * add arrow on png
  * feat(driving_environment_analyzer): remove dependency to autoware_auto_tf2 (`#31 <https://github.com/autowarefoundation/autoware_tools/issues/31>`_)
  ---------
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  * fix markdownlint in README
  * add maitainers
  * update msgs
  * add linearized pure pursuit control
  * change default params
  * add stop automatic driving
  * update control cmd limit
  * [bug fix] recompute trajectory when changing rosparam online
  * fix dead link
  * run pre-commit
  * fix spell
  * Update bag2lanelet/scripts/bag2map.py
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: masayukiaino <101699734+masayukiaino@users.noreply.github.com>
  * refresh cool down for setting initial pose in psim.
  * style(pre-commit): autofix
  * parameterize publishing_speed_factor
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: Fumiya Watanabe <rej55.g@gmail.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Toru Hishinuma <92719065+th1991-01@users.noreply.github.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <shumpei.wakabayashi@tier4.jp>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: dmoszynski <121798334+dmoszynski@users.noreply.github.com>
  Co-authored-by: Mehmet Dogru <48479081+mehmetdogru@users.noreply.github.com>
  Co-authored-by: Vincent Richard <richard-v@macnica.co.jp>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Khalil Selyan <36904941+KhalilSelyan@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: masayukiaino <101699734+masayukiaino@users.noreply.github.com>
* fix: autoware_msgs (`#68 <https://github.com/autowarefoundation/autoware_tools/issues/68>`_)
* feat(planning_debug_tools): add cpu_usage_checker.py (`#66 <https://github.com/autowarefoundation/autoware_tools/issues/66>`_)
  * feat(planning_debug_tools): add cpu_usage_checker.py
  * update README
  ---------
* fix(perception_replayer): support two kinds of old messages in rosbags (`#63 <https://github.com/autowarefoundation/autoware_tools/issues/63>`_)
  * fix(percerption_replayer): make sure that old message in rosbags can be converted to new type correctly (`#60 <https://github.com/autowarefoundation/autoware_tools/issues/60>`_)
  * ensure that old message in rosbagcan be converted to new type correctly.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * support two kinds of old rosbag
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(percerption_replayer): make sure that old message in rosbags can be converted to new type correctly (`#60 <https://github.com/autowarefoundation/autoware_tools/issues/60>`_)
  * ensure that old message in rosbagcan be converted to new type correctly.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: change_perception_reproducer_parameter (`#57 <https://github.com/autowarefoundation/autoware_tools/issues/57>`_)
  * change reproducer cool down to 80 s
  * change default parameter.
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#59 <https://github.com/autowarefoundation/autoware_tools/issues/59>`_)
  * refactor(universe_utils): add autoware namespace
  style(pre-commit): autofix
  * refactor(motion_utils): add autoware namespace
  ---------
* refactor(motion_utils)!: add autoware prefix and include dir (`#53 <https://github.com/autowarefoundation/autoware_tools/issues/53>`_)
  style(pre-commit): autofix
* refactor(autoware_universe_utils)!: rename tier4_autoware_utils to autoware_universe_utils (`#52 <https://github.com/autowarefoundation/autoware_tools/issues/52>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(perception_replayer): add a new perception_reproducer. (`#44 <https://github.com/autowarefoundation/autoware_tools/issues/44>`_)
  * fix a bug that perception_replayer/perception_reproducer can't publish old 'autoware_auto_perception_msgs' in old rosbags.
  * hard reset to fix conflict.
  * replace perception_reproducer_v2,py to perception_reproducer.py, update README
  ---------
* fix(perception_reproducer): support reproduce/replay old rosbags (`#40 <https://github.com/autowarefoundation/autoware_tools/issues/40>`_)
  * fix a bug that perception_replayer/perception_reproducer can't publish old 'autoware_auto_perception_msgs' in old rosbags.
  * style(pre-commit): autofix
  * fix some spell errors.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(perception_replayer):  order rosbags by starting_time (`#43 <https://github.com/autowarefoundation/autoware_tools/issues/43>`_)
  * feat(simulator_compatibility_test): temporariy remove tests for control_mode_command (`#34 <https://github.com/autowarefoundation/autoware_tools/issues/34>`_)
  * fix: order rosbags by starting_time instead of file name.
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: change from autoware_auto_msgs to autoware_msgs (`#30 <https://github.com/autowarefoundation/autoware_tools/issues/30>`_)
  * feat!: replace autoware_auto_msgs with autoware_msgs
  * style(pre-commit): autofix
  * feat: port remaining autoware_auto_msgs to autoware_msgs  (`#32 <https://github.com/autowarefoundation/autoware_tools/issues/32>`_)
  * feat: port remaining autoware_auto_msgs to autoware_msgs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * remove <depend>autoware_msgs</depend>
  * remove non-existent dep
  ---------
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: M. Fatih Cırıt <mfc@leodrive.ai>
* add gitignore for .pyc and remove already tracked .pyc files (`#19 <https://github.com/autowarefoundation/autoware_tools/issues/19>`_)
* feat(perception_replayer): add a button to publish the goal pose (`#15 <https://github.com/autowarefoundation/autoware_tools/issues/15>`_)
* docs(planning_debug_tools): fix reactive script example (`#16 <https://github.com/autowarefoundation/autoware_tools/issues/16>`_)
  * docs(planning_debug_tools): fix reactive script example
  * fix(repos): add build dependency
  ---------
* fix(perception_replyer): fix dying when no traffic signals (`#14 <https://github.com/autowarefoundation/autoware_tools/issues/14>`_)
* feat: move tools from autoware.universe (`#8 <https://github.com/autowarefoundation/autoware_tools/issues/8>`_)
  * feat: move tools from autoware.universe
  * update build_depends.repos
  * update build_depends.repos
  ---------
* Contributors: Kem (TiankuiXian), Kosuke Takeuchi, Kyoichi Sugahara, Maxime CLEMENT, Ryohsuke Mitsudome, Satoshi OTA, Takayuki Murooka, Tiankui Xian, Yukihiro Saito, Yutaka Kondo, awf-autoware-bot[bot]

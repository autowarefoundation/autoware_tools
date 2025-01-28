^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tier4_target_object_type_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* chore: sync files (`#11 <https://github.com/sasakisasaki/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(rviz_plugin): move peripheral rviz plugin (`#27 <https://github.com/sasakisasaki/autoware_tools/issues/27>`_)
  * feat:  calibrator tier4 pkg name (`#200 <https://github.com/sasakisasaki/autoware_tools/issues/200>`_)
  * feat: change names
  * feat: move pkg to common
  * feat: change pkg name
  * fix: library path (`#225 <https://github.com/sasakisasaki/autoware_tools/issues/225>`_)
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  * feat: add rviz plugin to publish and control the simulated clock (`#349 <https://github.com/sasakisasaki/autoware_tools/issues/349>`_)
  * Add tier4_clock_rviz_plugin to publish&control the sim clock in rviz
  * Add step control
  * Fix precommit
  * Update documentation
  * Fix spellcheck
  * Update plugin description and icon
  * Rename package
  * Fix bug with long duration jumps (high speed + low rate)
  * ci: check include guard (`#438 <https://github.com/sasakisasaki/autoware_tools/issues/438>`_)
  * ci: check include guard
  * apply pre-commit
  * Update .pre-commit-config.yaml
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * fix: pre-commit
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * chore: sync files (`#629 <https://github.com/sasakisasaki/autoware_tools/issues/629>`_)
  * chore: sync files
  * ci(pre-commit): autofix
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * style: fix format of package.xml (`#844 <https://github.com/sasakisasaki/autoware_tools/issues/844>`_)
  * refactor: use autoware cmake (`#849 <https://github.com/sasakisasaki/autoware_tools/issues/849>`_)
  * remove autoware_auto_cmake
  * add build_depend of autoware_cmake
  * use autoware_cmake in CMakeLists.txt
  * fix bugs
  * fix cmake lint errors
  * chore: upgrade cmake_minimum_required to 3.14 (`#856 <https://github.com/sasakisasaki/autoware_tools/issues/856>`_)
  * fix(accel_brake_map_calibrator): rviz panel type (`#895 <https://github.com/sasakisasaki/autoware_tools/issues/895>`_)
  * fixed panel type
  * modified instruction for rosbag replay case
  * modified update_map_dir service name
  * feat: add manual controller (`#989 <https://github.com/sasakisasaki/autoware_tools/issues/989>`_)
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
  * feat(manual_controller): support backward driving (`#1119 <https://github.com/sasakisasaki/autoware_tools/issues/1119>`_)
  * feat(manual_controller): support backward driving
  * chore: unite variable
  * feat: add screen capture package (`#1177 <https://github.com/sasakisasaki/autoware_tools/issues/1177>`_)
  * feat: add screen capture package
  * feat: add state to current process
  * style: refactor codes
  * fix: time
  * feat: add mp4 writer and finalize folders
  * feat: add trigger by service
  * feat: update recorder
  * fix: change codec of mp4 to h264 (`#1262 <https://github.com/sasakisasaki/autoware_tools/issues/1262>`_)
  * feat: optimize screen capture panel (`#1470 <https://github.com/sasakisasaki/autoware_tools/issues/1470>`_)
  * feat: optimize screen capture panel
  * apply pre-commit
  * fix: remove unused check of rviz plugin version (`#1474 <https://github.com/sasakisasaki/autoware_tools/issues/1474>`_)
  * fix(rviz_plugin): fix screen capture plugin (`#1492 <https://github.com/sasakisasaki/autoware_tools/issues/1492>`_)
  * refactor(tier4_calibration_rviz_plugin): apply clang-tidy (`#1596 <https://github.com/sasakisasaki/autoware_tools/issues/1596>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy (`#1597 <https://github.com/sasakisasaki/autoware_tools/issues/1597>`_)
  * refactor(tier4_control_rviz_plugin): apply clang-tidy
  * revert: readability-identifier-naming
  * revert(tier4_calibration_rviz_plugin): readability-identifier-naming (`#1618 <https://github.com/sasakisasaki/autoware_tools/issues/1618>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix release process to handle video writer correctly (`#1622 <https://github.com/sasakisasaki/autoware_tools/issues/1622>`_)
  * refactor(tier4_screen_capture_rviz_plugin):apply clang-tidy (`#1649 <https://github.com/sasakisasaki/autoware_tools/issues/1649>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check (`#1790 <https://github.com/sasakisasaki/autoware_tools/issues/1790>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix spell check
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: add rtc  manager rviz plugin (`#1936 <https://github.com/sasakisasaki/autoware_tools/issues/1936>`_)
  * feat: add rtc  manager rviz plugin
  * chore: cosmetic change
  * chore: remove indent
  * feat: add rtc safe unsafe color
  * fix: typo
  * chore: simplify layout
  * feat: update rtc panel
  * feat(rtc_manager_panel): add rtc all approval (`#2009 <https://github.com/sasakisasaki/autoware_tools/issues/2009>`_)
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
  * feat(rtc_manager_rviz_plugin): add_indivisual_exe (`#2021 <https://github.com/sasakisasaki/autoware_tools/issues/2021>`_)
  * feat(tier4_screen_capture_rviz_plugin): add prefix to video name (`#2038 <https://github.com/sasakisasaki/autoware_tools/issues/2038>`_)
  feat: add  prefix
  * feat(tier4_debug_rviz_plugin): pie chart with float32 multi array stamped (`#2055 <https://github.com/sasakisasaki/autoware_tools/issues/2055>`_)
  * feat(tier4_debug_rviz_plugin): add ros2 pie chart with Float32MultiArrayStamped
  * update README.md
  * fix typo
  * fixed license
  * fix
  * removed unnecessary include
  * fix
  * fix
  * fix(tier4_control_rviz_plugin): add time stamp for control command (`#2154 <https://github.com/sasakisasaki/autoware_tools/issues/2154>`_)
  * fix(rtc_manager_rviz_plugin): size check (`#2163 <https://github.com/sasakisasaki/autoware_tools/issues/2163>`_)
  * feat(behavior_path_planner): external request lane change (`#2442 <https://github.com/sasakisasaki/autoware_tools/issues/2442>`_)
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
  * chore(rtc_manager_rviz_plugin): add code owner (`#2792 <https://github.com/sasakisasaki/autoware_tools/issues/2792>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status (`#2791 <https://github.com/sasakisasaki/autoware_tools/issues/2791>`_)
  * feat(rtc_manager_rviz_plugin): add the number of rtc status
  * chore: simplify layout
  ---------
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  * feat(automatic_goal): add automatic goal rviz plugin (`#3031 <https://github.com/sasakisasaki/autoware_tools/issues/3031>`_)
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
  * feat(rtc_manager_rviz_plugin): add avoidance by lc (`#3118 <https://github.com/sasakisasaki/autoware_tools/issues/3118>`_)
  * fix(tier4_screen_capture_rviz_plugin): fix extra/missing naming components (`#3207 <https://github.com/sasakisasaki/autoware_tools/issues/3207>`_)
  * chore: sync files (`#3227 <https://github.com/sasakisasaki/autoware_tools/issues/3227>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat: change external lane change rtc name (`#3259 <https://github.com/sasakisasaki/autoware_tools/issues/3259>`_)
  * feat: change external lane change rtc name
  * update config
  ---------
  * feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#3458 <https://github.com/sasakisasaki/autoware_tools/issues/3458>`_)
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
  * fix(rtc_manager_rviz_plugin): update panel visualization properly (`#3517 <https://github.com/sasakisasaki/autoware_tools/issues/3517>`_)
  * refactor(behavior_path_planner): rename pull_over to goal_planner (`#3501 <https://github.com/sasakisasaki/autoware_tools/issues/3501>`_)
  * build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/sasakisasaki/autoware_tools/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  * build(iron): remove rmw_qos_profile_t (`#3809 <https://github.com/sasakisasaki/autoware_tools/issues/3809>`_)
  * refactor(start_planner): rename pull out to start planner (`#3908 <https://github.com/sasakisasaki/autoware_tools/issues/3908>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name (`#4539 <https://github.com/sasakisasaki/autoware_tools/issues/4539>`_)
  * fix(accel_brake_map_calibrator_button_panel): fix calibration service name
  * misc
  ---------
  * feat(rviz_plugin): add target object type display (`#4855 <https://github.com/sasakisasaki/autoware_tools/issues/4855>`_)
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
  * fix(rtc_manager_panel): fix panel chattering (`#4988 <https://github.com/sasakisasaki/autoware_tools/issues/4988>`_)
  * build(tier4_target_object_type_rviz_plugin): add missing cv_bridge dependency (`#5000 <https://github.com/sasakisasaki/autoware_tools/issues/5000>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level (`#5112 <https://github.com/sasakisasaki/autoware_tools/issues/5112>`_)
  * feat(logging_level_configure): add rviz plugin to change logging level
  * change file names
  * move initialization code from constructor to onInitialize
  * add maintainer
  * add maintainer
  * fix include
  ---------
  * feat(logger_level_reconfigure_plugin): use node interface and some cosmetic updates (`#5204 <https://github.com/sasakisasaki/autoware_tools/issues/5204>`_)
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
  * refactor(lane_change): add debug log (`#5308 <https://github.com/sasakisasaki/autoware_tools/issues/5308>`_)
  * docs(logger_level_reconfigure): update readme (`#5471 <https://github.com/sasakisasaki/autoware_tools/issues/5471>`_)
  * feat(localization): enable logging_level_configure (`#5487 <https://github.com/sasakisasaki/autoware_tools/issues/5487>`_)
  * feat(localization): enable logging_level_configure
  * style(pre-commit): autofix
  * update logger config
  * fix pre-commit
  * add tier4_autoware_utils in dependency
  * add tier4_autoware_utils in dependency
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * Logger level update (`#5494 <https://github.com/sasakisasaki/autoware_tools/issues/5494>`_)
  * address ordering
  * add grouping
  * remove unused comment
  ---------
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button (`#5666 <https://github.com/sasakisasaki/autoware_tools/issues/5666>`_)
  * feat(logging_level_configure_rviz_plugin): add autoware_util logger button
  * add for control
  ---------
  * refactor(lane_change): standardizing lane change logger name (`#5899 <https://github.com/sasakisasaki/autoware_tools/issues/5899>`_)
  * feat(tier4_logging_level_configure_rviz_plugin): add goal/start planner (`#5978 <https://github.com/sasakisasaki/autoware_tools/issues/5978>`_)
  * chore: set log level of debug printing in rviz plugin to DEBUG (`#5996 <https://github.com/sasakisasaki/autoware_tools/issues/5996>`_)
  * feat(rviz_plugin): add string visualization plugin (`#6100 <https://github.com/sasakisasaki/autoware_tools/issues/6100>`_)
  * feat(tier4_automatic_goal_rviz_plugin): make it possible to register checkpoints (`#6153 <https://github.com/sasakisasaki/autoware_tools/issues/6153>`_)
  * chore(object_type_panel): update rosparam name config (`#6347 <https://github.com/sasakisasaki/autoware_tools/issues/6347>`_)
  * style(update): autoware tools icons (`#6351 <https://github.com/sasakisasaki/autoware_tools/issues/6351>`_)
  * fix(readme): add acknowledgement for material icons in tool plugins (`#6354 <https://github.com/sasakisasaki/autoware_tools/issues/6354>`_)
  * feat(mission_planner_rviz_plugin): create mission planner tool (`#6362 <https://github.com/sasakisasaki/autoware_tools/issues/6362>`_)
  * feat(mission_planner_rviz_plugin): create package
  * fix copyright
  * add interrupted state
  * use full license text instead of spdx
  ---------
  * fix(manual_controller): set PARK gear when going from reverse to drive (`#6230 <https://github.com/sasakisasaki/autoware_tools/issues/6230>`_)
  * feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/sasakisasaki/autoware_tools/issues/6823>`_)
  * feat(launch): add logging_demo::LoggerConfig into container
  * fix(logger_level_reconfigure_plugin): fix yaml
  * feat(logging_level_configure): add composable node
  ---------
  * revert: "feat(logger_level_configure): make it possible to change level of container logger (`#6823 <https://github.com/sasakisasaki/autoware_tools/issues/6823>`_)" (`#6842 <https://github.com/sasakisasaki/autoware_tools/issues/6842>`_)
  This reverts commit 9d045335d8e3763984bce8dea92f63de3423ebde.
  * docs(tier4_logging_level_configure_rviz_plugin): update document (`#6720 <https://github.com/sasakisasaki/autoware_tools/issues/6720>`_)
  * docs(tier4_logging_level_configure_rviz_plugin): update document
  * fix spell check
  * fix Warning
  ---------
  * refactor(bpp, avoidance): remove unnecessary verbose flag (`#6822 <https://github.com/sasakisasaki/autoware_tools/issues/6822>`_)
  * refactor(avoidance): logger small change
  * refactor(bpp): remove verbose flag
  ---------
  * feat(tier4_screen_capture_panel): add new service to capture screen shot (`#6867 <https://github.com/sasakisasaki/autoware_tools/issues/6867>`_)
  * feat(tier4_screen_capture_panel): add new service to capture screen shot
  * docs(tier4_screen_capture_rviz_plugin): update readme
  ---------
  * refactor(lane_change): fix logger (`#6873 <https://github.com/sasakisasaki/autoware_tools/issues/6873>`_)
  * fix(route_handler): add logger (`#6888 <https://github.com/sasakisasaki/autoware_tools/issues/6888>`_)
  * fix(route_handler): add logger
  * fix indent
  ---------
  * docs(tier4_simulated_clock_rviz_plugin): update how to use (`#6914 <https://github.com/sasakisasaki/autoware_tools/issues/6914>`_)
  * docs(tier4_simulated_clock_rviz_plugin): update how to use
  * fixed tabbed warning
  * fix warning not working
  * Fix bullet list
  ---------
  * refactor(bpp): path shifter clang tidy and logging level configuration (`#6917 <https://github.com/sasakisasaki/autoware_tools/issues/6917>`_)
  * fix(accel_brake_calibrator): fix to set service name and exception failure (`#6973 <https://github.com/sasakisasaki/autoware_tools/issues/6973>`_)
  * add service
  * fix exception
  * fix style
  * refactor(motion_utils): supress log message with rclcpp logging (`#6955 <https://github.com/sasakisasaki/autoware_tools/issues/6955>`_)
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
* Contributors: Mamoru Sobue, awf-autoware-bot[bot]

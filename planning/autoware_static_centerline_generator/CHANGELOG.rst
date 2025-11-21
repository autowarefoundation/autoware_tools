^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_static_centerline_generator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2025-11-16)
------------------
* feat(static_centerline): add jitter validation (`#305 <https://github.com/autowarefoundation/autoware_tools/issues/305>`_)
  * feat(static_centerline): add jitter validation
  * fix: extra comment
  * fix
  * fix readme and comment
  * fix
  * fix: optimization_trajectory_based_centerline
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* fix(static_centerline_generator): inside centerline start point in start lanelet (`#274 <https://github.com/autowarefoundation/autoware_tools/issues/274>`_)
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* feat(static_centerline): enabled input of lanelet_sequence (`#299 <https://github.com/autowarefoundation/autoware_tools/issues/299>`_)
  * feat(static_centerline): enabled input of lanelet_sequence
  * pre-commit
  * fix cast
  * pre-commit
  * fix readme
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* chore: sync files (`#278 <https://github.com/autowarefoundation/autoware_tools/issues/278>`_)
* Contributors: Kazunori-Nakajima, awf-autoware-bot[bot]

0.3.0 (2025-08-11)
------------------
* fix(autoware_static_centerline_generator): fix test dependency (`#285 <https://github.com/autowarefoundation/autoware_tools/issues/285>`_)
  * fix(autoware_static_centerline_generator): fix test dependency
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* ci(build-and-test): fix ci issues (`#283 <https://github.com/autowarefoundation/autoware_tools/issues/283>`_)
* chore: sync files (`#214 <https://github.com/autowarefoundation/autoware_tools/issues/214>`_)
* fix(static_centerline_generator): do not embed a single point centerline (`#271 <https://github.com/autowarefoundation/autoware_tools/issues/271>`_)
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* fix(autoware_static_centerline_generator): fix the name of the executable in static_centerline_generator.launch.xml (`#258 <https://github.com/autowarefoundation/autoware_tools/issues/258>`_)
  fix the name of the executable in static_centerline_generator.launch.xml
* feat(static_centerline_generator): add various tests (`#250 <https://github.com/autowarefoundation/autoware_tools/issues/250>`_)
  * refactor
  * update test
  * fix
  * update
  * fix cmake
  * update cmake
  * fix
  * tmp: add debug print
  * update comment of cmake
  * update rviz
  * fix
  ---------
* fix(static_centerline_generator): collision check on overlapped lanes and more visualization for debugging (`#249 <https://github.com/autowarefoundation/autoware_tools/issues/249>`_)
  * fix collision check on the overlapped lanes
  * fix
  * update README.md
  * remove cerr
  * fix
  * visualize centerline
  * visualize start/goal pose
  * fix whole_optimized_traj_points connection
  * use lanelet::Id
  * update sample script
  * fix test
  ---------
* feat(static_centerline_generator): add goal modify function (`#248 <https://github.com/autowarefoundation/autoware_tools/issues/248>`_)
  * feat(static_centerline_generator): add goal function
  * pre-commit
  * fix comment content
  * fix spell-check
  * refactor
  * refactoring version 1
  * refactoring vesion 2
  * style(pre-commit): autofix
  * fix pre-commit
  * del route_ptr member variable
  * refactoring version 3
  * merge
  * visualize iterative trajectory planning
  * fix
  * update rviz
  * use route instead of route_lane_ids
  * fix
  * fix goal connection
  * fix TODO
  * fix
  * decrease wait_time_during_planning_iteration
  * fix
  * modify test
  * organize test
  * fix
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(static_centerline_generator): visualize iterative planning results and introduce glog (`#241 <https://github.com/autowarefoundation/autoware_tools/issues/241>`_)
  * add glog
  * fix
  * visualize iterative trajectory planning
  * fix conflict
  * fix
  * fix cmake
  * add glog in build_depends.repos
  * fix build_depends.repos
  ---------
* Contributors: Kazunori-Nakajima, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taiki Yamada, Takayuki Murooka, awf-autoware-bot[bot]

0.2.0 (2025-03-24)
------------------
* fix static_centerline parameter path (`#218 <https://github.com/autowarefoundation/autoware_tools/issues/218>`_)
* fix(autoware_static_centerline_generator): changes to support modifications made to autoware.core (`#225 <https://github.com/autowarefoundation/autoware_tools/issues/225>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#206 <https://github.com/autowarefoundation/autoware_tools/issues/206>`_)
  * feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, Zhanhong Yan

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* feat(autoware_static_centerline_generator): rename autoware_mission_planner to autoware_mission_planner_universe (`#195 <https://github.com/autowarefoundation/autoware_tools/issues/195>`_)
* test(static_centerline_generator): add launch test with autoware_sample_vehicle_launch package (`#190 <https://github.com/autowarefoundation/autoware_tools/issues/190>`_)
  * test(static_centerline_generator): add launch test with autoware_sample_vehicle_launch package
  * fix
  * fix dependency to autoware_launch
  ---------
* feat: add autoware_static_centerline_generator (`#189 <https://github.com/autowarefoundation/autoware_tools/issues/189>`_)
  * feat: add autoware_static_centerline_generator
  * fix
  ---------
* Contributors: Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo

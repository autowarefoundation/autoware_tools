^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package parameter_estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix(deviation_evaluator, parameter_estimator): migrate map launch to component_map.launch.xml (`#466 <https://github.com/autowarefoundation/autoware_tools/issues/466>`_)
  `autowarefoundation/autoware_launch#1980 <https://github.com/autowarefoundation/autoware_launch/issues/1980>`_ renamed tier4_map_launch to
  autoware_map_launch and decomposed its map.launch.xml aggregator into per-node
  launch files, so there is no more single map.launch.xml; the exported map
  composition is now autoware_launch/launch/components/component_map.launch.xml.
  These launch files still included
  $(find-pkg-share tier4_map_launch)/launch/map.launch.xml, so the pre-commit
  "check package depends" hook kept re-adding the removed tier4_map_launch as an
  exec_depend even after `#465 <https://github.com/autowarefoundation/autoware_tools/issues/465>`_ dropped it from package.xml -- leaving the
  dependency unresolvable and autoware.universe's
  build-and-test-packages-above-differential red.
  Switch both launch files to component_map.launch.xml (from autoware_launch, an
  existing exec_depend), which resolves the map paths from map_path and loads the
  map parameters internally from autoware_map_config. With the tier4_map_launch /
  autoware_map_launch references gone, drop both from package.xml; the map nodes
  remain available transitively through autoware_launch.
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* fix(deviation_evaluator, parameter_estimator): rename tier4_map_launch dependency to autoware_map_launch (`#465 <https://github.com/autowarefoundation/autoware_tools/issues/465>`_)
  * fix(deviation_evaluator, parameter_estimator): rename tier4_map_launch dependency to autoware_map_launch
  `autowarefoundation/autoware_launch#1980 <https://github.com/autowarefoundation/autoware_launch/issues/1980>`_ (merged) renamed the tier4_map_launch
  package to autoware_map_launch. deviation_evaluator and parameter_estimator still
  exec_depend on the now-removed tier4_map_launch, so rosdep can no longer resolve
  the key and autoware.universe's build-and-test-packages-above-differential job
  fails at dependency resolution on any PR whose reverse-dependency set includes
  these tools.
  Point the exec_depend at the new autoware_map_launch package (kept in sorted
  order). Both packages already exec_depend on autoware_launch, which lives in the
  same repository as autoware_map_launch, so the new key resolves.
  Note: the launch files still include
  `$(find-pkg-share tier4_map_launch)/launch/map.launch.xml`, which was decomposed
  into per-node launch files in autoware_map_launch. Updating those includes is a
  separate runtime follow-up; this change fixes the packaging/rosdep breakage.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masaya Kataoka <cld-masaya.kataoka@tier4.jp>
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Masaya Kataoka

0.7.0 (2026-05-01)
------------------

0.6.0 (2026-02-14)
------------------

0.5.0 (2025-12-30)
------------------

0.4.0 (2025-11-16)
------------------

0.3.0 (2025-08-11)
------------------
* fix(parameter_estimatior): update package dependencies (`#280 <https://github.com/autowarefoundation/autoware_tools/issues/280>`_)
  * fix(parameter_estimatior): update package dependencies
  * style(pre-commit): autofix
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#214 <https://github.com/autowarefoundation/autoware_tools/issues/214>`_)
* Contributors: Kazunori-Nakajima, awf-autoware-bot[bot]

0.2.0 (2025-03-24)
------------------

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* feat(parameter_estimator)!: replace tier4_debug_msgs with tier4_internal_debug_msgs (`#198 <https://github.com/autowarefoundation/autoware_tools/issues/198>`_)
  * feat(parameter_estimator)!: replace tier4_debug_msgs with tier4_internal_debug_msgs
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(global_parameter_loader): prefix package and namespace with autoware (`#159 <https://github.com/autowarefoundation/autoware_tools/issues/159>`_)
* fix: some fix on calibration_adapter and parameter_estimator (`#127 <https://github.com/autowarefoundation/autoware_tools/issues/127>`_)
  * some fix on calibration_adapter and parameter_estimator
  * minor fix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
* Changed from map.launch.py to map.launch.xml (`#71 <https://github.com/autowarefoundation/autoware_tools/issues/71>`_)
* feat: add vehicle, control and localization calibration tools (`#22 <https://github.com/autowarefoundation/autoware_tools/issues/22>`_)
  * add vehicle control localization tools
  * style(pre-commit): autofix
  * fix pre commit
  * Fix to pass cspell about localization
  * ignore words from spellcheck
  * ROS2 -> ROS 2
  * feat: convert autoware_auto_msg to autoware_msg (`#42 <https://github.com/autowarefoundation/autoware_tools/issues/42>`_)
  * merge
  * delete autoware_auto
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * add prefix to vehicle_info_utils
  * style(pre-commit): autofix
  * fix: add s to VehicleInfoUtil
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shintaro Sakoda <shintaro.sakoda@tier4.jp>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: tomoya.kimura <tomoya.kimura@tier4.jp>
* Contributors: Esteve Fernandez, KeiNakazato, Ryohsuke Mitsudome, SakodaShintaro, Yutaka Kondo, yabuta

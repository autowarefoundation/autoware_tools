^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package driving_environment_analyzer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat(autoware_lanelet2_extension): replace ported autoware_lanelet2_extension in autoware_tools (`#318 <https://github.com/autowarefoundation/autoware_tools/issues/318>`_)
  replace getLaneletLength3d with length3d
* fix(driving_environment_analyzer): ignore deprecate warning (`#316 <https://github.com/autowarefoundation/autoware_tools/issues/316>`_)
* Contributors: Mamoru Sobue, Sarun MUKDAPITAK

0.4.0 (2025-11-16)
------------------

0.3.0 (2025-08-11)
------------------
* chore: sync files (`#214 <https://github.com/autowarefoundation/autoware_tools/issues/214>`_)
* Contributors: awf-autoware-bot[bot]

0.2.0 (2025-03-24)
------------------
* fix(driving_environment_analyzer): add missing include files for autoware_universe_utils (`#212 <https://github.com/autowarefoundation/autoware_tools/issues/212>`_)
* Contributors: Ryohsuke Mitsudome

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* fix: to use autoware_lanelet2_extension (`#178 <https://github.com/autowarefoundation/autoware_tools/issues/178>`_)
  * Fixed to use autoware_lanelet2_extension
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* add autoware prefix to map_loader (`#122 <https://github.com/autowarefoundation/autoware_tools/issues/122>`_)
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
* fix(driving_environment_analyzer): remove unused autoware_interpolation dependency (`#123 <https://github.com/autowarefoundation/autoware_tools/issues/123>`_)
* refactor: update signal_processing downstream dependencies (`#110 <https://github.com/autowarefoundation/autoware_tools/issues/110>`_)
* add autoware\_ prefix to map_projection_loader (`#93 <https://github.com/autowarefoundation/autoware_tools/issues/93>`_)
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
* refactor(route_handler)!: include autoware_route_handler (`#54 <https://github.com/autowarefoundation/autoware_tools/issues/54>`_)
  Co-authored-by: M. Fatih Cırıt <xmfcx@users.noreply.github.com>
* fix(driving_environment_analyzer): fix unused package dependency (`#46 <https://github.com/autowarefoundation/autoware_tools/issues/46>`_)
  * fix: deps
  * fix: formatter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix package dependency, includes and access (`#38 <https://github.com/autowarefoundation/autoware_tools/issues/38>`_)
* fix(driving_environment_analyzer): fixed depend pkg name about behavior_path_planner_common (`#36 <https://github.com/autowarefoundation/autoware_tools/issues/36>`_)
  * Fixed depend pkg name from behavior_path_planner_common to autoware_behavior_path_planner_common
  * style(pre-commit): autofix
  * fix: add deps
  * fix: add deps
  * style(pre-commit): autofix
  * Added "autoware\_" to lane_departure_checker
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
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
* feat(driving_environment_analyzer): remove dependency to autoware_auto_tf2 (`#31 <https://github.com/autowarefoundation/autoware_tools/issues/31>`_)
* feat(driving_environment_analyzer): add rviz plugin (`#23 <https://github.com/autowarefoundation/autoware_tools/issues/23>`_)
  * feat(driving_environment_analyzer): add rviz plugin
  * feat(driving_environment_analyzer): output csv file
  ---------
* feat(driving_environment_analyzer): add new package (`#13 <https://github.com/autowarefoundation/autoware_tools/issues/13>`_)
  * feat(driving_environment_analyzer): add package
  * fix: typo
  * fix: rename func
  ---------
* Contributors: Esteve Fernandez, Kosuke Takeuchi, M. Fatih Cırıt, Masaki Baba, Ryohsuke Mitsudome, SakodaShintaro, Satoshi OTA, Takayuki Murooka, Yukihiro Saito, Yutaka Kondo, awf-autoware-bot[bot], mkquda

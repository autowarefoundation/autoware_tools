^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simulator_compatibility_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-05-01)
------------------
* chore: sync files (`#311 <https://github.com/autowarefoundation/autoware_tools/issues/311>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(simulator_compatibility_test): use pytest runner to fix Jazzy CI failure (`#390 <https://github.com/autowarefoundation/autoware_tools/issues/390>`_)
  Python 3.12 (Jazzy) changed unittest to return exit code 5 when no
  tests are collected, while Python 3.10 (Humble) returns 0. The empty
  tests_require caused colcon to use the unittest runner, which never
  discovered any tests since the test classes don't inherit
  unittest.TestCase. The colcon pytest runner handles exit code 5
  gracefully.
  Also add norecursedirs to prevent pytest from discovering simulator
  tests that require a real MORAI connection.
* Contributors: Mete Fatih Cırıt, awf-autoware-bot[bot]

0.6.0 (2026-02-14)
------------------

0.5.0 (2025-12-30)
------------------

0.4.0 (2025-11-16)
------------------

0.3.0 (2025-08-11)
------------------

0.2.0 (2025-03-24)
------------------

0.1.0 (2025-01-28)
------------------
* fix: autoware_msgs (`#68 <https://github.com/autowarefoundation/autoware_tools/issues/68>`_)
* feat(simulator_compatibility_test): temporariy remove tests for control_mode_command (`#34 <https://github.com/autowarefoundation/autoware_tools/issues/34>`_)
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
* feat: move tools from autoware.universe (`#8 <https://github.com/autowarefoundation/autoware_tools/issues/8>`_)
  * feat: move tools from autoware.universe
  * update build_depends.repos
  * update build_depends.repos
  ---------
* Contributors: Kosuke Takeuchi, Ryohsuke Mitsudome, Takayuki Murooka, Yukihiro Saito

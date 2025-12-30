^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_position_error_evaluator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2025-12-30)
------------------
* fix: to be consistent version in all package.xml(s)
* feat(position_error_evaluator): add tool for evaluating position error (`#301 <https://github.com/autowarefoundation/autoware_tools/issues/301>`_)
  * autoware_evaluate_position_error_using_line_rviz_plugin
  * remove abs
  * style: reorder include section
  * feat: add some libraries
  * style: change line break position
  * style: Remove line breaks
  * style: Aligning the text
  * feat: Change Lane line extraction method
  * feat: Check angle if the segment is within line range
  * feat: Change map  line width
  * feat: output csv result file
  * feat: Change csv file naming
  If conflicted, The name is added sequential number
  * feat: take a screenshot after measurement
  * fix: Copyright messages
  * refactor: Replace magic number to const
  * refactor: Remove unused includes
  * feat: Output trajectory.csv of measured map points
  * feat: Undo just after measurement
  * doc: Add README
  * chore: Rename the package name
  * doc: Follow markdownlint on README
  * fix: cpplint error
  * style(pre-commit): autofix
  * chore: Add maintainer/author
  * fix: Remove needless module in CMakeLists
  * style: Replace forbidden/unknown words
  * style(pre-commit): autofix
  * feat: Not measure yaw difference if the stop line measurement is on
  Because the line is likely to be short to measure yaw difference accurately.
  * refactor: remove unnecessary dependency
  ---------
  Co-authored-by: Yamato Ando <yamato.ando@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Motz, github-actions

* fix: to be consistent version in all package.xml(s)
* feat(position_error_evaluator): add tool for evaluating position error (`#301 <https://github.com/autowarefoundation/autoware_tools/issues/301>`_)
  * autoware_evaluate_position_error_using_line_rviz_plugin
  * remove abs
  * style: reorder include section
  * feat: add some libraries
  * style: change line break position
  * style: Remove line breaks
  * style: Aligning the text
  * feat: Change Lane line extraction method
  * feat: Check angle if the segment is within line range
  * feat: Change map  line width
  * feat: output csv result file
  * feat: Change csv file naming
  If conflicted, The name is added sequential number
  * feat: take a screenshot after measurement
  * fix: Copyright messages
  * refactor: Replace magic number to const
  * refactor: Remove unused includes
  * feat: Output trajectory.csv of measured map points
  * feat: Undo just after measurement
  * doc: Add README
  * chore: Rename the package name
  * doc: Follow markdownlint on README
  * fix: cpplint error
  * style(pre-commit): autofix
  * chore: Add maintainer/author
  * fix: Remove needless module in CMakeLists
  * style: Replace forbidden/unknown words
  * style(pre-commit): autofix
  * feat: Not measure yaw difference if the stop line measurement is on
  Because the line is likely to be short to measure yaw difference accurately.
  * refactor: remove unnecessary dependency
  ---------
  Co-authored-by: Yamato Ando <yamato.ando@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Motz, github-actions

0.4.0 (2025-11-17)
------------------

0.3.0 (2025-08-11)
------------------

0.2.0 (2025-03-25)
------------------

0.1.0 (2025-01-29)
------------------

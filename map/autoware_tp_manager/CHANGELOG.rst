^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tp_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2025-12-30)
------------------

0.4.0 (2025-11-16)
------------------
* chore: update maintainer (`#304 <https://github.com/autowarefoundation/autoware_tools/issues/304>`_)
  * chore: update maintainer
  remove Ryu Yamamoto
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Koji Minoda
  * chore: update maintainer
  remove Shintaro Sakoda
  ---------
* Contributors: Motz

0.3.0 (2025-08-11)
------------------
* style(pre-commit): autofix (`#275 <https://github.com/autowarefoundation/autoware_tools/issues/275>`_)
  * apply pre-commit
  * fix: add maintainer in planning_debug tools package.xml
  ---------
* feat: add tools to collect and check average TPs of PCD maps (`#213 <https://github.com/autowarefoundation/autoware_tools/issues/213>`_)
  * Move all parameters of point cloud divider to launch xml file
  * style(pre-commit): autofix
  * Cleaning things
  * style(pre-commit): autofix
  * Remove unused package
  * Add maintainers from L/M team
  * style(pre-commit): autofix
  * Make TP checker check poses instead of segments
  * style(pre-commit): autofix
  * Move parameters of point cloud divider to xml files
  * style(pre-commit): autofix
  * Fixed issues detected by pre-commit.ci
  * style(pre-commit): autofix
  * Fixed issues detected by pre-commit.ci
  * style(pre-commit): autofix
  * Fixed issues detected by pre-commit.ci
  * Use range query to search for segments when scan topic is not available
  * style(pre-commit): autofix
  * Change range to query_range and radius to avoid shadowing python's built-in variable
  * Mark changed segments passed by the vehicle
  * style(pre-commit): autofix
  * Update README
  * style(pre-commit): autofix
  * Fixing comments
  * style(pre-commit): autofix
  * Remove LICENSE file
  * Add a smooth color band to visualize average TPs and enable checking results to be saved to files
  * style(pre-commit): autofix
  * Fix a bug in tp_collector
  * style(pre-commit): autofix
  * Delete rosbag_filter.py, which was accidentally added
  * Fixed tp checker and collector
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Anh Nguyen, Kyoichi Sugahara

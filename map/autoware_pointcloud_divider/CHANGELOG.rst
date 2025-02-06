^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pointcloud_divider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_pointcloud_divider): fix the image in README.md  (`#125 <https://github.com/autowarefoundation/autoware_tools/issues/125>`_)
  * Since merge_pcd is not used anymore, fix the image in README.md of autoware_pointcloud_divider
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_pointcloud_divider): fix a bug in the customized pcd reader (`#114 <https://github.com/autowarefoundation/autoware_tools/issues/114>`_)
  * Fixed a bug from the customized pcd reader, which makes the reader reads more points than it should do
  * Fixed a spell error
  ---------
* feat(map): add pointcloud divider and pointcloud merger (`#100 <https://github.com/autowarefoundation/autoware_tools/issues/100>`_)
  * Added point cloud merger
  * Update README and remove unused svg file
  * style(pre-commit): autofix
  * Fix pre-commit
  * style(pre-commit): autofix
  * Fix spell check
  * style(pre-commit): autofix
  * Fix pre-commit ci
  * Fix pre-commit ci
  * Fix pre-commit ci
  * Add autoware\_ prefix to nodes
  * style(pre-commit): autofix
  * Add autoware\_ prefix to pointcloud merger and divider; refactor code; rearrange files
  * Follow autoware directory structure
  * Move parameters to yaml files, create schema json files, and apply clang format
  * Fix an error while reading PCD file
  * style(pre-commit): autofix
  * Fix pre-commit ci
  * Fix pre-commit-optional
  * style(pre-commit): autofix
  * Fix merger's schema file
  * style(pre-commit): autofix
  * Fixed review comments
  * style(pre-commit): autofix
  * Fixed spell check
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
* Contributors: Anh Nguyen, Yutaka Kondo, awf-autoware-bot[bot]

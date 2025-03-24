^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_data_analyzer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2025-03-24)
------------------

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* feat(autoware_planning_data_analyzer)!: replace tier4_debug_msgs with tier4_internal_debug_msgs (`#199 <https://github.com/autowarefoundation/autoware_tools/issues/199>`_)
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(global_parameter_loader): prefix package and namespace with autoware (`#159 <https://github.com/autowarefoundation/autoware_tools/issues/159>`_)
* refactor(glog_component): prefix package and namespace with autoware (`#158 <https://github.com/autowarefoundation/autoware_tools/issues/158>`_)
* perf(planning_data_analyzer): improve performance of weight grid search (`#116 <https://github.com/autowarefoundation/autoware_tools/issues/116>`_)
  * perf(planning_data_analyzer): improve weight search logic
  * perf: multithread
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_data_analyzer): add new package (`#103 <https://github.com/autowarefoundation/autoware_tools/issues/103>`_)
  * feat(planning_data_analyzer): add new package
  * style(pre-commit): autofix
  * refactor: use map
  * style(pre-commit): autofix
  * fix: print
  * style(pre-commit): autofix
  * refactor: small change
  * feat: weight grid search
  * feat: add time delta
  * style(pre-commit): autofix
  * fix: typo
  * fix: ignore spell check for foxglove json
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo, awf-autoware-bot[bot]

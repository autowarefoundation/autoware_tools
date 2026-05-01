^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_data_analyzer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-05-01)
------------------
* feat(planning_data_analyzer): remove override scene related source and script (`#400 <https://github.com/autowarefoundation/autoware_tools/issues/400>`_)
  * remove override scene related source and script
  * fix pre-commit
  ---------
* docs(planning_data_analyzer): add displacement-based metrics note (`#399 <https://github.com/autowarefoundation/autoware_tools/issues/399>`_)
* feat(planning_data_analyzer): add remaining open-loop EPDMS metrics (`#385 <https://github.com/autowarefoundation/autoware_tools/issues/385>`_)
  * feat(planning_data_analyzer): add open-loop extended comfort metric
  * feat(planning_data_analyzer): add remaining open-loop EPDMS metrics
  * style(planning_data_analyzer): apply pre-commit formatting
  * fix(planning_data_analyzer): address review comments
  * feat(planning_data_analyzer): add human filter and synthetic epdms
  * fix(pre-commit): resolve full hook failures
  * fix(planning_data_analyzer): address follow-up review comments
  * feat: full topic generation of synthetic EPDMS
  * Refine open-loop metric utilities and fixes
  * chore: drop .pre-commit-config.yaml change from PR
  * chore: align .pre-commit-config.yaml with PR base
  * chore: match .pre-commit-config.yaml to PR base commit
  * chore: drop remaining pre-commit diff
  * refactor: use range-based loop in driving direction compliance
  * refactor: nest epdms aggregation structs
  * style: use range-based loops in open loop gt tests
  * refine analyzer readability and docs
  * fix analyzer readme links
  ---------
* chore: sync files (`#311 <https://github.com/autowarefoundation/autoware_tools/issues/311>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(planning_data_analyzer): avoid GCC 13 false-positive stringop-overread in test (`#388 <https://github.com/autowarefoundation/autoware_tools/issues/388>`_)
  Use push_back instead of initializer list construction to prevent GCC 13
  from emitting a spurious -Wstringop-overread warning during deep inlining
  of vector copy-assignment. The initializer list code path triggers a false
  positive even when assigned to a named variable first.
* feat(planning_data_analyzer): add open-loop lane keeping metric (`#383 <https://github.com/autowarefoundation/autoware_tools/issues/383>`_)
  * feat(planning_data_analyzer): add open-loop lane keeping metric
  * fix(planning_data_analyzer): address lane keeping review feedback
* feat(planning_data_analyzer): add open-loop drivable area compliance metric (`#381 <https://github.com/autowarefoundation/autoware_tools/issues/381>`_)
  * feat(planning_data_analyzer): add open-loop drivable area compliance metric
  * refactor(planning_data_analyzer): improve dac drivable area evaluation
  * refactor(planning_data_analyzer): align dac lanelet scope
* feat(planning_data_analyzer): add open-loop history comfort metric (`#379 <https://github.com/autowarefoundation/autoware_tools/issues/379>`_)
  * feat(planning_data_analyzer): add open-loop history comfort metric
  * chore(planning_data_analyzer): drop local implementation note from PR
  * refactor(planning_data_analyzer): extract history comfort metric utility
  * style(planning_data_analyzer): apply clang-format to history comfort utility
  * fix(planning_data_analyzer): address history comfort review comments
* feat(planning_data_analyzer): add open-loop AHE and FHE metrics (`#377 <https://github.com/autowarefoundation/autoware_tools/issues/377>`_)
  * feat(planning_data_analyzer): add open-loop AHE and FHE metrics
  * fix(planning_data_analyzer): use normalize_radian and separate AHE/FHE outputs
  ---------
* fix(planning_data_analyzer): add missing any include (`#373 <https://github.com/autowarefoundation/autoware_tools/issues/373>`_)
* chore(autoware_planning_data_analyzer): refactor json format (`#372 <https://github.com/autowarefoundation/autoware_tools/issues/372>`_)
  * refactor json formate
  * pre-commit
  * remove redundant log and move func to header
  ---------
* feat: add namespaced open-loop metric outputs for dual topic evaluation (`#371 <https://github.com/autowarefoundation/autoware_tools/issues/371>`_)
  Co-authored-by: Kem (TiankuiXian) <1041084556@qq.com>
* feat(planning_data_analyzer): dlr style-format change on ade and fde (`#368 <https://github.com/autowarefoundation/autoware_tools/issues/368>`_)
  * fix: restore output rosbag filename
  * feat: dlr-style-format modification on ade/fde
  * refactor: add top-level result, backup_bag_path, and  header improvement
  * chore:add a unit test covering the new DLR horizon-key output contract
  ---------
* feat(planning_data_analyzer): add topic to original bag (`#367 <https://github.com/autowarefoundation/autoware_tools/issues/367>`_)
  * add topic to original bag
  * rename ground truth trajectory
  ---------
* feat(planning_data_analyzer): specify output path for json file (`#366 <https://github.com/autowarefoundation/autoware_tools/issues/366>`_)
  * specify output path for json file
  * change file name for json
  ---------
* feat: open loop gt source mode for sliced scenarios(rosbag) with extended gt trajectory (`#364 <https://github.com/autowarefoundation/autoware_tools/issues/364>`_)
  * chore(static_centerline_generator): remove unnecessary include (`#355 <https://github.com/autowarefoundation/autoware_tools/issues/355>`_)
  * planning_data_analyzer: add gt trajectory mode and align output topics
  * fix(static_centerline_generator): update initialization of path_generator (`#345 <https://github.com/autowarefoundation/autoware_tools/issues/345>`_)
  fix path generator initialization
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  * planning_data_analyzer: support gt_source_mode and gt trajectory output
  * fix(autoware_static_centerline_generator): Commenting out flaky test (`#365 <https://github.com/autowarefoundation/autoware_tools/issues/365>`_)
  commenting out flaky test
  * pre-commit
  * refactor:range base, midpoint, and applied less strict failure logic
  * test: align gt trajectory missing GT behavior
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: Mitsuhiro Sakamoto <50359861+mitukou1109@users.noreply.github.com>
  Co-authored-by: Arjun Jagdish Ram <arjun.ram@tier4.jp>
  Co-authored-by: Go Sakayori <go.sakayori@tier4.jp>
  Co-authored-by: Kem (TiankuiXian) <1041084556@qq.com>
* chore(planning_data_analyzer): add maintainer (`#356 <https://github.com/autowarefoundation/autoware_tools/issues/356>`_)
  update maintainer
* Contributors: Go Sakayori, Kem (TiankuiXian), Mete Fatih Cırıt, awf-autoware-bot[bot], beomseok-kimm

0.6.0 (2026-02-14)
------------------
* feat: add Jazzy support for rosbag APIs (`#347 <https://github.com/autowarefoundation/autoware_tools/issues/347>`_)
* Contributors: Ryohsuke Mitsudome

0.5.0 (2025-12-30)
------------------
* chore(planning_data_analyzer): add output dir in launch (`#327 <https://github.com/autowarefoundation/autoware_tools/issues/327>`_)
  * add output dir in launch
  * change trajectory topic in yaml
  ---------
* feat(planning_data_analyzer): update function to evaluate open loop simulation (`#325 <https://github.com/autowarefoundation/autoware_tools/issues/325>`_)
  * update package based on new planning framework repositry
  * remove unnecessary files
  * fix dependency
  * fix pre-commit
  * remove unneccesary files
  * fix file names
  * fix pre-commit for utils
  * fix bag file path
  * fix pre-commit for utils test
  * add copyright for python scripts
  ---------
* chore(autoware_planning_data_analyzer): update maintainer (`#319 <https://github.com/autowarefoundation/autoware_tools/issues/319>`_)
  change and add maintainer
* Contributors: Go Sakayori

0.4.0 (2025-11-16)
------------------

0.3.0 (2025-08-11)
------------------
* chore: sync files (`#214 <https://github.com/autowarefoundation/autoware_tools/issues/214>`_)
* Contributors: awf-autoware-bot[bot]

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

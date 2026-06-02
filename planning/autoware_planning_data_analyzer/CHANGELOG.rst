^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_data_analyzer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat(planning_data_analyzer): filtering the `lateral_deviation_centerline` by `intersection_area` (`#440 <https://github.com/autowarefoundation/autoware_tools/issues/440>`_)
  * tmp
  * fix
  * fix readme
  * addressed comments
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* docs(planning_data_analyzer): add EPDMS metric equations (`#438 <https://github.com/autowarefoundation/autoware_tools/issues/438>`_)
  * docs(planning_data_analyzer): add EPDMS metric equations
  * docs(planning_data_analyzer): address sakayori's review for EPDMS doc
  - Link epdms_metrics.md in README
  - Refine velocity notation to vector (v_t -> \vec{v}_t)
  - Rename 'hard admissibility gate' to 'multiplicative penalty gate'
  - Clarify tiered penalties (0.5) for NC and DDC
  - Correct explanation of single-proposal EP simplification
  * refactor(planning_data_analyzer): added clickable functions on the epdms metric md file
  * style(pre-commit): autofix
  * refactor: Added parameter table
  * style(pre-commit): autofix
  * refactor: refined explanation
  * style(pre-commit): autofix
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_data_analyzer): migrate EP metric (`#436 <https://github.com/autowarefoundation/autoware_tools/issues/436>`_)
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): update EPDMS aggregation filter (`#437 <https://github.com/autowarefoundation/autoware_tools/issues/437>`_)
  * feat(planning_data_analyzer): update EPDMS aggregation filter
  * feat(planning_data_analyzer): exclude EC from human filter
  * fix(planning_data_analyzer): drop dead human-filter EC state
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): migrate EC metric (`#435 <https://github.com/autowarefoundation/autoware_tools/issues/435>`_)
  * feat(planning_data_analyzer): migrate EC metric
  * fix(planning_data_analyzer): address EC review comments
  * fix(extended_comfort): address follow-up style and library nits
  - Narrow rclcpp includes to time and duration
  - Drop redundant get_time_seconds helper in favor of rclcpp::Duration::seconds()
  - Use [] instead of .at() for indexing in bounded loops
  - Move lateral acceleration design note to function-level comment
  * style(pre-commit): autofix
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_data_analyzer): migrate HC metric (`#434 <https://github.com/autowarefoundation/autoware_tools/issues/434>`_)
  * feat(planning_data_analyzer): migrate HC metric
  * refactor(planning_data_analyzer): name HC motion history padding
  * test(planning_data_analyzer): align HC tests with acceleration signals
  * fix(planning_data_analyzer): preserve GT acceleration for HC
  * fix(history_comfort): address sakayori's PR comments
  - Update availability and score when motion history is missing
  - Fix past-horizon loop boundary to include t=-dt sample
  - Add comments for frame assumption and lateral acceleration handling
  - Optimize message lookup with std::lower_bound
  - Use named constant for closest sample tolerance factor
  * style(pre-commit): autofix
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_data_analyzer): migrate LK metric (`#432 <https://github.com/autowarefoundation/autoware_tools/issues/432>`_)
  * feat(planning_data_analyzer): migrate LK metric
  * fix(planning_data_analyzer): address LK review comments
  * fix(planning_data_analyzer): address LK CI and review nits
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): migrate TTC metric (`#431 <https://github.com/autowarefoundation/autoware_tools/issues/431>`_)
  * feat(planning_data_analyzer): migrate TTC metric
  * test(planning_data_analyzer): cover TTC bad-area behind guard
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): migrate TLC metric (`#429 <https://github.com/autowarefoundation/autoware_tools/issues/429>`_)
  * feat(planning_data_analyzer): migrate TLC metric
  * refactor(planning_data_analyzer): remove unused TLC debug payload
  * fix(planning_data_analyzer): address TLC review comments
  * test(planning_data_analyzer): fix TLC stop-line fixture
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): migrate DDC metric (`#428 <https://github.com/autowarefoundation/autoware_tools/issues/428>`_)
  * feat(planning_data_analyzer): use local context for DDC
  * fix(planning_data_analyzer): remove unused DDC window fields
  * fix(planning_data_analyzer): drop unused DDC include
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): add NAVSIM-style DAC metric (`#427 <https://github.com/autowarefoundation/autoware_tools/issues/427>`_)
  * feat(planning_data_analyzer): migrate DAC to semantic drivable area
  * test(planning_data_analyzer): suppress DAC map conversion deprecation
  * fix(planning_data_analyzer): address DAC review comments
  * fix(planning_data_analyzer): include geometry point helper
  * fix(planning_data_analyzer): use lanelet conversion namespace
  * fix(planning_data_analyzer): avoid lanelet conversion helper
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): migrate NC metric (`#426 <https://github.com/autowarefoundation/autoware_tools/issues/426>`_)
  * feat(planning_data_analyzer): use tracked objects for NC
  * feat(planning_data_analyzer): migrate NC to tracked objects
  * refactor(planning_data_analyzer): move future object slicing helper
  * refactor(planning_data_analyzer): avoid duplicate object slicing
  * fix(planning_data_analyzer): address NC review comments
  * refactor(planning_data_analyzer): address NC helper review
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): add EPDMS runtime topic controls (`#425 <https://github.com/autowarefoundation/autoware_tools/issues/425>`_)
  * feat(planning_data_analyzer): add EPDMS runtime topic controls
  * fix(planning_data_analyzer): address EPDMS runtime review
  * fix(planning_data_analyzer): handle all enabled metrics selector
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* refactor(planning_data_analyzer): split EPDMS geometry helpers (`#423 <https://github.com/autowarefoundation/autoware_tools/issues/423>`_)
  * refactor(planning_data_analyzer): split EPDMS geometry helpers
  (cherry picked from commit 5b45980cbc9967370ab72c1ae37081908a0296dc)
  * fix(planning_data_analyzer): align EPDMS helper include guards
  * fix(planning_data_analyzer): satisfy helper split pre-commit hooks
  * refactor(planning_data_analyzer): address EPDMS helper review comments
  * refactor(planning_data_analyzer): deduplicate EPDMS geometry helpers
  * fix(planning_data_analyzer): satisfy geometry helper pre-commit
  * docs(planning_data_analyzer): explain Point2d distance adapter
  * fix(planning_data_analyzer): use non-deprecated lanelet angle API
  * fix(planning_data_analyzer): call experimental lanelet angle API
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
* feat(planning_data_analyzer): aggregate ADE/FDE only for override scenes (`#424 <https://github.com/autowarefoundation/autoware_tools/issues/424>`_)
  * feat(planning_data_analyzer): aggregate ADE/FDE only for override scenes
  Detect AUTONOMOUS->MANUAL transitions from /vehicle/status/control_mode
  and emit per-horizon ADE/FDE/etc. statistics under override_only/* in the
  summary JSON, using only trajectory samples whose timestamp falls inside
  the configurable window after each transition.
  * refactor(planning_data_analyzer): extract override window helpers and add unit tests
  Move compute_override_windows and is_within_any_window out of OpenLoopEvaluator
  into a free-function module under utils/ so the pure logic can be exercised
  without instantiating the evaluator. Add gtest coverage for empty/single-event
  inputs, AUTONOMOUS->MANUAL filtering, multiple transitions, and window boundary
  inclusivity.
  * fix(planning_data_analyzer): use RCL_ROS_TIME for override window endpoints
  Trajectory timestamps are constructed from ROS message stamps (RCL_ROS_TIME),
  but rclcpp::Time(int64_t) defaults to RCL_SYSTEM_TIME. Comparing the two raises
  "can't compare times with different time sources" at runtime once any
  AUTONOMOUS->MANUAL transition is detected. Explicitly pin override window
  endpoints to RCL_ROS_TIME and add a regression test that asserts the clock
  type, plus refresh the boundary-inclusivity test to mirror the production
  clock.
  * fix year
  * fix pre-commit
  * style: apply clang-format
  ---------
* refactor(planning_data_analyzer): organize EPDMS metric files (`#421 <https://github.com/autowarefoundation/autoware_tools/issues/421>`_)
  * refactor(planning_data_analyzer): organize EPDMS metric files
  * style(pre-commit): autofix
  * refactor(planning_data_analyzer): use source-root-relative includes
  * style(pre-commit): autofix
  ---------
  Co-authored-by: beomseok-kimm <beomseok.kim.2@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_data_analyzer): parallelize open-loop evaluation and reduce metric-path overhead (`#401 <https://github.com/autowarefoundation/autoware_tools/issues/401>`_)
  * feat(planning_data_analyzer): parallelize open-loop evaluation
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  (cherry picked from commit b41036676913d2873a845338d3d48407bd7befd7)
  * perf(planning_data_analyzer): reduce repeated metric path work
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  (cherry picked from commit e338d6dff4a4cb918152d2cdacd3f2661de4d455)
  * perf(autoware_planning_data_analyzer): optimize memory and ensure thread-safety in parallel evaluation
  - Pre-allocate member vectors and write directly using indexed access to avoid large temporary results.
  - Prime RouteHandler to build internal caches/indices single-threaded before parallel access.
  - Address reviewer feedback regarding memory overhead and concurrency safety.
  ---------
  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
* Contributors: Beomseok Kim, Go Sakayori, Kazunori-Nakajima, beomseok-kimm

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

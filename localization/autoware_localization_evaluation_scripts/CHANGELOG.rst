^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_localization_evaluation_scripts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2025-11-16)
------------------
* feat(autoware_localization_evaluation_scripts): add new diagnostics to check and refactor diagnostics_flag_check (`#310 <https://github.com/autowarefoundation/autoware_tools/issues/310>`_)
  * fix process when there is no DiagnosticsFlagCheck in scenario
  * fix typo
  * add diagnostics flag check_is_initial_pose_reliable
  * refactor script
  * add debug print
  * add nvtl and localization_error_monitor diags
  * make time expression richer
  * make sec and nanosec explicilty int
  * fix large int casting
  * add pose_instability
  * Update document
  * make the code clean
  * fix spelling error
  * remove unnecessary print
  ---------
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
* fix(autoware_localization_evaluation_scripts): fix diagnostics flag check expection handling (`#303 <https://github.com/autowarefoundation/autoware_tools/issues/303>`_)
  * fix process when there is no DiagnosticsFlagCheck in scenario
  * fix typo
  ---------
* feat(autoware_localization_evaluation_scripts): detect rise/fall flags of localization diagnostics (`#298 <https://github.com/autowarefoundation/autoware_tools/issues/298>`_)
  * Enable to read diagnostics flags
  * make the constructor of OverallCriteriaMask robust to unknown conditions
  * make input of diagnostics_flag_check simple and make large int value inputs robust
  * add document for diagnostics_flag_check
  * fix cspell, add explanation
  * fix docs mistake
  ---------
* feat(autoware_localization_evaluation_scripts): create masks for each criterion (`#297 <https://github.com/autowarefoundation/autoware_tools/issues/297>`_)
  * add mask to evaluate each criterion
  * minor fix
  * remove print debugs
  * fix spelling mistakes
  ---------
* Contributors: Motz, Taiki Yamada

0.3.0 (2025-08-11)
------------------
* fix: change the criteria for localization  (`#267 <https://github.com/autowarefoundation/autoware_tools/issues/267>`_)
  * Added the ndt filter
  * Fixed the threshold from 1% to 5%
  * Removed the redundant line
  ---------
* feat: add acceleration evaluation (`#265 <https://github.com/autowarefoundation/autoware_tools/issues/265>`_)
  * Added acceleration evaluation
  * style(pre-commit): autofix
  * Added a blank
  * Fixed comments
  * Refined calc_acceleration_diff.py
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: change the maintainer of `autoware_localization_evaluation_scripts` (`#266 <https://github.com/autowarefoundation/autoware_tools/issues/266>`_)
  * Changed the maintainer of `autoware_localization_evaluation_scripts`
  * Fix
  ---------
* fix: to apply to a pose only tsv file (`#261 <https://github.com/autowarefoundation/autoware_tools/issues/261>`_)
  Fixed to apply to a pose only tsv file
* feat: add `summary.json` (`#260 <https://github.com/autowarefoundation/autoware_tools/issues/260>`_)
  * Added `summary.json`
  * Added velocities
  * Added mean\_*_velocity_norm
  * Added diagnostics
  * style(pre-commit): autofix
  * Added `cspell:ignore rotvec`
  * Add a guard
  * style(pre-commit): autofix
  * Fixed "E266 too many leading '#' for block comment"
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_localization_evaluation_scripts): add parse accel (`#237 <https://github.com/autowarefoundation/autoware_tools/issues/237>`_)
  * Added parsing accel
  * Added extraction
  * Fixed the script name
  ---------
* Contributors: SakodaShintaro

0.2.0 (2025-03-24)
------------------
* Added plt.close() (`#221 <https://github.com/autowarefoundation/autoware_tools/issues/221>`_)
* fix: set save dir (`#220 <https://github.com/autowarefoundation/autoware_tools/issues/220>`_)
  * fix: set save dir
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_localization_evaluation_scripts): change target directories (`#219 <https://github.com/autowarefoundation/autoware_tools/issues/219>`_)
  * Removed a japanese comment
  * Fixed target directories
  ---------
* feat: add `analyze_rosbags_parallel.py` (`#217 <https://github.com/autowarefoundation/autoware_tools/issues/217>`_)
  * Fixed storage_id
  * Added analyze_rosbags_parallel.sh
  * style(pre-commit): autofix
  * Rewrote in python
  * style(pre-commit): autofix
  * Fixed README.md
  * Create a main function of plot_diagnostics.py
  * style(pre-commit): autofix
  * Created a main function of extract_pose_from_rosbag.py
  * Created a main function of compare_trajectories.py
  * Removed unused variables
  * Fixed topic_reference
  * Updated README.md
  * style(pre-commit): autofix
  * Updated README.md
  * Added "and not d.is_symlink()"
  * style(pre-commit): autofix
  * Removed a storage option
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `compare_trajectories.py` and `extract_pose_from_rosbag.py` (`#216 <https://github.com/autowarefoundation/autoware_tools/issues/216>`_)
  * Added compare_trajectories.py and extract_pose_from_rosbag.py
  * style(pre-commit): autofix
  * Rewrite as a literal
  * Added "cspell: ignore rotvec"
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `autoware_localization_evaluation_scripts` (`#215 <https://github.com/autowarefoundation/autoware_tools/issues/215>`_)
  * Added autoware_localization_evaluation_scripts
  * Added a linebreak
  * style(pre-commit): autofix
  * Added a storage option
  * Added options in README.md
  * Added ylabel("gyro_bias [rad/s]")
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Hayato Mizushima, SakodaShintaro

* Added plt.close() (`#221 <https://github.com/autowarefoundation/autoware_tools/issues/221>`_)
* fix: set save dir (`#220 <https://github.com/autowarefoundation/autoware_tools/issues/220>`_)
  * fix: set save dir
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_localization_evaluation_scripts): change target directories (`#219 <https://github.com/autowarefoundation/autoware_tools/issues/219>`_)
  * Removed a japanese comment
  * Fixed target directories
  ---------
* feat: add `analyze_rosbags_parallel.py` (`#217 <https://github.com/autowarefoundation/autoware_tools/issues/217>`_)
  * Fixed storage_id
  * Added analyze_rosbags_parallel.sh
  * style(pre-commit): autofix
  * Rewrote in python
  * style(pre-commit): autofix
  * Fixed README.md
  * Create a main function of plot_diagnostics.py
  * style(pre-commit): autofix
  * Created a main function of extract_pose_from_rosbag.py
  * Created a main function of compare_trajectories.py
  * Removed unused variables
  * Fixed topic_reference
  * Updated README.md
  * style(pre-commit): autofix
  * Updated README.md
  * Added "and not d.is_symlink()"
  * style(pre-commit): autofix
  * Removed a storage option
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `compare_trajectories.py` and `extract_pose_from_rosbag.py` (`#216 <https://github.com/autowarefoundation/autoware_tools/issues/216>`_)
  * Added compare_trajectories.py and extract_pose_from_rosbag.py
  * style(pre-commit): autofix
  * Rewrite as a literal
  * Added "cspell: ignore rotvec"
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `autoware_localization_evaluation_scripts` (`#215 <https://github.com/autowarefoundation/autoware_tools/issues/215>`_)
  * Added autoware_localization_evaluation_scripts
  * Added a linebreak
  * style(pre-commit): autofix
  * Added a storage option
  * Added options in README.md
  * Added ylabel("gyro_bias [rad/s]")
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Hayato Mizushima, SakodaShintaro

0.1.0 (2025-01-29)
------------------

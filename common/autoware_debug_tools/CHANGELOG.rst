^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_debug_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2025-12-30)
------------------

0.4.0 (2025-11-16)
------------------
* chore(autoware_debug_tools): ignore log in frequent_log_checker (`#276 <https://github.com/autowarefoundation/autoware_tools/issues/276>`_)
* Contributors: Takayuki Murooka

0.3.0 (2025-08-11)
------------------
* style(pre-commit): autofix (`#275 <https://github.com/autowarefoundation/autoware_tools/issues/275>`_)
  * apply pre-commit
  * fix: add maintainer in planning_debug tools package.xml
  ---------
* fix(frequent_log_checker): print the largest number of each frequent log (`#228 <https://github.com/autowarefoundation/autoware_tools/issues/228>`_)
* Contributors: Kyoichi Sugahara, Takayuki Murooka

0.2.0 (2025-03-24)
------------------
* fix(autoware_debug_tools): time keeper does not work with the old message type (`#224 <https://github.com/autowarefoundation/autoware_tools/issues/224>`_)
  * fix(autoware_debug_tools): TimeKeeper does not work with the old message type
  * fix typo
  ---------
* feat(autoware_debug_tools): add frequent_log_checker.py (`#172 <https://github.com/autowarefoundation/autoware_tools/issues/172>`_)
  * feat(autoware_debug_tools): add frequent_log_checker.py
  * minor change
  * minor fix
  ---------
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
* Contributors: Takayuki Murooka

0.1.0 (2025-01-28)
------------------
* feat: use autoware_internal_debug_msgs in processing_time_plotter.py (`#184 <https://github.com/autowarefoundation/autoware_tools/issues/184>`_)
  * feat: use autoware_internal_debug_msgs in processing_time_plotter.py
  * fix typo: argment -> argument
  * fix typo: LINESTYLES -> LINE_STYLES
  ---------
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_debug_tools): add rosout_log_reconstructor.py (`#160 <https://github.com/autowarefoundation/autoware_tools/issues/160>`_)
  * feat(autoware_debug_tools): add rosout_log_reconstructor.py
  * update README.md
  ---------
* fix: to plot "exe_time_ms" of ndt_scan_matcher (`#161 <https://github.com/autowarefoundation/autoware_tools/issues/161>`_)
  * Fixed to plot "exe_time_ms" of ndt_scan_matcher
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Added maintainers (`#162 <https://github.com/autowarefoundation/autoware_tools/issues/162>`_)
* feat(autoware_debug_tools): add topic connection checker (`#132 <https://github.com/autowarefoundation/autoware_tools/issues/132>`_)
  * feat: add topic connection checker, first commit as a proposal
  * style(pre-commit): autofix
  * implement diagnostic based topic settings; implement the topic localization
  * temp
  * refactor the codes
  * add readme for topic connection checker
  * test with debug
  * style(pre-commit): autofix
  * fix flake8
  * ignore rviz2 node, ignore hyper-linked files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(system_usage_monitor): change the node search method (`#137 <https://github.com/autowarefoundation/autoware_tools/issues/137>`_)
  * Fixed the node search method
  * Added a comment
  ---------
* feat: add "skip_plt_show" arg to plotter (`#136 <https://github.com/autowarefoundation/autoware_tools/issues/136>`_)
  * Added "skip_plt_show" arg to plotter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: enable to choose topic with argument (`#95 <https://github.com/autowarefoundation/autoware_tools/issues/95>`_)
* feat(autoware_debug_tools): processing time total processing tree (`#92 <https://github.com/autowarefoundation/autoware_tools/issues/92>`_)
  * feat: add total processing time tree to processing time visualizer
  * feat: add percentage of processing time
  * feat: add rest of the measured timekeepers
  * style(pre-commit): autofix
  * feat:  print average first and the worst case next
  * style(pre-commit): autofix
  * feat: refactor processing time tree sum method to summarize_tree
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_debug_tools): add system performance plotter (`#91 <https://github.com/autowarefoundation/autoware_tools/issues/91>`_)
  * feat(autoware_debug_tools): add system performance plotter
  * update README
  * fix README.md
  ---------
* feat(processing_time_visualizer): add summarize option (`#90 <https://github.com/autowarefoundation/autoware_tools/issues/90>`_)
  * add summarize feature
  * add summarize
  * style(pre-commit): autofix
  * fix typo
  * make summarize option dynamic
  * apply suggestion
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* fix(autoware_debug_tools): the image of system_usage_monitor was wrong (`#87 <https://github.com/autowarefoundation/autoware_tools/issues/87>`_)
* feat(autoware_dubug_tools): add function to report worst time when r is pressed at the end (`#86 <https://github.com/autowarefoundation/autoware_tools/issues/86>`_)
* feat(autoware_debug_tools): add system_usage_monitor.py (`#85 <https://github.com/autowarefoundation/autoware_tools/issues/85>`_)
  * feat(autoware_debug_tools): add system_usage_monitor.py
  * publish system usage
  ---------
* feat(autoware_debug_tools): add processing time visualizer (`#75 <https://github.com/autowarefoundation/autoware_tools/issues/75>`_)
  * feat(autoware_debug_tools): add processing time visualizer
  * ignore spell check
  * update everythings
  ---------
* Contributors: Masaki Baba, SakodaShintaro, Taekjin LEE, Takayuki Murooka, Yoshi Ri, Yukinari Hisaki, Yuxuan Liu, awf-autoware-bot[bot]

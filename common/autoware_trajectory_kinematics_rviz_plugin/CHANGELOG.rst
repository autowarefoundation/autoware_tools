Changelog for package autoware_trajectory_kinematics_rviz_plugin
================================================================

0.46.0 (2026-04-08)
-------------------
* RViz panel for trajectory / scored-candidate kinematics plotting (Qt Charts).
* Review follow-ups: coalesced panel refresh, tooltip throttling, monotonic nearest-point search, chart teardown order, tests, packaging metadata.
* Second-round: `package.xml` Qt deps trimmed; subscription callbacks use `QPointer`; list signature includes labels; topic poll pauses when hidden; cached UI font; `const` default palette; chart teardown RAII; misc panel hygiene.

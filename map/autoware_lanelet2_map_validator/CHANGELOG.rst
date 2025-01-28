^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_map_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-01-28)
------------------
* Initial release
* Containing the following changes
  * feat(autoware_lanelet2_map_validator): introduce autoware_lanelet2_map_validator (`#118 <https://github.com/autowarefoundation/autoware_tools/pull/118>`_)
  * chore(autoware_lanelet2_map_validator): add maintainers (`#141 <https://github.com/autowarefoundation/autoware_tools/pull/141>`_)
  * chore(autoware_lanelet2_map_validator): add requirement vm-02-02 to autoware_requirement_set (`#143 <https://github.com/autowarefoundation/autoware_tools/pull/143>`_)
  * refactor(lanelet2_map_validator): move headers to include/ (`#144 <https://github.com/autowarefoundation/autoware_tools/pull/144>`_)
  * feat(autoware_lanelet2_map_validator): allow prerequisites attribute for input (`#147 <https://github.com/autowarefoundation/autoware_tools/pull/147>`_)
  * Removed redundant appendIssues (`#148 <https://github.com/autowarefoundation/autoware_tools/pull/148>`_)
  * fix(lanelet2_map_validator): change validation order in regulatory_elements_details (`#151 <https://github.com/autowarefoundation/autoware_tools/pull/151>`_)
  * refactor(lanelet2_map_validator): move custom implementation to lanelet::autoware::validation (`#152 <https://github.com/autowarefoundation/autoware_tools/pull/152>`_)
  * refactor(lalenet2_map_validator): divide map loading process (`#153 <https://github.com/autowarefoundation/autoware_tools/pull/153>`_)
  * feat(lanelet2_map_validator): add test codes for existing validators (`#150 <https://github.com/autowarefoundation/autoware_tools/pull/150>`_)
  * feat(lanelet2_map_validator): added issue codes (`#163 <https://github.com/autowarefoundation/autoware_tools/pull/163>`_)
  * feat(lanelet2_map_validator): add validator to check traffic light facing (`#165 <https://github.com/autowarefoundation/autoware_tools/pull/165>`_)
  * Fixed issue that invalid prerequisites are not reflected to the results (`#169 <https://github.com/autowarefoundation/autoware_tools/pull/169>`_)
  * docs(lanelet2_map_validator): add a new document how_to_contribute.md (`#170 <https://github.com/autowarefoundation/autoware_tools/pull/170>`_)
  * feat(lanelet2_map_validator): check whether intersection_area satisfies vm-03-08 (`#171 <https://github.com/autowarefoundation/autoware_tools/pull/171>`_)
  * chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/pull/11>`_)
  * feat(autoware_lanelet_map_validator): add dangling reference checker to non existing intersection_area (`#177 <https://github.com/autowarefoundation/autoware_tools/pull/177>`_)
  * chore(lanelet2_map_validator): automate test code compilation and categorize test codes (`#183 <https://github.com/autowarefoundation/autoware_tools/pull/183>`_)
  * feat(lanelet2_map_validator): generation script for new validators (`#180 <https://github.com/autowarefoundation/autoware_tools/pull/180>`_)
  * docs(autoware_lanelet2_map_validator): update usage (`#191 <https://github.com/autowarefoundation/autoware_tools/pull/191>`_)
  * fix(lanelet2_map_validator): restore missing intersection lane and removed unnecessary linestrings from intersection test maps (`#188 <https://github.com/autowarefoundation/autoware_tools/pull/188>`_)
  * docs(lanelet2_map_validator): update README (`#193 <https://github.com/autowarefoundation/autoware_tools/pull/193>`_)
  * feat(lanelet2_map_validator): add validator to check whether intersection lanelets have valid turn_direction tags (`#186 <https://github.com/autowarefoundation/autoware_tools/pull/186>`_)
  * feat(lanelet2_map_validator): check local coordinates declaration (#194) (`#194 <https://github.com/autowarefoundation/autoware_tools/pull/194>`_)

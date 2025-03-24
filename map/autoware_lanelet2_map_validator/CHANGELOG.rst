^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_map_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2025-03-24)
------------------

0.1.0 (2025-01-28)
------------------
* unify version to 0.0.0
* feat(lanelet2_map_validator): check local coordinates declaration (`#194 <https://github.com/autowarefoundation/autoware_tools/issues/194>`_)
  * Implement mapping.lane.local_coordinates_declaration
  * Added test for mapping.lane.local_coordinates_declaration
  * Added documents for mapping.lane.local_coordinates_declaration
  ---------
* feat(lanelet2_map_validator): add validator to check whether intersection lanelets have valid turn_direction tags (`#186 <https://github.com/autowarefoundation/autoware_tools/issues/186>`_)
  * Added validator which checks the "turn_direction" tag in intersection_areas
  * Added test code for mapping.intersection.turn_direction_tagging
  * Added document for mapping.intersection.turn_direction_tagging
  * Added mapping.intersection.turn_direction_tagging to autoware_requirement_set.json
  * Fixed spelling errors
  * Removed crosswalk boundaries from test map
  ---------
* docs(lanelet2_map_validator): update README (`#193 <https://github.com/autowarefoundation/autoware_tools/issues/193>`_)
  * Updated document
  * Added explanation about issue_code
  * Removed back ticks in the title
  * Fixed spelling issues
  ---------
* fix(lanelet2_map_validator): restore missing intersection lane and removed unnecessary linestrings from intersection test maps (`#188 <https://github.com/autowarefoundation/autoware_tools/issues/188>`_)
  * Restore missing linestring 197 and lanelet 49
  * Removed crosswalk remainings from intersection category test maps
  ---------
* docs(autoware_lanelet2_map_validator): update usage (`#191 <https://github.com/autowarefoundation/autoware_tools/issues/191>`_)
* feat(lanelet2_map_validator): generation script for new validators (`#180 <https://github.com/autowarefoundation/autoware_tools/issues/180>`_)
  * temporary commit
  * Added python script
  * Finished except docs
  * Added documents related stuff
  * Moved stuff to templates
  * Revised files to suit the current directory
  * Added arguments
  * Added prints
  * added #include <string> to test code
  * Fixed typo
  * Write explanation of create_new_validator.py
  * Added back slashes to example command
  * Enable the generation script to be run by `ros2 run`
  ---------
* chore(lanelet2_map_validator): automate test code compilation and categorize test codes (`#183 <https://github.com/autowarefoundation/autoware_tools/issues/183>`_)
  * Categorize test codes
  * Rewrite CMakeLists.txt so that contributors doesn't have to write the test code name in it
  ---------
* feat(autoware_lanelet_map_validator): add dangling reference checker to non existing intersection_area (`#177 <https://github.com/autowarefoundation/autoware_tools/issues/177>`_)
* chore: sync files (`#11 <https://github.com/autowarefoundation/autoware_tools/issues/11>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lanelet2_map_validator): check whether intersection_area satisfies vm-03-08 (`#171 <https://github.com/autowarefoundation/autoware_tools/issues/171>`_)
  * Create the framework for intersection_area_validity.
  * Made is_valid checker in intersection_area_validity
  * Split and create a new validator intersection_area_segement_type.
  * Completed intersection_area_segment_type
  * Added `vm-03-08` to autoware_requirement_set.json
  * Added `vm-03-08` to autoware_requirement_set.json
  * Added documents for intersection_area validators
  * Added `intersection_area` type polygons to sample_map.osm
  * Added test codes
  * Fixed spelling error
  * Removed original bbox calculation and use the one in the Lanelet2 library
  * Added explanation of functions
  ---------
* docs(lanelet2_map_validator): add a new document how_to_contribute.md (`#170 <https://github.com/autowarefoundation/autoware_tools/issues/170>`_)
  * Added a document how_to_contribute.md to lanelet2_map_validator
  * Added information about CMakeLists in tests.
  * Added figure illustrating the input output
  * Add a link to how_to_contribute.md to README.md
  * Fixed spelling and grammar mistakes
  * Fixed some sentences
  * Fixed spelling errors
  * Fixed link URLs
  * Quit using .. to direct to README.md
  * Fixed link mistakes
  ---------
* Fixed issue that invalid prerequisites are not reflected to the results (`#169 <https://github.com/autowarefoundation/autoware_tools/issues/169>`_)
* feat(lanelet2_map_validator): add validator to check traffic light facing (`#165 <https://github.com/autowarefoundation/autoware_tools/issues/165>`_)
  * Added valdiator missing_referrers_for_traffic_lights
  * Added validator traffic_light_facing
  * Added traffic_light_facing and missing_referrers_for_traffic_lights
  * Added new validators to README.md
  * Added test codes for traffic_light_facing and missing_referrers_for_traffic_lights
  * feat(lanelet2_map_validator): added issue codes  (`#163 <https://github.com/autowarefoundation/autoware_tools/issues/163>`_)
  * Added issue code processing
  * Revised tests for json processing
  * Added issue codes for missing_regulatory_elements_for_crosswalks
  * Added issue codes for regulatory_element_details_for_crosswalks
  * Added issue codes for missing_regulatory_elements_for_stop_lines
  * Added issue codes for missing_regulatory_elements_for_traffic_lights
  * Added issue codes for regulatory_element_details_for_traffic_lights
  * Added issue codes to docs
  * Change issue_code_prefix to append_issue_code_prefix
  * Fixed merging mistake
  ---------
  * Changed to append_issue_code_prefix.
  Added test for TrafficLight.CorrectFacing-001
  * Rearrange code structure
  * Fixed spelling mistakes
  * Fixed traffic_light_facing_procedure.svg
  * Fixed mistakes in document
  * Refine algorithm in traffic_light_facing.cpp
  * Shorten the data collection process, and updated the procedure svg
  * Use findUsages in missing_referrers_for_traffic_lights
  * Update map/autoware_lanelet2_map_validator/docs/traffic_light/missing_referrers_for_traffic_lights.md
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  * style(pre-commit): autofix
  * Quit stocking all lanelets to collect traffic light regulatory elements
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lanelet2_map_validator): added issue codes  (`#163 <https://github.com/autowarefoundation/autoware_tools/issues/163>`_)
  * Added issue code processing
  * Revised tests for json processing
  * Added issue codes for missing_regulatory_elements_for_crosswalks
  * Added issue codes for regulatory_element_details_for_crosswalks
  * Added issue codes for missing_regulatory_elements_for_stop_lines
  * Added issue codes for missing_regulatory_elements_for_traffic_lights
  * Added issue codes for regulatory_element_details_for_traffic_lights
  * Added issue codes to docs
  * Change issue_code_prefix to append_issue_code_prefix
  * Fixed merging mistake
  ---------
* feat(lanelet2_map_validator): add test codes for existing validators (`#150 <https://github.com/autowarefoundation/autoware_tools/issues/150>`_)
  * Added small maps for testing.
  Added test codes using these maps.
  * Rearrange architecture of test directory.
  Added regulatory_elements_details_for_crosswalks test
  * Removed old test programs
  * Removed test_regulatory_elements_details.cpp
  * Revised test (TestRegulatoryElementDetailsForTrafficLights, MissingRefers) to look into the loading errors
  * Added sample_map for testing a normal map
  * Reflect PR comments
  * Fixed detection area in sample_map.osm
  * Added autoware namespace to test codes
  ---------
* refactor(lalenet2_map_validator): divide map loading process (`#153 <https://github.com/autowarefoundation/autoware_tools/issues/153>`_)
* refactor(lanelet2_map_validator): move custom implementation to lanelet::autoware::validation (`#152 <https://github.com/autowarefoundation/autoware_tools/issues/152>`_)
* fix(lanelet2_map_validator): change validation order in regulatory_elements_details (`#151 <https://github.com/autowarefoundation/autoware_tools/issues/151>`_)
  * Changed the order to validate in regulatory_element_details
  * Revised test code
  ---------
* Removed redundant appendIssues (`#148 <https://github.com/autowarefoundation/autoware_tools/issues/148>`_)
* feat(autoware_lanelet2_map_validator): allow prerequisites attribute for input (`#147 <https://github.com/autowarefoundation/autoware_tools/issues/147>`_)
  * Added prerequisites tag to input.
  Moved process_requirements to validation.cpp
  * Added prerequisites to autoware_requirement_set.json
  * Redefine ValidatorInfo
  * Changed check_prerequisite_completion not to read the json_data
  * Added two tests CreateValidationQueueNoCycles CreateValidationQueueWithCycles
  * Added test CheckPrerequisiteCompletionSuccess CheckPrerequisiteCompletionFailure
  * Change how to load json files in tests.
  * Added test DescriptUnusedValidatorsToJson and SummarizeValidatorResults
  * Revised README.md to the current status
  * Fixed typo and unknown words
  * Reflect PR comments
  * Fixed typo
  ---------
* refactor(lanelet2_map_validator): move headers to include/ (`#144 <https://github.com/autowarefoundation/autoware_tools/issues/144>`_)
* chore(autoware_lanelet2_map_validator): add requirement vm-02-02 to autoware_requirement_set (`#143 <https://github.com/autowarefoundation/autoware_tools/issues/143>`_)
  * Add Sobue-san as maintainer of autoware_lanelet2_map_validator
  * Added maintainers to autoware_lanelet2_map_validator
  * Added vm-02-02 to autoware_requirement_set.json
  * Fixed error of autoware_lanelet2_map_validator template
  * Detect stop lines that are referred as `refers` role.
  ---------
* chore(autoware_lanelet2_map_validator): add maintainers (`#141 <https://github.com/autowarefoundation/autoware_tools/issues/141>`_)
  * Add Sobue-san as maintainer of autoware_lanelet2_map_validator
  * Added maintainers to autoware_lanelet2_map_validator
  ---------
* feat(autoware_lanelet2_map_validator): introduce autoware_lanelet2_map_validator (`#118 <https://github.com/autowarefoundation/autoware_tools/issues/118>`_)
  * introduce autoware_lanelet2_map_validator to autoware_tools
  * wrote description a little to README.md
  * style(pre-commit): autofix
  * Restore commented out parts.
  Removed rclcpp which is unused.
  * style(pre-commit): autofix
  * Separate validation rules to samller pieces.
  Added validation template
  * Split the validation code into smaller pieces.
  Added yaml input/output for a set of validations
  * Fixed test codes to use the separated codes
  * Removed unused code which are already divided to smaller codes.
  * Rename new_main.cpp to main.cpp
  * style(pre-commit): autofix
  * Wrote detailed README.md
  * Fixed commit mistake
  * Renew input command option to `-i` from `-r`.
  Fixed mistakes in README.md
  * style(pre-commit): autofix
  * Fixed long to uint64_t
  * Fixed spelling
  * style(pre-commit): autofix
  * Fixed typo
  * Split long lines in the code
  * style(pre-commit): autofix
  * Changed the entire structure.
  Fixed pre-commit.ci related errors.
  * style(pre-commit): autofix
  * Fixed pre-commit.ci related stuff
  * Write more details about the relationship to lanelet2_validation.
  Rewrite misleading examples.
  * Added figure of the architecture
  * Change the input/output to JSON
  * Revised architecture image of autoware_lanelet2_map_validator
  * fixed typo
  * Renew year numbers
  * Fixed dependency
  * Fixed pointed out issues
  * Improve error handling
  Refactor code style
  * Avoid clang format
  Delete unused variables
  * Removed redundant process.
  Restrict input/output format.
  * Added approaches to the documents
  * Fixed typo
  * Removed catch and improve io error handling
  * Fixed grammatical error.
  Fixed explanation of issues
  * Added stop_line validator to the table in the main README.md
  * Renamed lib to common.
  Refined CMakeLists.txt
  * Removed redundant under score
  * Removed redundant underscore again
  * Changed years.
  Removed redundant else statement.
  Removed debug comments
  * Removed underscore from test_regulatory_element_details.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mamoru Sobue, TaikiYamada4, Yutaka Kondo, awf-autoware-bot[bot]

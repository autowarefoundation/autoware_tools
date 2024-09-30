# missing_regulator_elements_for_stop_lines

## Validator name

mapping.stop_line.missing_regulatory_elements

## Feature

This validator checks whether each `stop_line` type linestring has a relevant regulatory element.
The issue specifies the stop_line "linestring" as the **primitive**, and the lanelet ID will be specified as the **ID**.

| Message                                           | Severity | Description                                                                                 |
| ------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| "No regulatory element refers to this stop line." | Error    | There is a `stop_line` type linestring that hasn't been referred to any regulatory element. |

## Related source codes

- missing_regulatory_elements_for_stop_line.cpp
- missing_regulatory_elements_for_stop_line.hpp

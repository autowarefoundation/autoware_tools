# missing_regulator_elements_for_crosswalk

## Validator name

mapping.crosswalk.missing_regulatory_elements

## Feature

This validator checks whether each `crosswalk` subtype lanelet has a relevant regulatory element.
The issue specifies the crosswalk "lanelet" as the **primitive**, and the lanelet ID will be specified as the **ID**.

| Message                                           | Severity | Description                                                                                 |
| ------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| "No regulatory element refers to this crosswalk." | Error    | There is a `crosswalk` subtype lanelet that hasn't been referred to any regulatory element. |

## Related source codes

- missing_regulatory_elements_for_crosswalk.hpp
- missing_regulatory_elements_for_crosswalk.cpp

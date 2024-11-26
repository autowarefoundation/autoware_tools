# missing_regulator_elements_for_stop_lines

## Validator name

mapping.stop_line.missing_regulatory_elements

## Feature

This validator checks whether each `stop_line` type linestring has a relevant regulatory element.
Required information for a stop line is written in the [Autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/category_stop_line/#vm-02-02-stop-sign).

The output issue marks "linestring" as the **primitive**, and the linestring ID is written together as **ID**.

| Issue Code                             | Message                                           | Severity | Description                                                                                 | Approach                                                   |
| -------------------------------------- | ------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| StopLine.MissingRegulatoryElements-001 | "No regulatory element refers to this stop line." | Error    | There is a `stop_line` type linestring that hasn't been referred to any regulatory element. | Create a regulatory element that refers to this stop line. |

## Related source codes

- missing_regulatory_elements_for_stop_line.cpp
- missing_regulatory_elements_for_stop_line.hpp

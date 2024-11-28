# missing_regulator_elements_for_crosswalk

## Validator name

mapping.crosswalk.missing_regulatory_elements

## Feature

This validator checks whether each `crosswalk` subtype lanelet has a relevant regulatory element.
Required information for a crosswalk is written in the [Autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/category_crosswalk/#vm-05-01-crosswalks-across-the-road).

The output issue marks "lanelet" as the **primitive**, and the lanelet ID is written together as **ID**.

| Issue Code                              | Message                                           | Severity | Description                                                                                 | Approach                                                                            |
| --------------------------------------- | ------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Crosswalk.MissingRegulatoryElements-001 | "No regulatory element refers to this crosswalk." | Error    | There is a `crosswalk` subtype lanelet that hasn't been referred to any regulatory element. | Create a `crosswalk` subtype regulatory element and refer to the crosswalk lanelet. |

## Related source codes

- missing_regulatory_elements_for_crosswalk.hpp
- missing_regulatory_elements_for_crosswalk.cpp

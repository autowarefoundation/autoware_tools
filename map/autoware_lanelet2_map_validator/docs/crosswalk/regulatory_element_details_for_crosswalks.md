# regulatory_element_details_for_crosswalks

## Validator name

mapping.crosswalk.regulatory_element_details

## Feature

This validator checks whether the details in the `crosswalk` subtype regulatory elements are valid.
Required information for a crosswalk is written in the [Autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/category_crosswalk/#vm-05-01-crosswalks-across-the-road).
This validator checks eight types of issues.

The output issue marks "lanelet", "linestring" or "regulatory_element" as the **primitive**, and the regulatory element ID is written together as **ID**.

| Message                                                                                  | Severity | Primitive          | Description                                                                                                            | Approach                                                                                                                                                                  |
| ---------------------------------------------------------------------------------------- | -------- | ------------------ | ---------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| "Refers of crosswalk regulatory element must have type of crosswalk."                    | Error    | lanelet            | There is a `crosswalk` subtype regulatory element whose `refers` is not a `crosswalk` subtype lanelet.                 | Check that the `refers` is a `crosswalk` subtype lanelet                                                                                                                  |
| "ref_line of crosswalk regulatory element must have type of stopline."                   | Error    | linestring         | There is a `crosswalk` subtype regulatory element whose `ref_line` is not a `stop_line` type linestring.               | Check that the `ref_line` is a `stop_line` type linestring                                                                                                                |
| "Crosswalk polygon of crosswalk regulatory element must have type of Crosswalk_polygon." | Error    | polygon            | There is a `crosswalk` subtype regulatory element whose `crosswalk_polygon` is not a `crosswalk_polygon` type polygon. | Check that the `crosswalk_polygon` mentioned in the regulatory element refers to a `crosswalk_polygon` type area.                                                         |
| "Regulatory element of crosswalk must have lanelet of crosswalk(refers)."                | Error    | regulatory element | There is a `crosswalk` subtype regulatory element that has no `refers`es.                                              | Write `refers` referring to a `crosswalk` subtype lanelet in the regulatory element                                                                                       |
| "Regulatory element of crosswalk must have only one lanelet of crosswalk(refers)."       | Error    | regulatory element | There is a `crosswalk` subtype regulatory element that has multiple `refers`es.                                        | A `crosswalk` subtype regulatory element can have only one `refers`. Remove the `refers` that is not a crosswalk lanelet.                                                 |
| "Regulatory element of crosswalk does not have stop line(ref_line)."                     | Info     | regulatory element | There is a `crosswalk` subtype regulatory element that has no `ref_line`s                                              | Generally, there should be a stop line for the crosswalk. Be sure that the stop line exists or doesn't.                                                                   |
| "Regulatory element of crosswalk is nice to have crosswalk_polygon."                     | Warning  | regulatory element | There is a `crosswalk` subtype regulatory element that has no `crosswalk_polygon`s.                                    | It is recommended to surround a crosswalk with a `crosswalk_polygon`. Create one and add a `crosswalk_polygon` role member to the regulatory element with the polygon ID. |
| "Regulatory element of crosswalk must have only one crosswalk_polygon."                  | Error    | regulatory element | There is a `crosswalk` subtype regulatory element that has multiple `crosswalk_polygon`s.                              | Only one `crosswalk_polygon` is allowed per polygon. Remove the unnecessary ones.                                                                                         |

## Related source codes

- regulatory_element_details_for_crosswalks.hpp
- regulatory_element_details_for_crosswalks.cpp

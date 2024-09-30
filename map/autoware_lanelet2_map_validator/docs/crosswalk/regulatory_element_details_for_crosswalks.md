# regulatory_element_details_for_crosswalks

## Validator name

mapping.crosswalk.regulatory_element_details

## Feature

This validator checks whether the details in the `crosswalk` subtype regulatory elements are valid.
This validator checks seven types of issues.

All output issues specify the crosswalk "regulatory element" as the **primitive**, and the lanelet ID will be specified as the **ID**.

| Message                                                                                  | Severity | Description                                                                                                            |
| ---------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------- |
| "Refers of crosswalk regulatory element must have type of crosswalk."                    | Error    | There is a `crosswalk` subtype regulatory element whose `refers` is not a `crosswalk` subtype lanelet.                 |
| "ref_line of crosswalk regulatory element must have type of stopline."                    | Error    | There is a `crosswalk` subtype regulatory element whose `ref_line` is not a `stop_line` type linestring.               |
| "Crosswalk polygon of crosswalk regulatory element must have type of Crosswalk_polygon." | Error    | There is a `crosswalk` subtype regulatory element whose `crosswalk_polygon` is not a `crosswalk_polygon` type polygon. |
| "Regulatory element of cross walk must have lanelet of crosswalk(refers)."               | Error    | There is a `crosswalk` subtype regulatory element that has no `refers`es.                                              |
| "Regulatory element of cross walk must have only one lanelet of crosswalk(refers)."      | Error    | There is a `crosswalk` subtype regulatory element that has multiple `refers`es.                                        |
| "Regulatory element of cross walk does not have stop line(ref_line)."                    | Info     | There is a `crosswalk` subtype regulatory element that has no `ref_line`s                                              |
| "Regulatory element of cross walk is nice to have crosswalk_polygon."                    | Warning  | There is a `crosswalk` subtype regulatory element that has no `crosswalk_polygon`s.                                    |
| "Regulatory element of cross walk must have only one crosswalk_polygon."                 | Error    | There is a `crosswalk` subtype regulatory element that has multiple `crosswalk_polygon`s.                              |

## Related source codes

- regulatory_element_details_for_crosswalks.hpp
- regulatory_element_details_for_crosswalks.cpp

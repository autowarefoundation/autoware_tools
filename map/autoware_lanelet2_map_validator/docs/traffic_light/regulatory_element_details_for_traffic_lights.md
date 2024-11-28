# regulatory_element_details_for_traffic_lights

## Validator name

mapping.traffic_light.regulatory_element_details

## Feature

This validator checks whether the details in the `traffic_light` subtype regulatory elements are valid.
Required information for traffic lights is written in the [Autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/category_traffic_light/#vm-04-01-traffic-light-basics).
This validator checks four types of issues.

The output issue marks "linestring" or "regulatory element" as the **primitive**, and the lanelet ID is written together as **ID**.

| Issue Code                                | Message                                                                       | Severity | Primitive          | Description                                                                                                    | Approach                                                                                    |
| ----------------------------------------- | ----------------------------------------------------------------------------- | -------- | ------------------ | -------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| TrafficLight.RegulatoryElementDetails-001 | "Regulatory element of traffic light must have a stop line(ref_line)."        | Error    | regulatory element | There is a `traffic_light` subtype regulatory element that has no `ref_line`s                                  | Add `ref_line` to the regulatory element that refers to the id of the stop line linestring. |
| TrafficLight.RegulatoryElementDetails-002 | "Refers of traffic light regulatory element must have type of traffic_light." | Error    | linestring         | There is a `traffic_light` subtype regulatory element whose `refers` is not a `traffic_light` type linestring. | Check that the `refers` in the regulatory element is a `traffic_light` type linestring.     |
| TrafficLight.RegulatoryElementDetails-003 | "ref_line of traffic light regulatory element must have type of stop_line."   | Error    | linestring         | There is a `traffic_light` subtype regulatory element whose `ref_line` is not a `stop_line` type linestring.   | Check that the `ref_line` in the regulatory element is a `stop_line` type linestring        |

## Related source codes

- regulatory_element_details_for_traffic_lights.hpp
- regulatory_element_details_for_traffic_lights.cpp

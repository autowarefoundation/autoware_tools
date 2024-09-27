# regulatory_element_details_for_traffic_lights

## Validator name

mapping.traffic_light.regulatory_element_details

## Feature

This validator checks whether the details in the `traffic_light` subtype regulatory elements are valid.
This validator checks four types of issues.

All output issues specify the traffic_light "regulatory element" as the **primitive**, and the linestring ID will be specified as the **ID**.

| Message | Severity | Description |
| ------- | -------- | ----------- |
| "Refers of traffic light regulatory element must have type of traffic_light." | Error | There is a `traffic_light` subtype regulatory element whose `refers` is not a `traffic_light` type linestring. |
| "Refline of traffic light regulatory element must have type of stop_line." | Error | There is a `traffic_light` subtype regulatory element whose `ref_line` is not a `stop_line` type linestring. |
| "Regulatory element of traffic light must have a traffic light(refers)." | Error | There is a `traffic_light` subtype regulatory element that has no `refers`es. |
| "Regulatory element of traffic light must have a stop line(ref_line)." | Error | There is a `traffic_light` subtype regulatory element that has no `ref_line`s |

## Related source codes

- regulatory_element_details_for_traffic_lights.hpp
- regulatory_element_details_for_traffic_lights.cpp

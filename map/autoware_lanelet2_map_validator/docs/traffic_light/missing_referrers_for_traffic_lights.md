# missing_referrers_for_traffic_lights

## Validator name

mapping.traffic_light.missing_referrers

## Feature

This validator checks whether each `traffic_light` type regulatory element has been referred by at least one lanelet.

| Issue Code                        | Message                                                                       | Severity | Primitive          | Description                                                                                  | Approach                                                                                      |
| --------------------------------- | ----------------------------------------------------------------------------- | -------- | ------------------ | -------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| TrafficLight.MissingReferrers-001 | Regulatory element of traffic light must be referred by at least one lanelet. | Error    | Regulatory Element | There is a `traffic_light` type regulatory element that hasn't been referred by any lanelet. | The lanelet that might be controlled by the traffic light must refer this regulatory element. |

## Related source codes

- missing_referrers_for_traffic_lights.cpp
- missing_referrers_for_traffic_lights.hpp

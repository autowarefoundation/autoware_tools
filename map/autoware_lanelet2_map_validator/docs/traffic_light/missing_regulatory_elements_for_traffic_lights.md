# missing_regulator_elements_for_traffic_lights

## Validator name

mapping.traffic_light.missing_regulatory_elements

## Feature

This validator checks whether each `traffic_light` type linestring has a relavant regulatory element.
The output issue specifies the traffic_light "linestring" as the **primitive**, and the linestring ID will be specified as the **ID**.

| Message | Severity | Description |
| ------- | -------- | ----------- |
| "No regulatory element refers to this traffic light." | Error | There is a `traffic_light` type linestring that hasn't been referred to any regulatory element. |

## Related source codes

- missing_regulatory_elements_for_traffic_light.hpp
- missing_regulatory_elements_for_traffic_light.cpp

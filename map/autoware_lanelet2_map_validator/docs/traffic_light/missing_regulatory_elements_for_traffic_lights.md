# missing_regulator_elements_for_traffic_lights

## Validator name

mapping.traffic_light.missing_regulatory_elements

## Feature

This validator checks whether each `traffic_light` type linestring has a relevant regulatory element.
Required information for traffic lights is written in the [Autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/category_traffic_light/#vm-04-01-traffic-light-basics).

The output issue marks "linestring" as the **primitive**, and the linestring ID is written together as **ID**.

| Issue Code                                 | Message                                               | Severity | Description                                                                                     | Approach                                                                           |
| ------------------------------------------ | ----------------------------------------------------- | -------- | ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| TrafficLight.MissingRegulatoryElements-001 | "No regulatory element refers to this traffic light." | Error    | There is a `traffic_light` type linestring that hasn't been referred to any regulatory element. | Create a `traffic_light` subtype regulatory element that refers to this linestring |

## Related source codes

- missing_regulatory_elements_for_traffic_light.hpp
- missing_regulatory_elements_for_traffic_light.cpp

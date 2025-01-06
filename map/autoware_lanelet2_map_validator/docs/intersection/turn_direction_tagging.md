# turn_direction_tagging

## Validator name

mapping.intersection.turn_direction_tagging

## Feature

This validator checks whether lanelets inside the `intersection_area` polygon have a `turn_direction` tag and it is set to "straight", "left", or "right".

The validator will output the follwoing issues with the corresponding primitive ID.

| Issue Code                            | Message                                              | Severity | Primitive | Description                                                                                                        | Approach                                                                                                                                                             |
| ------------------------------------- | ---------------------------------------------------- | -------- | --------- | ------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Intersection.TurnDirectionTagging-001 | "This lanelet is missing a turn_direction tag."      | Error    | Lanelet   | Lanelets at intersections must have a `turn_direction` tag but this lanelet doesn't have it.                       | Set a `turn_direction` tag to the lanelet with a value of `straight`, `left` or `right`. This tells the vehicle whether to use the blinkers or not at intersections. |
| Intersection.TurnDirectionTagging-002 | "Invalid turn_direction tag <INVALID_TAG> is found." | Error    | Lanelet   | The `turn_direction` tag of this lanelet is set to <INVALID_TAG> while it has to be `straight`, `left` or `right`. | Fix the tag value to `straight`, `left` or `right`.                                                                                                                  |

## Related source codes

- turn_direction_tagging.cpp
- turn_direction_tagging.hpp

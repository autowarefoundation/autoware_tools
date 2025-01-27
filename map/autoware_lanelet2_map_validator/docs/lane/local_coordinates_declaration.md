# local_coordinates_declaration

## Validator name

mapping.lane.local_coordinates_declaration

## Feature

This validator checks whether local coordinates are declared to each point in a Lanelet2 map if one or more points have local coordinates. If a Lanelet2 map is made to have local coordinates, each point MUST have both a `local_x` tag and a `local_y` tag.

If the Lanelet2 map is NOT made to have local coordinates and doesn't have a single point with `local_x` or `local_y` declared, this validator ignores the map and outputs no issues.

The validator outputs the following issue with the corresponding ID of the primitive.

| Issue Code                           | Message                                                    | Severity | Primitive | Description                                                                                                                                                  | Approach                                                                                   |
| ------------------------------------ | ---------------------------------------------------------- | -------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------ |
| Lane.LocalCoordinatesDeclaration-001 | This point doesn't have local coordinates while others do. | Error    | Point     | The map has a point that has tags `local_x` and `local_y` but this point doesn't have them. All points in a map must be set to have both tags or not at all. | Be sure to align whether each point has or does not have both tabs `local_x` and `local_y` |
| Lane.LocalCoordinatesDeclaration-002 | "local_x" is declared but "local_y" is not."               | Error    | Point     | The point has a `local_x` tag but doesn't have a `local_y` tag.                                                                                              | Be sure that all points have both tags `local_x` and `local_y`.                            |
| Lane.LocalCoordinatesDeclaration-003 | "local_y" is declared but "local_x" is not."               | Error    | Point     | The point has a `local_y` tag but doesn't have a `local_x` tag.                                                                                              | Be sure that all points have both tags `local_x` and `local_y`.                            |

## Related source codes

- local_coordinates_declaration.cpp
- local_coordinates_declaration.hpp

# intersection_area_segment_type

## Validator name

mapping.intersection.intersection_area_segment_type

## Feature

This validator check whether each `intersection_area` type polygon is made from points that belong to `road_border` type linestrings or the starting/ending edge of a lanelet.

This is achieved by the following procedure.

1. Create a 2D bounding box that circumscribes the polygon.
2. Collect `road_border` type linestrings within or intersecting the 2D bounding box.
3. Collect starting/ending edges of lanelets within or intersecting the 2D bounding box.
4. Examine each point that constitutes the polygon whether it belongs to the collection above.

The validator outputs the following issue with the corresponding ID of the primitive.

| Issue Code                                   | Message                                                                                                                     | Severity | Primitive | Description                                                                                                                                                                 | Approach                                                                                           |
| -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- | -------- | --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| Intersection.IntersectionAreaSegmentType-001 | "This intersection area is not made by points from road_border linestrings or lanelet edges. (Point ID: \<POINT ID LIST\>)" | Error    | Polygon   | The `intersection_area` polygon has points that doesn't belong to `road_border` type linestrings or lanelet edges. The violating points are listed up at \<POINT ID LIST\>. | Ensure that the `intersection_area` is formed ONLY by `road_border` linestrings and lanelet edges. |

### Supplementary information

Note that this validator only examines what type of linestring the points constituting the polygon belongs to, and doesn't examine they have a valid connection. Use the `mapping.intersection.intersection_area_validity` to check whether the polygon is `boost::geometry::is_valid()`.

## Related source codes

- intersection_area_segment_type.hpp
- intersection_area_segment_type.cpp

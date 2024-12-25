# intersection_area_dangling_reference

## Validator name

mapping.intersection.intersection_area_dangling_reference

## Feature

This validator checks whether each intersection lanelet(namely the lanelet with `turn_direction` property) has reference to existing `intersection_area` polygon. Such invalid case occurs when an intersection_area is deleted but its referrers are not updated.

This is achieved by the following procedure.

1. Obtain the set of `intersection_area` polygon IDs
2. Check if intersection lanelet has "intersection_area" key and its value is contained in the above IDs

The validator outputs the following issue with the corresponding ID of the primitive.

| Issue Code                                         | Message                                                                                 | Severity | Primitive | Description                                                                        | Approach                                                             |
| -------------------------------------------------- | --------------------------------------------------------------------------------------- | -------- | --------- | ---------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| Intersection.IntersectionAreaDanglingReference-001 | "Dangling reference to non-existing intersection area of ID \<LANELET ID\> is detected" | Error    | Lanelet   | Lookup to `intersection_area` from the reporeted lanelet will cause runtime error. | Go to the reported lanelet and delete "intersection_area" key entry. |

### Supplementary information

## Related source codes

- intersection_area_dangling_reference.hpp
- intersection_area_dangling_reference.cpp

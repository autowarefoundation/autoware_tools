# intersection_area_validity

## Validator name

mapping.intersection.intersection_area_validity

## Feature

This validator check whether each `intersection_area` type polygon satisfies [`boost::geometry::is_valid`](https://www.boost.io/doc/libs/1_86_0/libs/geometry/doc/html/geometry/reference/algorithms/is_valid/is_valid_2_with_message.html).

The validator outputs the following issue with the corresponding ID of the primitive.

| Issue Code                                | Message                                                                                 | Severity | Primitive | Description                                                                 | Approach                                                                                                                                                                                                                                                                       |
| ----------------------------------------- | --------------------------------------------------------------------------------------- | -------- | --------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Intersection.IntersectionAreaValidity-001 | "This intersection_area doesn't satisfy boost::geometry::is_valid (reason: \<MESSAGE\>) | Error    | Polygon   | The `intersection_area` polygon didn't satisfy `boost::geometry::is_valid`. | There are several reasons expected and it is written in "(reason: \<MESSAGE\>)". The \<MESSAGE\> is a copy of [the output message defined in the `boost::geometry` library](https://www.boost.org/doc/libs/1_86_0/boost/geometry/policies/is_valid/failing_reason_policy.hpp). |

## Related source codes

- intersection_area_validity.hpp
- intersection_area_validity.cpp

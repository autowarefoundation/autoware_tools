# autoware_lanelet2_map_validator

`autoware_lanelet2_map_validator` is a tool to validate Lanelet2 maps to ensure that Autoware can work properly with it.

The requirements for lanelet2 maps are described in [Vector Map creation requirement specifications (in Autoware Documentation)](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/).

| ID       | Requirements                                            |
| -------- | ------------------------------------------------------- |
| vm-01-01 | Lanelet basics                                          |
| vm-01-02 | Allowance for lane changes                              |
| vm-01-03 | Linestring sharing                                      |
| vm-01-04 | Sharing of the centerline of lanes for opposing traffic |
| vm-01-05 | Lane geometry                                           |
| vm-01-06 | Line position (1)                                       |
| vm-01-07 | Line position (2)                                       |
| vm-01-08 | Line position (3)                                       |
| vm-01-09 | Speed limits                                            |
| vm-01-10 | Centerline                                              |
| vm-01-11 | Centerline connection (1)                               |
| vm-01-12 | Centerline connection (2)                               |
| vm-01-13 | Roads with no centerline (1)                            |
| vm-01-14 | Roads with no centerline (2)                            |
| vm-01-15 | Road shoulder                                           |
| vm-01-16 | Road shoulder Linestring sharing                        |
| vm-01-17 | Side strip                                              |
| vm-01-18 | Side strip Linestring sharing                           |
| vm-01-19 | Walkway                                                 |
| vm-02-01 | Stop line alignment                                     |
| vm-02-02 | Stop sign                                               |
| vm-03-01 | Intersection criteria                                   |
| vm-03-02 | Lanelet's turn direction and virtual                    |
| vm-03-03 | Lanelet width in the intersection                       |
| vm-03-04 | Lanelet creation in the intersection                    |
| vm-03-05 | Lanelet division in the intersection                    |
| vm-03-06 | Guide lines in the intersection                         |
| vm-03-07 | Multiple lanelets in the intersection                   |
| vm-03-08 | Intersection Area range                                 |
| vm-03-09 | Range of Lanelet in the intersection                    |
| vm-03-10 | Right of way (with signal)                              |
| vm-03-11 | Right of way (without signal)                           |
| vm-03-12 | Right of way supplements                                |
| vm-03-13 | Merging from private area, sidewalk                     |
| vm-03-14 | Road marking                                            |
| vm-03-15 | Exclusive bicycle lane                                  |
| vm-04-01 | Traffic light basics                                    |
| vm-04-02 | Traffic light position and size                         |
| vm-04-03 | Traffic light lamps                                     |
| vm-05-01 | Crosswalks across the road                              |
| vm-05-02 | Crosswalks with pedestrian signals                      |
| vm-05-03 | Deceleration for safety at crosswalks                   |
| vm-05-04 | Fences                                                  |
| vm-06-01 | Buffer Zone                                             |
| vm-06-02 | No parking signs                                        |
| vm-06-03 | No stopping signs                                       |
| vm-06-04 | No stopping sections                                    |
| vm-06-05 | Detection area                                          |
| vm-07-01 | Vector Map creation range                               |
| vm-07-02 | Range of detecting pedestrians who enter the road       |
| vm-07-03 | Guardrails, guard pipes, fences                         |
| vm-07-04 | Ellipsoidal height                                      |

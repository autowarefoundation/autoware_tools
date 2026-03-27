# Comprehensive Open-Loop Metric Implementation Note

## TL;DR

- Current branch: `feat/epdms-hc`
- First implemented subscore: `HC`
- Status: implemented and verified
- Verification: package build passed, `test_offline_evaluation` passed
- Current position: good first production implementation in this package
- Important note: not yet exact NAVSIM-parity because NAVSIM-style history stitching is not yet reproduced

## Current Status

Implemented:

1. `HC`

Not yet implemented:

1. `DAC`
2. `LK`
3. `EC`
4. `DDC`
5. `TLC`
6. `TTC`
7. `EP`
8. `NC`
9. fairness filtering
10. final `EPDMS` aggregation

## `HC` Note

### Phase

- first implemented subscore
- foundational for later `EC`

### Result

Implemented trajectory-based `HC` in the open-loop evaluator.

Added:

- `history_comfort` score output
- comfort-related debug signals for trajectory analysis
- JSON output support
- rosbag output support
- regression test coverage

Main code:

- [trajectory_metrics.cpp](/home/beomseokkim2/workspace/pilot-auto/src/tools/planning/autoware_planning_data_analyzer/src/metrics/trajectory_metrics.cpp)
- [open_loop_evaluator.cpp](/home/beomseokkim2/workspace/pilot-auto/src/tools/planning/autoware_planning_data_analyzer/src/open_loop_evaluator.cpp)
- [test_open_loop_gt_source_mode.cpp](/home/beomseokkim2/workspace/pilot-auto/src/tools/planning/autoware_planning_data_analyzer/test/test_open_loop_gt_source_mode.cpp)

### Equation

$$
HC =
\begin{cases}
1, & \text{if all comfort constraints hold over the evaluated trajectory} \\
0, & \text{otherwise}
\end{cases}
$$

In this implementation, the comfort constraints are checked on finite-difference signals derived from the evaluated trajectory points:

$$
a_x(i) = \frac{v_x(i+1) - v_x(i)}{\Delta t(i)}, \qquad
a_y(i) = \frac{v_y(i+1) - v_y(i)}{\Delta t(i)}
$$

$$
j_x(i) = \frac{a_x(i+1) - a_x(i)}{\Delta t(i)}, \qquad
j_y(i) = \frac{a_y(i+1) - a_y(i)}{\Delta t(i)}
$$

$$
j(i) = \sqrt{j_x(i)^2 + j_y(i)^2}
$$

$$
\dot{\psi}(i) = \frac{\operatorname{wrap}(\psi(i+1)-\psi(i))}{\Delta t(i)}, \qquad
\ddot{\psi}(i) = \frac{\dot{\psi}(i+1)-\dot{\psi}(i)}{\Delta t(i)}
$$

$$
\Delta t(i) = \max\left(t_{i+1}-t_i,\;10^{-3}\right)
$$

The implemented pass condition is:

$$
\forall i,\;
-4.05 < a_x(i) < 2.40,\;
|a_y(i)| < 4.89,\;
|j(i)| < 8.37,\;
|j_x(i)| < 4.13,\;
|\dot{\psi}(i)| < 0.95,\;
|\ddot{\psi}(i)| < 1.93
$$

Briefly:

- `a_x` and `a_y` come from trajectory velocity differences
- `j_x` and `j_y` come from acceleration differences
- `j` is the combined jerk magnitude
- `\dot{\psi}` and `\ddot{\psi}` come from trajectory heading differences
- if any sampled point violates any bound, `HC = 0`

Applied thresholds:

- longitudinal acceleration
- lateral acceleration
- jerk magnitude
- longitudinal jerk
- yaw rate
- yaw acceleration

Threshold source:

- [epdms_metric.md](/home/beomseokkim2/workspace/pilot-auto/src/tools/planning/autoware_planning_data_analyzer/docs/epdms_metric.md)

### Verification

Verified with:

```bash
colcon build --packages-select autoware_planning_data_analyzer
colcon test --packages-select autoware_planning_data_analyzer --ctest-args -R test_offline_evaluation
```

Result:

- build passed
- test passed

### Further Note

This implementation is:

- appropriate for this package
- mathematically aligned with the intended `HC` threshold logic
- suitable as the first step toward full `EPDMS`

This implementation is not yet:

- a strict NAVSIM-faithful replica

Reason:

- the current analyzer checks comfort directly on the evaluated planned trajectory
- full NAVSIM parity would require reproducing NAVSIM-style history construction before the comfort check

## Next

Recommended next metrics:

1. `DAC`
2. `LK`
3. `EC`

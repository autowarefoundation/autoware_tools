# Displacement-Based Metrics

This document summarizes the ground-truth-comparison metrics that are written to the output bag by the open-loop evaluator.

Target metric topics:

- `ade`
- `fde`
- `ahe`
- `fhe`
- `lateral_deviation`
- `longitudinal_deviation`

These metrics are computed point-wise against ground truth over the predicted trajectory. They are saved as output bag topics for the full predicted trajectory, and they are also used to derive multi-horizon results for `full`, `1s`, `2s`, `4s`, and `8s`.

## Common Setup

Let the predicted trajectory contain points indexed by $i = 0, 1, \ldots, N$.

For each point $i$:

- $(x_i^{\mathrm{pred}}, y_i^{\mathrm{pred}})$: predicted position
- $(x_i^{\mathrm{gt}}, y_i^{\mathrm{gt}})$: ground-truth position
- $\psi_i^{\mathrm{pred}}$: predicted yaw
- $\psi_i^{\mathrm{gt}}$: ground-truth yaw
- $t_i$: relative timestamp of the predicted point

In this project, $t_i$ means the `time_from_start` of the $i$-th predicted trajectory point.

For a requested horizon `H`, the evaluator chooses:

$$
k = \max \{ i \mid t_i \le H \}
$$

and only reports that horizon if:

$$
H - t_k \le 0.1
$$

This means horizon metrics are derived from the latest trajectory point whose `time_from_start` does not exceed the requested horizon, with a tolerance of `0.1s`.

## ADE

### Definition

Average Displacement Error (ADE) is the average 2D position error between the predicted and ground-truth trajectories.

### Official Equation

$$
\mathrm{ADE} = \frac{1}{M} \sum_{i=1}^{M} \sqrt{(x_i^{gt} - x_i^{pred})^2 + (y_i^{gt} - y_i^{pred})^2}
$$

### Implemented Equation in This Project

First define the point-wise displacement error:

$$
e_i = \sqrt{(x_i^{gt} - x_i^{pred})^2 + (y_i^{gt} - y_i^{pred})^2}
$$

The saved `ade` topic stores the running ADE array:

$$
\mathrm{ADE}_i = \frac{1}{i+1} \sum_{j=0}^{i} e_j
$$

For a horizon `H`, the reported horizon ADE is:

$$
\mathrm{ADE}(H) = \mathrm{ADE}_k = \frac{1}{k+1} \sum_{j=0}^{k} e_j
$$

### Implementation Notes

- The average starts from the first available predicted trajectory point, not from a separate global time origin.
- If the first predicted point has `time_from_start = 0.1s`, then ADE starts from that point.
- The output bag topic is a `Float64MultiArray` containing the running ADE over the full trajectory.
- Multi-horizon ADE values are derived from this array using the horizon cutoff.

## FDE

### Definition

Final Displacement Error (FDE) is the 2D position error at the final evaluation point.

### Official Equation

$$
\mathrm{FDE} = \sqrt{(x_M^{gt} - x_M^{pred})^2 + (y_M^{gt} - y_M^{pred})^2}
$$

### Implemented Equation in This Project

Using the same point-wise displacement error:

$$
e_i = \sqrt{(x_i^{gt} - x_i^{pred})^2 + (y_i^{gt} - y_i^{pred})^2}
$$

the horizon FDE is:

$$
\mathrm{FDE}(H) = e_k
$$

### Implementation Notes

- The saved `fde` topic contains the point-wise displacement error array `e_i`, not a running FDE.
- The horizon or full-trajectory FDE is obtained by taking the cutoff element from that array.
- This is why the topic name is `fde` even though the bag message contains one value per trajectory point.

## AHE

### Definition

Average Heading Error (AHE) is the average absolute heading difference between the predicted and ground-truth trajectories.

### Official Equation

$$
\mathrm{AHE} = \frac{1}{M} \sum_{i=1}^{M} \left| \mathrm{normalize}(\psi_i^{pred} - \psi_i^{gt}) \right|
$$

### Implemented Equation in This Project

Define the point-wise heading error:

$$
h_i = \left| \mathrm{normalize}(\psi_i^{pred} - \psi_i^{gt}) \right|
$$

The saved `ahe` topic stores the running average heading error array:

$$
\mathrm{AHE}_i = \frac{1}{i+1} \sum_{j=0}^{i} h_j
$$

For a horizon `H`, the reported horizon AHE is:

$$
\mathrm{AHE}(H) = \mathrm{AHE}_k = \frac{1}{k+1} \sum_{j=0}^{k} h_j
$$

### Implementation Notes

- Heading error is stored in radians.
- The absolute heading difference is normalized before taking the magnitude.
- The output bag topic is a `Float64MultiArray` containing the running AHE over the full trajectory.

## FHE

### Definition

Final Heading Error (FHE) is the absolute heading error at the final evaluation point.

### Official Equation

$$
\mathrm{FHE} = \left| \mathrm{normalize}(\psi_M^{pred} - \psi_M^{gt}) \right|
$$

### Implemented Equation in This Project

Using the point-wise heading error:

$$
h_i = \left| \mathrm{normalize}(\psi_i^{pred} - \psi_i^{gt}) \right|
$$

the horizon FHE is:

$$
\mathrm{FHE}(H) = h_k
$$

### Implementation Notes

- The saved `fhe` topic contains the point-wise heading error array `h_i`, not a running FHE.
- The horizon or full-trajectory FHE is obtained by taking the cutoff element from that array.
- This mirrors how `fde` is handled for position error.

## Lateral Deviation

### Definition

Lateral deviation is the signed lateral position error between prediction and ground truth, measured in the ground-truth vehicle frame.

### Implemented Equation in This Project

First compute the global position difference:

$$
\Delta x_i = x_i^{pred} - x_i^{gt}, \quad \Delta y_i = y_i^{pred} - y_i^{gt}
$$

Then rotate this difference by $-\psi_i^{\mathrm{gt}}$ into the ground-truth vehicle frame:

$$
\begin{bmatrix}
\Delta x_i^{veh} \\
\Delta y_i^{veh}
\end{bmatrix}
=
\begin{bmatrix}
\cos(-\psi_i^{gt}) & -\sin(-\psi_i^{gt}) \\
\sin(-\psi_i^{gt}) & \cos(-\psi_i^{gt})
\end{bmatrix}
\begin{bmatrix}
\Delta x_i \\
\Delta y_i
\end{bmatrix}
$$

The implemented lateral deviation is:

$$
\mathrm{LatDev}_i = \Delta y_i^{veh}
$$

For a horizon `H`, the evaluator derives:

$$
\mathrm{AverageLateralDeviation}(H) = \frac{1}{k+1} \sum_{j=0}^{k} |\mathrm{LatDev}_j|
$$

$$
\mathrm{MaxLateralDeviation}(H) = \max_{0 \le j \le k} |\mathrm{LatDev}_j|
$$

### Implementation Notes

- The saved `lateral_deviation` topic contains the signed point-wise lateral deviation array.
- The horizon metrics use absolute values when computing average and max aggregates.
- This metric is different from route-centerline lateral deviation used by lane-keeping logic.

## Longitudinal Deviation

### Definition

Longitudinal deviation is the signed longitudinal position error between prediction and ground truth, measured in the ground-truth vehicle frame.

### Implemented Equation in This Project

Using the same rotated vehicle-frame offset as above, the implemented longitudinal deviation is:

$$
\mathrm{LonDev}_i = \Delta x_i^{veh}
$$

For a horizon `H`, the evaluator derives:

$$
\mathrm{AverageLongitudinalDeviation}(H) = \frac{1}{k+1} \sum_{j=0}^{k} |\mathrm{LonDev}_j|
$$

$$
\mathrm{MaxLongitudinalDeviation}(H) = \max_{0 \le j \le k} |\mathrm{LonDev}_j|
$$

### Implementation Notes

- The saved `longitudinal_deviation` topic contains the signed point-wise longitudinal deviation array.
- The horizon metrics use absolute values when computing average and max aggregates.

## Output Bag Topics

The following ground-truth-comparison metric topics are written to the output bag:

- `ade`
- `fde`
- `ahe`
- `fhe`
- `lateral_deviation`
- `longitudinal_deviation`

All of the above are written as `std_msgs/msg/Float64MultiArray`.

## Horizon-Derived Results

The following multi-horizon results are derived from the arrays above for `full`, `1s`, `2s`, `4s`, and `8s`:

- `ADE`
- `FDE`
- `AHE`
- `FHE`
- `average_lateral_deviation`
- `max_lateral_deviation`
- `average_longitudinal_deviation`
- `max_longitudinal_deviation`

These horizon results are stored in the structured result JSON written by the evaluator, rather than as separate per-horizon bag topics.

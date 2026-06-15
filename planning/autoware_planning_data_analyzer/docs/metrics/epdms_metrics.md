# EPDMS Metrics

This document summarizes the Autoware-compatible migrated EPDMS metrics written
by the open-loop evaluator. The metric is migrated from the original NAVSIM
EPDMS definition in
<https://github.com/autonomousvision/navsim/blob/main/docs/metrics.md>.

It describes the semantic score first, then the migrated Autoware subscore
equations used by this package.

This Autoware-compatible migration keeps the final single-trajectory EPDMS
composition, but adapts the inputs to Autoware ROS topics, Autoware lanelet
maps, and one selected planner trajectory. The score is local to each
synchronized evaluation sample; it does not implement NAVSIM's later
pseudo-closed-loop scene aggregation stage.

## Common Notation

Let the selected trajectory be:

$$
Q = (q_0, q_1, \ldots, q_N)
$$

For trajectory sample $q_t$:

- $T_t$ is the relative time from `time_from_start`.
- $p_t$ is the ego pose.
- $\vec{v}_t$ is the 2D world-frame velocity vector.
- $P_t$ is the ego footprint polygon generated from `VehicleInfo` at $p_t$.
- $X_{t,k}$ is an ego footprint corner, with $k$ ranging over the four corners.
- $R_t$ is the route/lanelet context available from `RouteHandler`.

Each subscore is reported with an availability flag. The synthetic EPDMS score
is available only when every required raw subscore is available.

The EPDMS subscores are written as uppercase symbols in equations. Each item
links to the corresponding subscore logic section:

- [`NC`](#nc-no-at-fault-collision): no at-fault collision
- [`DAC`](#dac-drivable-area-compliance): drivable area compliance
- [`DDC`](#ddc-driving-direction-compliance): driving direction compliance
- [`TLC`](#tlc-traffic-light-compliance): traffic light compliance
- [`TTC`](#ttc-time-to-collision-within-bound): time-to-collision within bound
- [`LK`](#lk-lane-keeping): lane keeping
- [`HC`](#hc-history-comfort): history comfort
- [`EC`](#ec-extended-comfort): extended comfort
- [`EP`](#ep-ego-progress): ego progress

## Parameters

This section lists the main constants used by the implemented EPDMS equations.

| Symbol / value                    |                                      Default | Used by                  | Meaning                                                                   |
| --------------------------------- | -------------------------------------------: | ------------------------ | ------------------------------------------------------------------------- |
| $w_{EP}$                          |                                          `5` | EPDMS                    | Ego-progress weight                                                       |
| $w_{TTC}$                         |                                          `5` | EPDMS                    | Time-to-collision within bound weight                                     |
| $w_{LK}$                          |                                          `2` | EPDMS                    | Lane-keeping weight                                                       |
| $w_{HC}$                          |                                          `2` | EPDMS                    | History-comfort weight                                                    |
| $w_{EC}$                          |                                          `2` | EPDMS                    | Extended-comfort weight                                                   |
| $\epsilon_H$                      |                                     `1.0e-9` | Human filter             | Human-reference zero threshold                                            |
| $\tau_{stop}$                     |                                   `0.05 m/s` | NC                       | Stopped ego / stopped track threshold                                     |
| $\theta_{behind}$                 |                                    `150 deg` | NC, TTC                  | Object-behind angle threshold                                             |
| $r_{semantic}$                    |                                       `15 m` | DAC, shared area context | Expanded search range for semantic drivable-area polygons                 |
| $r_{border}$                      |                                        `5 m` | DAC, shared area context | Expanded search range for `road_border` line strings                      |
| $\rho$                            | `{0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0} m` | DAC                      | Road-border side-probe distances                                          |
| $d^{max}_{corner,border}$         |                                      `3.0 m` | DAC                      | Maximum corner-to-road-border distance for fallback acceptance            |
| $d^{max}_{semantic,border}$       |                                      `4.0 m` | DAC                      | Maximum semantic-boundary-to-road-border distance for fallback acceptance |
| $T_{DDC}$                         |                                      `1.0 s` | DDC                      | Rolling window for oncoming-progress accumulation                         |
| $C_{minor}$                       |                                      `2.0 m` | DDC                      | Lower threshold for partial DDC penalty                                   |
| $C_{major}$                       |                                      `6.0 m` | DDC                      | Threshold for full DDC penalty                                            |
| $\tau_{TTC,stop}$                 |                                  `0.005 m/s` | TTC                      | Stopped ego threshold for skipping TTC projection                         |
| $\delta$                          |                     `{0.0, 0.3, 0.6, 0.9} s` | TTC                      | Future projection offsets                                                 |
| $\theta_{ahead}$                  |                                     `30 deg` | TTC                      | Object-ahead angle threshold                                              |
| $D_{max}$                         |                                      `0.5 m` | LK                       | Maximum accepted lateral deviation from route centerline                  |
| $T_{LK}$                          |                                      `2.0 s` | LK                       | Maximum continuous lane-keeping violation duration                        |
| $T^{pre}_{LC}$                    |                                      `1.0 s` | LK                       | Lane-change pre-grace duration                                            |
| $T^{post}_{LC}$                   |                                      `1.0 s` | LK                       | Lane-change post-grace duration                                           |
| $v_{queue}$                       |                                    `1.0 m/s` | LK                       | Queue low-speed threshold                                                 |
| $T_{queue}$                       |                                      `1.0 s` | LK                       | Queue progress-check window                                               |
| $d_{queue}$                       |                                      `1.5 m` | LK                       | Maximum progress for queue exemption                                      |
| $T_{release}$                     |                                      `1.5 s` | LK                       | Queue-release grace duration                                              |
| $T_{past}$                        |                                      `1.5 s` | HC                       | Past motion-history horizon                                               |
| $\Delta T_{HC}$                   |                                      `0.1 s` | HC                       | History-comfort sample interval                                           |
| $T_{future}$                      |                                      `4.0 s` | HC                       | Future trajectory horizon used for HC                                     |
| $a_x^{min}$                       |                                `-4.05 m/s^2` | HC                       | Minimum longitudinal acceleration                                         |
| $a_x^{max}$                       |                                 `2.40 m/s^2` | HC                       | Maximum longitudinal acceleration                                         |
| $\lvert a_y \rvert^{max}$         |                                 `4.89 m/s^2` | HC                       | Maximum lateral acceleration magnitude                                    |
| $\lvert j \rvert^{max}$           |                                 `8.37 m/s^3` | HC                       | Maximum jerk magnitude                                                    |
| $\lvert j_x \rvert^{max}$         |                                 `4.13 m/s^3` | HC                       | Maximum longitudinal jerk magnitude                                       |
| $\lvert \dot{\psi} \rvert^{max}$  |                                 `0.95 rad/s` | HC                       | Maximum yaw-rate magnitude                                                |
| $\lvert \ddot{\psi} \rvert^{max}$ |                               `1.93 rad/s^2` | HC                       | Maximum yaw-acceleration magnitude                                        |
| $\tau_a$                          |                                        `0.7` | EC                       | Maximum acceleration RMS discrepancy                                      |
| $\tau_j$                          |                                        `0.5` | EC                       | Maximum jerk RMS discrepancy                                              |
| $\tau_{\dot{\psi}}$               |                                        `0.1` | EC                       | Maximum yaw-rate RMS discrepancy                                          |
| $\tau_{\ddot{\psi}}$              |                                        `0.1` | EC                       | Maximum yaw-acceleration RMS discrepancy                                  |
| $\tau_G$                          |                                      `5.0 m` | EP                       | Progress denominator threshold for fallback                               |

## EPDMS Definition

The Extended Predictive Driver Model Score (EPDMS) is a comprehensive, rule-based evaluation metric used to benchmark the performance and safety of autonomous driving systems and trajectory planners

EPDMS scores one selected trajectory by multiplying safety and rule-compliance
subscores with a weighted quality term. The human filter is applied inside the
aggregation so that failures also observed in the human reference are suppressed.

| Metric                                 | Weight     | Range         |
| -------------------------------------- | ---------- | ------------- |
| No at-fault collision (`NC`)           | multiplier | `{0, 0.5, 1}` |
| Drivable area compliance (`DAC`)       | multiplier | `{0, 1}`      |
| Driving direction compliance (`DDC`)   | multiplier | `{0, 0.5, 1}` |
| Traffic light compliance (`TLC`)       | multiplier | `{0, 1}`      |
| Ego progress (`EP`)                    | `5`        | `[0, 1]`      |
| Time-to-collision within bound (`TTC`) | `5`        | `{0, 1}`      |
| Lane keeping (`LK`)                    | `2`        | `{0, 1}`      |
| History comfort (`HC`)                 | `2`        | `{0, 1}`      |
| Extended comfort (`EC`)                | `2`        | `{0, 1}`      |

The full EPDMS is defined as:

$$
EPDMS =
\left(
\prod_{m \in \{NC, DAC, DDC, TLC\}}
filter_m(agent, human)
\right)
\cdot
\left(
\frac{
\sum_{m \in \{EP, TTC, LK, HC, EC\}}
w_m \cdot filter_m(agent, human)
}{
\sum_{m \in \{EP, TTC, LK, HC, EC\}}
w_m
}
\right)
$$

where:

$$
filter_m(agent, human) =
\begin{cases}
1, & \text{if } m \ne EC,\ A_m \text{ and } H_m \text{ are available, and } \left\lvert H_m \right\rvert \le 10^{-9} \\
A_m, & \text{otherwise}
\end{cases}
$$

Here:

Here:

- **Multiplicative penalty gate**:  
  The left product term combines `NC`, `DAC`, `DDC`, and `TLC`. These are safety
  and rule-compliance checks. A score of `0` collapses EPDMS to `0`, while `0.5`
  partially penalizes it. Therefore, this term controls whether the weighted
  quality score should pass through fully, partially, or not at all.

- **Weighted quality term**:  
  The right fraction combines `EP`, `TTC`, `LK`, `HC`, and `EC` using their
  configured weights. It is normalized by the total weight, so it remains in
  `[0, 1]`. Larger weights make `EP` and `TTC` more influential than lane
  keeping and comfort.

- **Human filter**:  
  `filter_m(agent, human)` returns `1` when the human reference also failed the
  same metric, suppressing shared human-reference failures. Otherwise, it keeps
  the agent score. `EC` bypasses this filter because it measures consistency
  between consecutive agent trajectories.

## Shared Map and Area Sources

Several subscores reuse the same Autoware footprint and lanelet-map helper
outputs. This section defines the area scopes used later in the subscore
equations.

For each trajectory sample, `evaluate_trajectory_footprints()` builds the ego
footprint $P_t$ and, when a route handler is available, computes
`EgoAreaEvaluation`.

The semantic drivable-area search uses the ego footprint bounding box expanded
by `15m`. It collects:

- route-designated road lanelets collected along the selected trajectory
- all road lanelets from the expanded local map search
- road lanelets at the current pose if they intersect the ego footprint
- road-shoulder lanelets from the expanded local map search
- `intersection_area` polygons from the expanded local map search
- `hatched_road_markings` polygons from the expanded local map search
- `parking_lot` polygons from the expanded local map search

The road-border fallback uses `road_border` line strings from the ego footprint
bounding box expanded by `5m`. Road borders are never converted into a broad
drivable polygon; they are used only as local one-corner boundary evidence after
semantic drivable-area containment fails.

The driving-direction local context uses a center-point search, not the full ego
footprint. It searches lanelets and `intersection_area` polygons within a `5m`
point bounding box. Local route-consistent lanelets include route lanelets and
same-direction nearby road or shoulder lanelets, where same direction means the
lanelet yaw differs from the route reference yaw by at most `45deg`. A `0.35m`
margin around these local lanelets is accepted for route-lane containment.

The traffic-light and ego-progress subscores use route-relevant lanelets
collected along the selected trajectory from road lanelets at each trajectory
pose that are also route lanelets. They do not use the 15 m global semantic
drivable-area search.

## NC: No At-Fault Collision

NC checks whether the ego trajectory overlaps a recorded tracked object in an
ego-responsible way.

NC uses the ego footprint $P_t$ from the
[shared map and area sources](#shared-map-and-area-sources). Its lateral-fault
bad-area term reuses the shared `MultipleLanes_t` and `NonDrivableArea_t` flags
from the [semantic drivable-area evaluation](#shared-map-and-area-sources).
Therefore, lateral at-fault classification depends on the 15 m semantic
drivable-area search and the 5 m road-border fallback, while front and stopped
track collisions do not depend on map-area polygons.

For object track $o$, let $O_{t,o}$ be the interpolated object polygon at the
trajectory query time. Candidate contact is:

$$
Contact_{t,o} =
\mathbf{1}[P_t \cap O_{t,o} \ne \emptyset]
$$

Objects whose highest-probability classification is `UNKNOWN` are ignored. This
suppresses short-lived Autoware tracking artifacts.

The ego stopped flag is:

$$
StoppedEgo_t = \mathbf{1}[v_t \le 0.05]
$$

For a tracked object:

$$
StoppedTrack_{t,o} =
\mathbf{1}[\text{object is not an agent}]
\lor
\mathbf{1}[v_{t,o} \le 0.05]
$$

An object is behind ego when the relative angle from ego heading to the
ego-to-object vector exceeds the behind threshold used by the helper:

$$
Behind_{t,o} =
\mathbf{1}[\alpha_{t,o} > 150^\circ]
$$

The front-bumper hit flag is:

$$
FrontHit_{t,o} =
\mathbf{1}[F_t \cap O_{t,o} \ne \emptyset]
$$

where $F_t$ is the front-bumper line segment of the ego footprint.

Collision type is evaluated in this priority order:

1. If $StoppedEgo_t$ is true, then
   $CollisionType_{t,o}=STOPPED\_EGO$.
2. Else if $StoppedTrack_{t,o}$ is true, then
   $CollisionType_{t,o}=STOPPED\_TRACK$.
3. Else if $Behind_{t,o}$ is true, then
   $CollisionType_{t,o}=ACTIVE\_REAR$.
4. Else if $FrontHit_{t,o}$ is true, then
   $CollisionType_{t,o}=ACTIVE\_FRONT$.
5. Otherwise, $CollisionType_{t,o}=ACTIVE\_LATERAL$.

The bad-area flag used for lateral fault judgement is:

$$
BadArea_t = MultipleLanes_t \lor NonDrivableArea_t
$$

The at-fault predicate is:

$$
AtFault_{t,o} =
\mathbf{1}[CollisionType_{t,o}=ACTIVE\_FRONT]
\lor
\mathbf{1}[CollisionType_{t,o}=STOPPED\_TRACK]
\lor
\mathbf{1}[CollisionType_{t,o}=ACTIVE\_LATERAL \land BadArea_t]
$$

Per-contact score is:

$$
\phi_{t,o}=0.0
\quad \text{for an at-fault collision with an agent}
$$

$$
\phi_{t,o}=0.5
\quad \text{for an at-fault collision with a non-agent}
$$

$$
\phi_{t,o}=1.0
\quad \text{otherwise}
$$

The final subscore is the minimum at-fault contact score:

$$
NC = \min_{t,o} \phi_{t,o}
$$

When there is no at-fault contact, $NC=1.0$.

## DAC: Drivable Area Compliance

DAC checks whether all ego footprint corners remain inside the semantic
drivable area, with a conservative road-border fallback.

DAC is the direct consumer of the
[shared semantic drivable-area evaluation](#shared-map-and-area-sources). It
uses the full ego footprint, not only the ego center. At each trajectory sample,
all four ego corners must be accepted by either a semantic drivable polygon or
the conservative road-border fallback.

For each footprint, the semantic drivable union is:

$$
U_t =
RoadLanelets_t
\cup ShoulderLanelets_t
\cup IntersectionAreas_t
\cup HatchedRoadMarkings_t
\cup ParkingLots_t
$$

The road-lanelet set is intentionally not narrowed to only ego-route direction.
Opposite-direction lanelets are still physically road surface for DAC; DDC is
responsible for wrong-way progress.

The detailed area scope is:

- `RoadLanelets_t`: route-designated road lanelets along the trajectory, plus
  all road lanelets from the ego-footprint bounding box expanded by `15m`, plus
  road lanelets at the pose that intersect the footprint
- `ShoulderLanelets_t`: road-shoulder lanelets from the ego-footprint bounding
  box expanded by `15m`
- `IntersectionAreas_t`: `intersection_area` polygons from the ego-footprint
  bounding box expanded by `15m`
- `HatchedRoadMarkings_t`: `hatched_road_markings` polygons from the
  ego-footprint bounding box expanded by `15m`
- `ParkingLots_t`: `parking_lot` polygons from the ego-footprint bounding box
  expanded by `15m`

The semantic corner predicate is:

$$
SemanticDrivable_{t,k} =
\mathbf{1}[X_{t,k} \in U_t]
$$

If a corner is not semantically drivable, the road-border fallback may accept it.
Let:

- $S_{t,k}$ be the closest point from $X_{t,k}$ to the boundary of $U_t$.
- $B_{t,k}$ be the closest point from $X_{t,k}$ to a candidate `road_border`
  line segment.
- $n_{t,k}$ be the unit normal of that border segment.
- $Y^+_{t,k}(\rho)=B_{t,k}+\rho n_{t,k}$.
- $Y^-_{t,k}(\rho)=B_{t,k}-\rho n_{t,k}$.

The implementation probes:

$$
\rho \in \{0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0\}
$$

The road side is valid only when exactly one of $Y^+$ and $Y^-$ is inside
$U_t$. The corner is accepted by the road-border fallback only if all of these
conditions hold:

- exactly one side sample is semantically drivable
- $X_{t,k}$ lies on the same border half-plane as that drivable side
- $X_{t,k}$ lies in the bounded gap from $S_{t,k}$ to $B_{t,k}$
- the vector from $S_{t,k}$ to $B_{t,k}$ is across the border, not along it
- the corner-to-border distance is at most `3.0m`
- the semantic-boundary-to-border distance is at most `4.0m`

In equation form:

$$
RoadBorderFallback_{t,k} =
OneSemanticSide_{t,k}
\land SameRoadSide_{t,k}
\land BoundedGap_{t,k}
\land AcrossBorder_{t,k}
\land d(X_{t,k},B_{t,k}) \le 3.0
\land d(S_{t,k},B_{t,k}) \le 4.0
$$

The final corner predicate is:

$$
CornerDrivable_{t,k} =
SemanticDrivable_{t,k}
\lor
RoadBorderFallback_{t,k}
$$

A trajectory sample is non-drivable if any corner fails:

$$
NonDrivableArea_t =
\mathbf{1}
\left[
\sum_k CornerDrivable_{t,k} < 4
\right]
$$

The final DAC score is:

$$
DAC =
\mathbf{1}
\left[
\forall t,\; NonDrivableArea_t = 0
\right]
$$

## DDC: Driving Direction Compliance

DDC measures wrong-way or oncoming progress over a rolling horizon.

DDC uses the
[driving-direction local context](#shared-map-and-area-sources), not the DAC
semantic drivable union. The context is computed from the ego center point with
a `5m` local lanelet and intersection search. A sample is treated as not on the
route direction when the center point is outside local route-consistent
road/shoulder lanelets, including the `0.35m` lane margin.

For sample $t$, let:

- $g_t$ be positive ego progress since the previous sample.
- $Oncoming_t$ be true when ego is judged to be in oncoming traffic.
- $Intersection_t$ be true when ego is in an intersection context.

The oncoming predicate used by the implementation is:

$$
Oncoming_t = \neg InRouteLanePolygon_t
$$

The route-lane predicate includes exact containment and the lane-margin fallback:

$$
InRouteLanePolygon_t =
InLocalRouteConsistentLane_t
\lor
InLocalRouteConsistentLaneMargin_t
$$

The intersection predicate is true when the ego center is inside a local
`intersection_area` polygon, or inside a route-consistent lanelet tagged as an
intersection lanelet:

$$
Intersection_t =
InIntersectionArea_t
\lor
InIntersectionLanelet_t
$$

Only non-intersection oncoming progress contributes. When the sample is in
oncoming traffic and not in an intersection:

$$
c_t = \max(g_t, 0)
$$

Otherwise:

$$
c_t = 0
$$

The rolling one-second oncoming progress is:

$$
C_t =
\sum_{j:T_t-T_j \le 1.0} c_j
$$

The maximum rolling oncoming progress is:

$$
C_{max} = \max_t C_t
$$

The score is:

$$
DDC = 1.0
\quad \text{if}
\quad C_{max} < 2.0
$$

$$
DDC = 0.5
\quad \text{if}
\quad 2.0 \le C_{max} < 6.0
$$

$$
DDC = 0.0
\quad \text{if}
\quad C_{max} \ge 6.0
$$

## TLC: Traffic Light Compliance

TLC checks whether the ego footprint crosses a relevant stop line while the
traffic signal requires stopping.

TLC does not use red-light pseudo polygons. It uses route-relevant lanelets,
traffic-light regulatory elements, traffic-light group messages, and regulatory
stop-line line strings. Ego interaction is tested by the ego footprint
intersecting the stop line.

The evaluator first collects traffic-light regulatory element groups from route
lanelets relevant to the selected trajectory. For each group $r$:

- $StopLine_r$ is the regulatory stop line.
- $SelectedLanelets_r$ are route lanelets matching the inferred intended turn
  direction when available.
- $Signal_r$ is the matching `TrafficLightGroup` message by regulatory element ID.

Turn intent is inferred from the current turn indicator when it is left or right;
otherwise it is inferred from route lanelet turn-direction attributes when
unambiguous.

Stop-line crossing is:

$$
Cross_{t,r} =
\mathbf{1}[P_t \cap StopLine_r \ne \emptyset]
$$

Stop-required state is:

$$
StopRequired_{t,r} =
\mathbf{1}
\left[
\exists l \in SelectedLanelets_r :
isTrafficSignalStop(l, Signal_r)
\right]
$$

The violation predicate is:

$$
TLCViolation_{t,r} =
Cross_{t,r} \land StopRequired_{t,r}
$$

The final score is:

$$
TLC =
\mathbf{1}
\left[
\forall t,r,\; TLCViolation_{t,r}=0
\right]
$$

If no relevant traffic-light groups exist, the metric is available and returns
$1.0$.

## TTC: Time-to-Collision Within Bound

TTC projects the current ego state to short future offsets and checks overlap
with recorded tracked objects at matching future query times.

TTC uses two different geometry sources. For map context at the current
trajectory sample, it reuses
[shared footprint evaluation flags](#shared-map-and-area-sources):
`MultipleLanes_t`, `NonDrivableArea_t`, and `Intersection_t`. These flags come
from the same 15 m semantic drivable-area search and 5 m road-border fallback
used by [DAC](#dac-drivable-area-compliance), with intersection context enabled.
For collision geometry, TTC builds a separate projected ego footprint at each
offset $\delta$ and compares it against recorded tracked-object polygons; the
projected footprint is not reclassified against map polygons at $t+\delta$.

The checked offsets are:

$$
\delta \in \{0.0, 0.3, 0.6, 0.9\}
$$

For sample $t$ and offset $\delta$, ego pose is projected with the current
velocity:

$$
p_{t,\delta}^{proj} = p_t + \vec{v}_t \delta
$$

The projected ego footprint is:

$$
P_{t,\delta}^{proj} = footprint(p_{t,\delta}^{proj})
$$

The object polygon is interpolated from recorded tracks at time $T_t+\delta$:

$$
O_{t+\delta,o}
$$

Overlap is:

$$
TTCIntersect_{t,\delta,o} =
\mathbf{1}
\left[
P_{t,\delta}^{proj} \cap O_{t+\delta,o} \ne \emptyset
\right]
$$

Stopped ego samples are skipped when:

$$
v_t < 0.005
$$

Previously collided object IDs are skipped, and `UNKNOWN` classified objects are
ignored.

The bad-or-intersection predicate is:

$$
BadOrIntersection_t =
MultipleLanes_t
\lor NonDrivableArea_t
\lor Intersection_t
$$

The relative-position tests are nuPlan-style angle checks:

$$
Ahead_{t,\delta,o} =
\mathbf{1}[\alpha_{t,\delta,o} < 30^\circ]
$$

$$
Behind_{t,\delta,o} =
\mathbf{1}[\alpha_{t,\delta,o} > 150^\circ]
$$

TTC fails when the object is ahead, or when ego is in a bad/intersection area
and the object is not behind:

$$
TTCFail_{t,\delta,o} =
TTCIntersect_{t,\delta,o}
\land
\left(
Ahead_{t,\delta,o}
\lor
(BadOrIntersection_t \land \neg Behind_{t,\delta,o})
\right)
$$

The final score is:

$$
TTC =
\mathbf{1}
\left[
\forall t,\delta,o,\; TTCFail_{t,\delta,o}=0
\right]
$$

## LK: Lane Keeping

LK evaluates sustained route-centerline deviation outside intersections, queues,
and explicit lane-change intent windows.

LK uses route centerline geometry, not the DAC drivable-area polygon union. For
each trajectory sample, the reference lanelet is the closest lanelet within the
route when available; otherwise, a route lanelet at the pose is used. The
intersection exemption uses the same
[5 m center-point driving-direction local context](#shared-map-and-area-sources)
as [DDC](#ddc-driving-direction-compliance).

For each sample:

$$
d_t =
lateralDistanceToCenterline(referenceLanelet_t, p_t)
$$

The over-threshold flag is:

$$
Over_t =
\mathbf{1}[|d_t| > D_{max}]
$$

where the configured default is:

$$
D_{max}=0.5
$$

Lane-change exemption windows are built from left/right turn indicators and
hazard lights. Each active signal interval $[a,b]$ is expanded by:

$$
[a - 1.0,\; b + 1.0]
$$

The queue exemption checks low speed and low progress over a one-second window:

$$
Queue_t =
\mathbf{1}
\left[
v_t \le 1.0
\land
ProgressWindow_t \le 1.5
\right]
$$

After a queue sample, queue-release grace is active for `1.5s`.

The per-sample violation flag is:

$$
LKViolation_t =
Over_t
\land \neg Intersection_t
\land \neg LaneChangeExempt_t
\land \neg Queue_t
\land \neg QueueRelease_t
$$

Let $\mathcal{R}$ be the set of continuous runs where $LKViolation_t$ is true.
The score is:

$$
LK =
\mathbf{1}
\left[
\forall r \in \mathcal{R},\; duration(r) < 2.0
\right]
$$

## HC: History Comfort

HC evaluates whether the padded past-plus-future ego motion stays within NAVSIM
comfort thresholds.

The padded state sequence is:

$$
S^{HC} =
[S^{ego\_past}_{kinematic}; S^{trajectory}_{proposal}]
$$

The past segment is sampled from recorded odometry and acceleration history from
`-1.5s` to `-0.1s` relative to the trajectory stamp. The future segment uses the
selected trajectory from `0.0s` through `4.0s`.

The comfort signal helper computes:

$$
a_x(t),\quad a_y(t),\quad j(t),\quad j_x(t),\quad \dot{\psi}(t),\quad \ddot{\psi}(t)
$$

The six checks are:

$$
-4.05 < a_x(t) < 2.40
$$

$$
|a_y(t)| < 4.89
$$

$$
|j(t)| < 8.37
$$

$$
|j_x(t)| < 4.13
$$

$$
|\dot{\psi}(t)| < 0.95
$$

$$
|\ddot{\psi}(t)| < 1.93
$$

The final score is:

$$
HC =
\mathbf{1}
\left[
\forall t,\;
(-4.05 < a_x(t) < 2.40)
\land
(|a_y(t)| < 4.89)
\land
(|j(t)| < 8.37)
\land
(|j_x(t)| < 4.13)
\land
(|\dot{\psi}(t)| < 0.95)
\land
(|\ddot{\psi}(t)| < 1.93)
\right]
$$

## EC: Extended Comfort

EC compares consecutive planned trajectories and penalizes large dynamic-signal
changes over their time-overlap.

Let:

- $Q^{prev}$ be the previous selected trajectory.
- $Q^{curr}$ be the current selected trajectory.
- $\Delta T$ be the header-stamp interval.
- $\Delta q$ be the trajectory sample interval.

The overlap shift is:

$$
k = round(\Delta T / \Delta q)
$$

The compared overlap sequences are:

$$
S^{prev} = Q^{prev}_{k:}
$$

$$
S^{curr} = Q^{curr}_{:N-k}
$$

For signal:

$$
s \in \{a, j, \dot{\psi}, \ddot{\psi}\}
$$

the RMS discrepancy is:

$$
RMS_s =
\sqrt{
\frac{1}{N_s}
\sum_{n=0}^{N_s-1}
\left(
s_n(S^{curr}) - s_n(S^{prev})
\right)^2
}
$$

The thresholds are:

$$
\tau_a=0.7,\quad
\tau_j=0.5,\quad
\tau_{\dot{\psi}}=0.1,\quad
\tau_{\ddot{\psi}}=0.1
$$

The final score is:

$$
EC =
\mathbf{1}
\left[
RMS_a \le 0.7
\land
RMS_j \le 0.5
\land
RMS_{\dot{\psi}} \le 0.1
\land
RMS_{\ddot{\psi}} \le 0.1
\right]
$$

The first trajectory has no previous trajectory and is therefore unavailable for
EC.

## EP: Ego Progress

EP measures route progress of the selected trajectory. This implementation uses
the current single selected trajectory topic, not a multi-candidate proposal
batch.

EP uses route-relevant lanelets collected along the selected trajectory. It does
not use DAC's semantic drivable-area union, and it does not use a search over
opposite-direction or shoulder lanelets except when those lanelets are part of
the route-relevant centerline context returned by the route handler.

Let:

$$
s_0 = arcLengthOnRoute(p_0)
$$

$$
s_N = arcLengthOnRoute(p_N)
$$

Raw progress is:

$$
G = \max(s_N - s_0, 0)
$$

The multiplicative safety mask is:

$$
M = NC \cdot DAC \cdot DDC \cdot TLC
$$

The denominator used by the single-proposal NAVSIM-style ratio is:

$$
D = G \cdot M
$$

With the progress threshold:

$$
\tau_G = 5.0
$$

the score is:

$$
EP =
clamp(G / D, 0, 1)
\quad \text{if}
\quad D > \tau_G
$$

Otherwise:

$$
EP = 1.0
$$

Because this implementation currently has only one selected trajectory proposal,
EP is effectively always `1.0` when available. Multi-candidate proposal-batch
progress remains a future faithfulness improvement.

## Output Topics

EPDMS metric topics are written under the EPDMS metric namespace:

```text
/open_loop/metrics/epdms/*
```

They are produced for enabled EPDMS metrics. By default, all EPDMS metrics are
enabled unless `open_loop.enabled_metrics` selects a subset.

### Score topics

The main score topics are scalar `std_msgs/msg/Float64` values:

| Topic                                                     | Meaning                                                                                                                     | Example                                                               |
| --------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| `/open_loop/metrics/epdms/no_at_fault_collision`          | `NC` score. Checks whether the ego trajectory has an ego-responsible collision with a recorded object.                      | `0.0` means an at-fault collision happened.                           |
| `/open_loop/metrics/epdms/drivable_area_compliance`       | `DAC` score. Checks whether all ego footprint corners stay in semantic drivable area or accepted road-border fallback area. | `1.0` means no non-drivable-area violation.                           |
| `/open_loop/metrics/epdms/driving_direction_compliance`   | `DDC` score. Checks extended progress on oncoming or wrong-direction route area.                                            | `0.5` means minor oncoming progress.                                  |
| `/open_loop/metrics/epdms/traffic_light_compliance`       | `TLC` score. Checks whether the ego violates relevant red traffic-light stop lines.                                         | `0.0` means a red-light violation.                                    |
| `/open_loop/metrics/epdms/time_to_collision_within_bound` | `TTC` score. Checks short-horizon projected ego collision against recorded objects.                                         | `0.0` means a collision within the TTC bound.                         |
| `/open_loop/metrics/epdms/lane_keeping`                   | `LK` score. Checks prolonged centerline deviation outside exempted lane-change, queue, and intersection conditions.         | `0.0` means a continuous LK violation reached the duration threshold. |
| `/open_loop/metrics/epdms/history_comfort`                | `HC` score. Checks acceleration, jerk, yaw-rate, and yaw-acceleration comfort bounds over past-plus-planned motion.         | `1.0` means all comfort components are inside bounds.                 |
| `/open_loop/metrics/epdms/extended_comfort`               | `EC` score. Checks consistency between consecutive planned trajectories using dynamic-signal RMS differences.               | `0.0` means at least one RMS component exceeds its threshold.         |
| `/open_loop/metrics/epdms/ego_progress`                   | `EP` score. Checks route progress. With the current single selected trajectory, it is effectively `1.0` when available.     | `1.0` means available single-proposal progress passes.                |
| `/open_loop/metrics/epdms/synthetic_epdms_raw`            | Aggregated raw EPDMS from planner subscores.                                                                                | `0.0` means at least one multiplicative gate collapsed the score.     |
| `/open_loop/metrics/epdms/synthetic_epdms_human_filtered` | Aggregated EPDMS after suppressing failures also observed in the human reference, except `EC`.                              | `1.0` means the filtered aggregate fully passes.                      |

### Availability and reason topics

Each main subscore publishes matching availability and reason topics:

```text
/open_loop/metrics/epdms/<subscore>_available
/open_loop/metrics/epdms/<subscore>_reason
```

The availability topic is `std_msgs/msg/Bool`. The reason topic is
`std_msgs/msg/String`.

For example:

```text
/open_loop/metrics/epdms/time_to_collision_within_bound_available = true
/open_loop/metrics/epdms/time_to_collision_within_bound_reason = "collision_within_bound"
```

The published reason strings are implementation diagnostics. Common examples
include:

- `available`
- `collision_within_bound`
- `non_compliant_corner_outside_drivable_area`
- `major_oncoming_progress`
- `minor_oncoming_progress`
- `at_fault_collision_with_agent`
- `at_fault_lateral_collision_with_agent`
- `available_no_relevant_traffic_lights`
- `unavailable_no_previous_trajectory`

The synthetic aggregate topics publish availability flags:

- `/open_loop/metrics/epdms/synthetic_epdms_raw_available`
- `/open_loop/metrics/epdms/synthetic_epdms_human_filtered_available`

Synthetic aggregate topics do not publish reason strings because their state is
derived from the component subscore availability and reason topics.

### EPDMS diagnostic scalar topics

The following scalar topics are not subscores, but explain subscore behavior:

| Topic                                                   | Type                   | Meaning                                                    | Example                                                    |
| ------------------------------------------------------- | ---------------------- | ---------------------------------------------------------- | ---------------------------------------------------------- |
| `/open_loop/metrics/epdms/time_to_at_fault_collision_s` | `std_msgs/msg/Float64` | Earliest at-fault collision time in the selected horizon.  | `2.3` means the first at-fault collision is 2.3 s ahead.   |
| `/open_loop/metrics/epdms/max_oncoming_progress_m`      | `std_msgs/msg/Float64` | Maximum rolling oncoming-direction progress used by `DDC`. | `6.4` means major oncoming progress threshold is exceeded. |

### Debug topics

EPDMS debug topics are written only when `open_loop.debug_topics_enabled` is
`true`. They are intended for Lichtblick validation and use this namespace:

```text
/debug/epdms/*
```

The shared trajectory debug topics are `visualization_msgs/msg/MarkerArray`:

- `/debug/epdms/trajectory/planned_horizon_4s`
- `/debug/epdms/trajectory/gt_horizon_4s`

The subscore debug topics are:

| Subscore | Topic                                               | Type                                 | Meaning                                                        |
| -------- | --------------------------------------------------- | ------------------------------------ | -------------------------------------------------------------- |
| `NC`     | `/debug/epdms/nc/collision_summary`                 | `std_msgs/msg/String`                | JSON summary of collision events.                              |
| `NC`     | `/debug/epdms/nc/ego_footprints`                    | `visualization_msgs/msg/MarkerArray` | Ego footprint horizon used for collision checking.             |
| `NC`     | `/debug/epdms/nc/object_footprints`                 | `visualization_msgs/msg/MarkerArray` | Recorded object polygons used for collision checking.          |
| `NC`     | `/debug/epdms/nc/overlap_areas`                     | `visualization_msgs/msg/MarkerArray` | Ego-object overlap polygons.                                   |
| `DAC`    | `/debug/epdms/dac/violation_summary`                | `std_msgs/msg/String`                | JSON summary of non-drivable-area violations.                  |
| `DAC`    | `/debug/epdms/dac/ego_footprints`                   | `visualization_msgs/msg/MarkerArray` | Ego footprints, with violating footprints highlighted.         |
| `DAC`    | `/debug/epdms/dac/admissible_road_areas`            | `visualization_msgs/msg/MarkerArray` | Road lanelet polygons accepted by DAC.                         |
| `DAC`    | `/debug/epdms/dac/admissible_shoulder_areas`        | `visualization_msgs/msg/MarkerArray` | Road-shoulder polygons accepted by DAC.                        |
| `DAC`    | `/debug/epdms/dac/admissible_intersection_areas`    | `visualization_msgs/msg/MarkerArray` | `intersection_area` polygons accepted by DAC.                  |
| `DAC`    | `/debug/epdms/dac/admissible_hatched_road_markings` | `visualization_msgs/msg/MarkerArray` | Hatched road-marking polygons accepted by DAC.                 |
| `DAC`    | `/debug/epdms/dac/admissible_parking_areas`         | `visualization_msgs/msg/MarkerArray` | Parking-lot polygons accepted by DAC.                          |
| `DAC`    | `/debug/epdms/dac/road_border_lines`                | `visualization_msgs/msg/MarkerArray` | Candidate `road_border` line strings.                          |
| `DAC`    | `/debug/epdms/dac/road_border_side_test_segments`   | `visualization_msgs/msg/MarkerArray` | Side-test segments used to infer the road side of a border.    |
| `DAC`    | `/debug/epdms/dac/road_border_gap_segments`         | `visualization_msgs/msg/MarkerArray` | Semantic-boundary-to-border gap segments.                      |
| `DAC`    | `/debug/epdms/dac/semantic_boundary_points`         | `visualization_msgs/msg/MarkerArray` | Closest semantic drivable-area boundary points.                |
| `DAC`    | `/debug/epdms/dac/road_border_closest_points`       | `visualization_msgs/msg/MarkerArray` | Closest points on candidate road-border segments.              |
| `DAC`    | `/debug/epdms/dac/corner_projection_points`         | `visualization_msgs/msg/MarkerArray` | Failed-corner projections used by the border fallback.         |
| `DAC`    | `/debug/epdms/dac/road_border_plus_samples`         | `visualization_msgs/msg/MarkerArray` | Positive-normal border side-probe samples.                     |
| `DAC`    | `/debug/epdms/dac/road_border_minus_samples`        | `visualization_msgs/msg/MarkerArray` | Negative-normal border side-probe samples.                     |
| `DAC`    | `/debug/epdms/dac/road_border_fallback_corners`     | `visualization_msgs/msg/MarkerArray` | Corners accepted by the road-border fallback.                  |
| `DAC`    | `/debug/epdms/dac/failing_corners`                  | `visualization_msgs/msg/MarkerArray` | Corners still classified as non-drivable.                      |
| `DDC`    | `/debug/epdms/ddc/violation_summary`                | `std_msgs/msg/String`                | JSON summary of oncoming-direction violations.                 |
| `DDC`    | `/debug/epdms/ddc/ego_centers`                      | `visualization_msgs/msg/MarkerArray` | Ego center path used for direction compliance.                 |
| `DDC`    | `/debug/epdms/ddc/oncoming_segments`                | `visualization_msgs/msg/MarkerArray` | Oncoming-progress segments.                                    |
| `DDC`    | `/debug/epdms/ddc/route_lane_polygons`              | `visualization_msgs/msg/MarkerArray` | Route-consistent lane polygons.                                |
| `DDC`    | `/debug/epdms/ddc/intersection_lane_polygons`       | `visualization_msgs/msg/MarkerArray` | Intersection lane polygons that relax oncoming classification. |
| `TLC`    | `/debug/epdms/tlc/violation_summary`                | `std_msgs/msg/String`                | JSON summary of red-light stop-line violations.                |
| `TLC`    | `/debug/epdms/tlc/ego_footprints`                   | `visualization_msgs/msg/MarkerArray` | Ego footprints near stop-line evaluation.                      |
| `TLC`    | `/debug/epdms/tlc/stop_lines`                       | `visualization_msgs/msg/MarkerArray` | Relevant traffic-light stop lines.                             |
| `TTC`    | `/debug/epdms/ttc/violation_summary`                | `std_msgs/msg/String`                | JSON summary of TTC collisions.                                |
| `TTC`    | `/debug/epdms/ttc/ego_footprints`                   | `visualization_msgs/msg/MarkerArray` | Projected ego footprints for TTC offsets.                      |
| `TTC`    | `/debug/epdms/ttc/object_footprints`                | `visualization_msgs/msg/MarkerArray` | Recorded object footprints at TTC query times.                 |
| `TTC`    | `/debug/epdms/ttc/overlap_areas`                    | `visualization_msgs/msg/MarkerArray` | TTC ego-object overlap polygons.                               |
| `LK`     | `/debug/epdms/lk/violation_summary`                 | `std_msgs/msg/String`                | JSON summary of lane-keeping violation runs.                   |
| `LK`     | `/debug/epdms/lk/ego_center_path`                   | `visualization_msgs/msg/MarkerArray` | Ego centerline samples colored by LK state.                    |
| `LK`     | `/debug/epdms/lk/reference_centerlines`             | `visualization_msgs/msg/MarkerArray` | Reference lane centerlines used for lateral deviation.         |
| `HC`     | `/debug/epdms/hc/component_status`                  | `std_msgs/msg/String`                | JSON status of comfort components.                             |
| `HC`     | `/debug/epdms/hc/sample_times`                      | `std_msgs/msg/Float64MultiArray`     | Sample times used by HC.                                       |
| `HC`     | `/debug/epdms/hc/segments`                          | `std_msgs/msg/Float64MultiArray`     | Numeric HC segment/component data for panels.                  |
| `HC`     | `/debug/epdms/hc/horizon_footprints`                | `visualization_msgs/msg/MarkerArray` | Ego footprints colored by comfort status.                      |
| `EC`     | `/debug/epdms/ec/comparison_summary`                | `std_msgs/msg/String`                | JSON summary of consecutive-trajectory comfort comparison.     |
| `EC`     | `/debug/epdms/ec/sample_times`                      | `std_msgs/msg/Float64MultiArray`     | Paired sample times used by EC.                                |
| `EC`     | `/debug/epdms/ec/delta_acceleration`                | `std_msgs/msg/Float64MultiArray`     | Per-sample acceleration differences.                           |
| `EC`     | `/debug/epdms/ec/delta_jerk`                        | `std_msgs/msg/Float64MultiArray`     | Per-sample jerk differences.                                   |
| `EC`     | `/debug/epdms/ec/delta_yaw_rate`                    | `std_msgs/msg/Float64MultiArray`     | Per-sample yaw-rate differences.                               |
| `EC`     | `/debug/epdms/ec/delta_yaw_accel`                   | `std_msgs/msg/Float64MultiArray`     | Per-sample yaw-acceleration differences.                       |
| `EP`     | `/debug/epdms/ep/progress_summary`                  | `std_msgs/msg/String`                | JSON summary of route-progress evaluation.                     |
| `EP`     | `/debug/epdms/ep/route_progress_points`             | `visualization_msgs/msg/MarkerArray` | Progress sample points along the route.                        |
| `EP`     | `/debug/epdms/ep/route_reference`                   | `visualization_msgs/msg/MarkerArray` | Route reference geometry used by EP.                           |

For example, when `NC` is `0.0`, inspect
`/debug/epdms/nc/collision_summary`, then visualize
`/debug/epdms/nc/ego_footprints`, `/debug/epdms/nc/object_footprints`, and
`/debug/epdms/nc/overlap_areas` to identify the at-fault collision geometry.

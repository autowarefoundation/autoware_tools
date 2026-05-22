# EPDMS Metrics

This document summarizes the Autoware-compatible migrated EPDMS metrics written
by the open-loop evaluator. The metric is migrated from the original NAVSIM
EPDMS definition in
<https://github.com/autonomousvision/navsim/blob/main/docs/metrics.md>.

It describes the semantic score first, then the migrated Autoware subscore
equations used by this package.

EPDMS means extended PDM score. NAVSIM EPDMS extends PDMS by adding lane keeping
and extended comfort as weighted metrics, driving-direction compliance and
traffic-light compliance as multiplicative metrics, and false-positive penalty
filtering by human-reference behavior.

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

The EPDMS subscores are:

- [`NC`](#nc-no-at-fault-collision): no at-fault collision
- [`DAC`](#dac-drivable-area-compliance): drivable area compliance
- [`DDC`](#ddc-driving-direction-compliance): driving direction compliance
- [`TLC`](#tlc-traffic-light-compliance): traffic light compliance
- [`TTC`](#ttc-time-to-collision-within-bound): time-to-collision within bound
- [`LK`](#lk-lane-keeping): lane keeping
- [`HC`](#hc-history-comfort): history comfort
- [`EC`](#ec-extended-comfort): extended comfort
- [`EP`](#ep-ego-progress): ego progress

## EPDMS Definition

EPDMS is a synthetic open-loop planning score for one selected trajectory. It
combines multiplicative penalty checks with weighted progress, risk,
lane-keeping, and comfort quality terms.

Let:

$$
\mathcal{M}=\{NC, DAC, DDC, TLC\}
$$

and:

$$
\mathcal{W}=\{EP, TTC, LK, HC, EC\}
$$

For weighted metric $m$, let $w_m$ be its configured weight. The raw score is:

$$
EPDMS_{raw} =
\left(
\prod_{m \in \mathcal{M}} m
\right)
\cdot
\left(
\frac{
\sum_{m \in \mathcal{W}} w_m m
}{
\sum_{m \in \mathcal{W}} w_m
}
\right)
$$

With the migrated Autoware weights:

$$
w_{EP}=5,\quad
w_{TTC}=5,\quad
w_{LK}=2,\quad
w_{HC}=2,\quad
w_{EC}=2
$$

the implemented raw score is:

$$
EPDMS_{raw} =
(NC \cdot DAC \cdot DDC \cdot TLC)
\cdot
\frac{
5EP + 5TTC + 2LK + 2HC + 2EC
}{16}
$$

For the human-filtered score, let $A_m$ be the agent score, $H_m$ be the
human-reference score, and $F_m$ be the filtered score:

$$
F_m =
1
\quad \text{if the agent and human scores are available and}
\quad |H_m| \le 10^{-9}
$$

Otherwise:

$$
F_m = A_m
$$

The filterable metrics are $NC$, $DAC$, $DDC$, $TLC$, $EP$, $TTC$, $LK$, and
$HC$. EC intentionally bypasses the human filter in the migrated synthetic
score:

$$
EPDMS_{HF} =
\left(
\prod_{m \in \{NC, DAC, DDC, TLC\}} F_m
\right)
\cdot
\left(
\frac{
\sum_{m \in \{EP, TTC, LK, HC\}} w_m F_m + w_{EC} EC
}{16}
\right)
$$

Equivalently:

$$
EPDMS_{HF} =
(F_{NC} \cdot F_{DAC} \cdot F_{DDC} \cdot F_{TLC})
\cdot
\frac{
5F_{EP} + 5F_{TTC} + 2F_{LK} + 2F_{HC} + 2EC
}{16}
$$

Here, $M=\prod_{m \in \mathcal{M}}m$ is the multiplicative penalty gate. Any zero
in `NC`, `DAC`, `DDC`, or `TLC` collapses the final score. Note that $NC$ and
$DDC$ are not strictly binary and may return $0.5$ (partial penalty) in certain
scenarios. $W$ is the normalized weighted quality term over `EP`, `TTC`, `LK`,
`HC`, and `EC`. The human filter is a false-positive suppression rule: $F_m=1$
suppresses the agent penalty when the human reference also failed metric $m$,
while $F_m=A_m$ keeps the agent score unchanged.

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

EPDMS metric topics are written under:

```text
/open_loop/metrics/epdms/*
```

The main score topics are:

- `/open_loop/metrics/epdms/no_at_fault_collision`
- `/open_loop/metrics/epdms/drivable_area_compliance`
- `/open_loop/metrics/epdms/driving_direction_compliance`
- `/open_loop/metrics/epdms/traffic_light_compliance`
- `/open_loop/metrics/epdms/time_to_collision_within_bound`
- `/open_loop/metrics/epdms/lane_keeping`
- `/open_loop/metrics/epdms/history_comfort`
- `/open_loop/metrics/epdms/extended_comfort`
- `/open_loop/metrics/epdms/ego_progress`
- `/open_loop/metrics/epdms/synthetic_epdms_raw`
- `/open_loop/metrics/epdms/synthetic_epdms_human_filtered`

Availability and reason topics are published with matching metric-specific
suffixes. Diagnostic raw arrays such as acceleration, jerk, TTC values, lateral
deviation, and travel distance intentionally remain outside the EPDMS metric
namespace.

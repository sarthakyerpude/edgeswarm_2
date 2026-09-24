# Algorithms

## 1. Priority — deterministic by construction

```
P = 0.30·urgency + 0.30·waiting + 0.15·battery + 0.15·distance + 0.10·commitment
```

Every input is a field broadcast in `RobotState`. Nothing local is used. So any
robot can compute any robot's score, and all of them get the same answer.

Three things make that reliable rather than merely likely:

1. **All inputs are broadcast.** No robot uses private knowledge.
2. **Scores are rounded to 6 decimals before any comparison.** Float
   arithmetic can differ in the last bits between an x86 laptop and an ARM
   Jetson. Rounding removes that as a source of disagreement. The rounding is
   load-bearing; do not remove it as "cosmetic".
3. **The order is total:** `(-score, lamport, robot_id)`. Since `robot_id` is
   unique, `wins(a,b)` and `wins(b,a)` can never both be true. Two robots can
   never both believe they won. `test_priority.py::test_total_order_is_strict`
   checks every permutation.

### Why each term

- **urgency** — an urgent task should not queue behind a routine one.
- **waiting** — the anti-starvation term, which is why its weight is
  joint-largest. See the argument below.
- **battery** — *low* battery raises priority. A robot that strands itself
  mid-aisle becomes a permanent obstacle, costing the fleet far more than
  letting it through first.
- **distance** — a robot near its goal finishes sooner and frees the contested
  space sooner.
- **commitment** — a robot already inside the zone gets a small bonus.
  Without it the system thrashes between two robots swapping the lead.

### Starvation argument

A waiting robot's `waiting_time` grows without bound; a moving robot's resets
to 0. The waiting term saturates at 15 s (weight 0.30), so after 15 s a waiting
robot has at least 0.30 from that term alone while any robot that keeps winning
keeps resetting to 0. A hard ceiling at 20 s forces the score to exactly 1.0,
which beats everything. So the maximum wait is bounded by roughly 20 s plus one
zone traversal, regardless of how the other terms are tuned.

This is an argument, not a proof of optimality. Measure actual worst-case waits
in your experiments and report them.

## 2. Conflict detection — four detectors

| Kind | Catches |
|---|---|
| CELL | same grid cell, overlapping time windows |
| SWAP | I go A→B while a peer goes B→A |
| ZONE | both need the same capacity-1 resource |
| TTC | continuous closest-point-of-approach |

**SWAP exists because CELL alone can miss a real collision.** If the two
robots' cell time-windows interleave rather than overlap, no single cell shows
a conflict, yet they meet head-on in the corridor between them.
`test_conflict.py::test_swap_conflict_detected` encodes exactly that case.

TTC is the continuous backstop for anything the discrete layers miss: control
error, odometry drift, an unplanned peer manoeuvre. Derivation:

```
r = p_b − p_a          relative position
v = v_b − v_a          relative velocity
t_cpa = −(r·v)/|v|²    minimises |r + v t|²
t_cpa < 0  ⇒ separating, no future collision
d_cpa = |r + v t_cpa|
collision predicted when d_cpa < D, D = r_sum + margin
first contact: t = (−(r·v) − √((r·v)² − |v|²(|r|² − D²))) / |v|²
```

The `|v|² < 1e-9` guard matters: two robots moving in parallel at identical
velocity have no defined CPA, and without the guard you divide by zero.

## 3. Zone mutual exclusion — Ricart-Agrawala, priority-ordered

**Safety invariant:** a robot enters zone Z only after `granted=true` from
*every* alive peer whose plan also needs Z.

It never enters on "I computed that I have priority". Implicit agreement holds
only if every robot has identical data; one lost packet, one 100 ms delay, and
two robots compute different winners and both enter. That is a collision. So
priority is used for *speed* and the explicit handshake for *safety*.

**A bug this package had, and how it was fixed.** Originally: peer A requests
first, I am not competing so I grant, then I want the zone and happen to
outrank A — a naive comparison made A grant to me too, and both entered. The
fix is an **ownership check**: once I hold grants from every relevant peer I am
the effective owner and always defer. Priority arbitrates *concurrent*
contention; it does not preempt an already-granted lock. Preemption would also
permit livelock, where a high-priority robot repeatedly snatches a zone from
one that already started moving.
`test_zone_protocol.py::test_mutual_exclusion_never_violated` covers both
orderings and six score combinations.

**Lamport clocks**, not wall clocks, order near-simultaneous requests. Three
Jetsons have unknown clock skew, so "who asked first" is not answerable from
timestamps; a logical clock gives consistent causal ordering with no sync at all.

**Three timeout escapes** so a lost grant never wedges the fleet: resend after
1 s (up to 3 times), stop requiring grants from a peer that died, and abandon
the zone entirely after 10 s and reroute.

**Aisles are ONE zone, not a list of cells.** If two robots each reserved half
an aisle from opposite ends they would meet in the middle with neither able to
back out cheaply. One zone per aisle makes that structurally impossible.

## 4. Deadlock — distributed wait-for graph

Each robot broadcasts one field: `waiting_for`. Every robot assembles the same
directed graph from the state messages it already receives, so detection costs
**zero extra messages**.

```
robot_1.waiting_for = robot_2     R1 → R2
robot_2.waiting_for = robot_3     R2 → R3
robot_3.waiting_for = robot_1     R3 → R1     cycle
```

Each node has at most one out-edge, so cycle detection is an O(n) walk with a
visited set — no DFS needed. Victim selection reuses the same total order, so
every robot in the cycle picks the same victim with no negotiation.

A cycle that does **not** contain me returns `None` — that is another robot's
problem, and appointing myself a victim for it would cause needless yielding.

**Two triggers, both required.** The structural detector is fast (~200 ms) and
precise, but trusts peers to report `waiting_for` correctly. If that message is
lost, or a peer crashed while holding a zone, only the 8 s timeout saves you.

**A 1.0 s minimum wait before believing a structural cycle.** Two robots that
both just sent a ZoneRequest briefly point at each other — a textbook 2-cycle
that resolves itself within one round trip. Reacting to it causes needless
rerouting. 1.0 s is ~10 ticks, far longer than a LAN round trip and far shorter
than a real deadlock.

## 5. Task allocation — auction with no auctioneer

```
ANNOUNCE → BID (0.5 s window) → RESOLVE (local, identical) → AWARD → MONITOR
```

Every robot hears every bid and runs the same deterministic selection:
`min(key=lambda r: (-bid[r], r))`. The winner self-assigns and announces. There
is no manager role and no server to lose. The cost is one extra broadcast per
robot per task — negligible at three robots.

```
bid  = 1 / (1 + cost)                          bounded (0,1], higher is better
cost = travel_time + 40·battery_risk + workload + 15·(1 − urgency_match)
```

**Battery is a hard constraint, not a penalty.** Below 15% projected remaining,
the robot does not bid at all. A robot that strands itself becomes a permanent
obstacle — far more expensive than any assignment inefficiency.

**Claim collisions** (both robots think they won, possible only if their bid
sets differed through packet loss) resolve by lower `robot_id` keeping the task.
Cheap, deterministic, rare.

**Dead-assignee recovery:** every robot notices the assignee went silent at the
same moment and the task returns to auction. The `min(alive_ids)` guard is
deduplication, not authority — without it every survivor re-announces at once
and you get a storm. The fleet still works without it, just noisily.

## 6. A* and space-time intent

Plain A*, 8-connected, octile heuristic, corner-cutting forbidden (a diagonal
step between two rack corners clips a rack leg on the real robot).

**Not D\* Lite.** On ~1100 free cells, A* expands 150-400 nodes: about 2-6 ms
on a laptop, 4-8 ms on a Jetson Nano, a few times a minute. D\* Lite's
incremental advantage only pays on much larger maps replanned at high rate, and
costs ~250 lines of subtle code. Measure before optimising.

**Congestion costs are where much of the throughput gain comes from.** Adding
~0.4 soft cost per cell a peer plans to occupy makes robots spread across
aisles *before* any negotiation happens. A robot that never enters the busy
aisle never has to argue about it.

`build_intent` converts a geometric path into time windows, widened by
`safety_margin_s` (default 0.5 s) at both ends to absorb clock skew, control
lag and prediction error. Tune it empirically and **report the tuning** — that
detail is what separates a serious project from a demo.

## Scope note

This package produces a `MotionPermit` (GO / SLOW / STOP / YIELD / REROUTE plus
a continuous `speed_scale`). It does **not** command `cmd_vel` by default;
`amr_navigation` owns the actuator. The A* here exists to generate the intent
to broadcast and to evaluate reroutes, not to drive the robot.

`speed_scale` being continuous rather than a GO/STOP boolean is deliberate. A
robot that slows from 0.5 to 0.3 m/s and glides through an intersection two
seconds later loses about 2 s; one that stops fully and re-accelerates under a
0.5 m/s² limit loses about 6 s. Across 30 tasks and 3 robots that difference
compounds.

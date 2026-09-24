# DDS in this project

## The layer stack

```
your node code
      |  rclpy
      v
   rcl / rmw          <- the middleware abstraction
      |
      v
  Cyclone DDS         <- discovery, serialisation, QoS, delivery
      |
      v
  UDP / RTPS on the wire
```

## What DDS does and does NOT do

| Layer | Coordinates the fleet? |
|---|---|
| DDS | **No.** It moves bytes and finds peers. It has no concept of a robot, an intersection or a priority. |
| `amr_fleet/core` | **Yes. Exclusively.** |

Saying "DDS handles coordination" is like saying TCP handles a website's
business logic. DDS makes the *communication* decentralised (no broker); our
algorithms make the *decisions* decentralised (no coordinator). Two different
claims, both needed. State them separately when you present.

## Object model

| DDS concept | ROS 2 equivalent | Here |
|---|---|---|
| DomainParticipant | one per **process** | `fleet_agent` x3 + bridge + monitor |
| Domain | `ROS_DOMAIN_ID` | 42 |
| Topic | ROS topic (`rt/` prefix on the wire) | `rt/fleet/robot_state` |
| DataWriter | publisher | robot_1's state publisher |
| DataReader | subscription | robot_2's peer subscription |
| QoS | `QoSProfile` | `nodes/qos_profiles.py` |

Since Foxy the RMW creates **one participant per process, not per node**. That
is why all twelve logical modules are composed into one `fleet_agent` process
per robot: twelve separate processes would mean twelve participants and
O(N^2) discovery traffic for no benefit.

## Discovery

**SPDP** (participant discovery): each participant periodically multicasts an
announcement to `239.255.0.1` on a port derived from the domain ID. Peers reply
directly by unicast.

**SEDP** (endpoint discovery): the two then exchange, over unicast, their full
publisher/subscriber lists with topic names, types and QoS. Each side runs the
compatibility check locally.

**After that, all data is unicast peer-to-peer.** Multicast is used only to
find each other.

```
t=0.0  robot_1 starts, creates a DomainParticipant
t=0.0  multicasts SPDP to 239.255.0.1:17900   (7400 + 250*42 + 0)
t=0.1  robot_2 hears it, replies by unicast
t=0.2  SEDP: "I write rt/fleet/robot_state, type RobotState, BEST_EFFORT..."
t=0.2  both run the RxO check locally -> match
t=0.3  robot_1 publishes; it arrives directly at robot_2
```

## `ROS_DOMAIN_ID`

A domain is a network isolation boundary. Participants in different domains
**cannot** see each other, whatever the topic names. The mechanism is port
arithmetic:

```
discovery multicast = 7400 + 250*domain
discovery unicast   = 7400 + 250*domain + 10 + 2*participant_index
user multicast      = 7400 + 250*domain + 1
user unicast        = 7400 + 250*domain + 11 + 2*participant_index

domain 42 -> discovery multicast on 17900
```

| Rule | Reason |
|---|---|
| Use 0-101 on Linux | above ~101 the computed ports collide with the ephemeral range |
| **Never 0** | it is the default; every tutorial and every classmate's laptop is there |
| Same domain on all three robots | different domains = they never see each other |
| Different domain for a parallel run | isolate a baseline run from a proposed run |

**Never hard-code it in source.** It is deployment configuration:

```bash
export ROS_DOMAIN_ID=42
```

Check who else is around before committing to a value:
```bash
ROS_DOMAIN_ID=0  ros2 node list    # on a shared network, often surprising
ROS_DOMAIN_ID=42 ros2 node list    # should be only yours
```

## Choosing Cyclone DDS

Jazzy's default is `rmw_fastrtps_cpp`. We recommend Cyclone:

| | Cyclone | Fast DDS |
|---|---|---|
| Memory per participant | ~8-12 MB | ~15-25 MB |
| Wi-Fi predictability | more consistent | more tuning needed |
| Multicast-free config | simple `<Peers>` block | more verbose |
| Single-PC throughput | good | better (shared memory) |

On one PC Fast DDS is genuinely faster thanks to its shared-memory transport.
Our messages are small (~120 B state) at low rates (~30 msg/s/robot), so we are
nowhere near that regime; Wi-Fi behaviour and one config file to learn matter
more for a project heading to three Jetsons.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Every terminal must export it.** A node started with Fast DDS cannot reliably
talk to one started with Cyclone, even on the same machine and domain. The
symptom — "two of my three robots see each other" — is maddening. Put it in
`~/.bashrc`, not in individual terminals.

**Explicitly rejected: Fast DDS Discovery Server mode.** It solves real scaling
problems by routing discovery through a dedicated server process. For this
project it would be self-defeating: a judge asking "what is that server?" would
have found a central point of failure in a project whose thesis is that there
isn't one.

## Wi-Fi configuration

Many access points throttle or drop multicast, and many enable AP isolation
which blocks client-to-client traffic entirely. Since multicast is used only
for discovery, disabling it and listing peers restores discovery without
changing how data flows. See `config/cyclonedds_wifi.xml`.

```bash
export CYCLONEDDS_URI=file:///<ws>/install/amr_description/share/amr_description/config/cyclonedds_wifi.xml
```

**The peer list is not centralisation.** It is a phone book, not a switchboard:
it says where to send discovery announcements. No listed address has authority
and no data passes through one.

ROS 2 Jazzy also offers RMW-independent alternatives worth knowing:
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET   # OFF | LOCALHOST | SUBNET | SYSTEM_DEFAULT
export ROS_STATIC_PEERS="192.168.50.11;192.168.50.12;192.168.50.13"
```
(Note `ROS_LOCALHOST_ONLY` is deprecated in Jazzy in favour of these.)

## QoS: the rule that silently breaks topics

DDS matches endpoints with a Request/Offered contract. If the publisher's
**offer** is weaker than the subscriber's **request**, they do not connect —
and ROS 2 reports no error. The topic just shows zero subscribers.

| Policy | Compatible when |
|---|---|
| Reliability | offered RELIABLE >= requested BEST_EFFORT |
| Durability | offered TRANSIENT_LOCAL >= requested VOLATILE |
| Deadline | offered period <= requested period |
| Liveliness | offered lease <= requested lease |

```bash
ros2 topic info /fleet/robot_state --verbose
# Publisher count > 0 but Subscription count = 0 is the tell.
```

This is why all profiles live in `nodes/qos_profiles.py` and are imported.
Never write a `QoSProfile` inline.

## Why state is BEST_EFFORT

A deliberate engineering choice, not laziness. With RELIABLE, a lost 10 Hz pose
triggers retransmission. By the time it arrives, two newer poses exist and the
retransmitted one is garbage. On congested Wi-Fi that traffic delays the
RELIABLE zone-grant messages that actually matter. You would be spending
bandwidth to deliver data you will discard.

Coordination messages are RELIABLE precisely because a dropped ZoneGrant makes
a robot wait for a message that will never come.

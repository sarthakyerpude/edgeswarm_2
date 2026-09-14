"""
Shared QoS profiles. IMPORT THESE - never write a QoSProfile inline.

WHY THIS FILE EXISTS
DDS matches endpoints using a Request/Offered contract. If a publisher OFFERS
a weaker QoS than a subscriber REQUESTS, they do not connect at all - and
ROS 2 reports no error. The topic simply shows zero subscribers. Inline
profiles drift apart over time and produce exactly that silent failure, which
is the single most common "my node receives nothing" bug.

COMPATIBILITY RULES (offered must be at least as strong as requested)
    Reliability : RELIABLE  >= BEST_EFFORT
    Durability  : TRANSIENT_LOCAL >= VOLATILE
    Deadline    : offered period <= requested period
    Liveliness  : offered lease  <= requested lease

Diagnose a mismatch with:
    ros2 topic info /fleet/robot_state --verbose
"""
from rclpy.duration import Duration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, LivelinessPolicy,
                       QoSProfile, ReliabilityPolicy)

# ---------------------------------------------------------------- SENSOR ----
# BEST_EFFORT + KEEP_LAST(1): only the newest scan matters. Retransmitting a
# LiDAR scan that is already 100 ms stale wastes bandwidth to deliver data we
# would immediately discard.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ----------------------------------------------------------------- STATE ----
# The 10 Hz peer broadcast. Same reasoning as sensors, plus:
#
# LIVELINESS (AUTOMATIC, 2 s lease): the middleware tells us when a publishing
# PARTICIPANT dies. Faster and more definitive than waiting for our own
# heartbeat timeout. We still keep the application-level timeout in
# core/peers.py, because DDS liveliness cannot detect a robot that is alive and
# publishing but has a failed sensor - and it does not exist at all on the
# raw-UDP fallback transport.
#
# DEADLINE (200 ms): a missed-deadline callback is a second, cheap early signal
# that a peer has gone quiet.
#
# LIFESPAN (500 ms): a pose older than half a second is auto-discarded by the
# middleware rather than delivered stale after a network stall.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    deadline=Duration(seconds=0, nanoseconds=200_000_000),
    lifespan=Duration(seconds=0, nanoseconds=500_000_000),
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=2),
)

# ------------------------------------------------------------ COORDINATION --
# RELIABLE: a dropped ZoneGrant causes a robot to wait for a message that will
# never arrive. These MUST be delivered.
#
# TRANSIENT_LOCAL: a robot that joins late (or restarts) immediately receives
# the last value on each coordination topic instead of driving blind until the
# next event.
#
# KEEP_LAST(10): enough to absorb a burst during a Wi-Fi stall. NOT KEEP_ALL -
# that is unbounded memory, which on a 4 GB Jetson is a real risk.
COORD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ------------------------------------------------------------- MAP UPDATE ---
# Deeper history (50) than COORD because several blockages can be reported in
# a burst and a late joiner needs the full current picture of the warehouse,
# not just the most recent event.
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)

# ---------------------------------------------------------------- DEFAULT ---
# For diagnostics a dashboard may or may not be listening to.
DIAGNOSTIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

__all__ = ["SENSOR_QOS", "STATE_QOS", "COORD_QOS", "MAP_QOS", "DIAGNOSTIC_QOS"]

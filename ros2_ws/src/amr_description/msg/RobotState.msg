# =============================================================================
# RobotState.msg
# =============================================================================
# Published by EVERY AMR on the SHARED topic /fleet/robot_state at 10 Hz.
#
# This single message serves two purposes at once:
#   1. State sharing  - peers learn where I am and what I am doing.
#   2. Heartbeat      - if this stops arriving, peers mark me DEAD.
#
# Every field here is an INPUT to some peer's decision. Nothing is decorative.
# If a peer cannot use a field to make a decision, it should not be here.
# =============================================================================

# stamp   = when this state was captured (NOT when it was sent)
# frame_id= "map" - all robots MUST agree on this frame or positions are
#           meaningless to each other.
std_msgs/Header header

# Stable identity, e.g. "robot_1". Set from the robot_id ROS parameter.
# Used as the deterministic tie-breaker in every priority comparison,
# so it MUST be unique and MUST NOT change at runtime.
string robot_id

# Monotonic counter, incremented once per published message.
# Peers detect packet loss by looking for gaps, and reject out-of-order
# UDP delivery by discarding any message whose seq <= the last seen seq.
uint32 seq

# ---------------------------------------------------------------- pose ------
# 2D pose in the map frame. We use Pose2D rather than PoseStamped because
# AMRs are planar: a full 3D pose + quaternion would be 7 floats of which
# 4 are always constant, tripling the message size for no information.
geometry_msgs/Pose2D pose

# ------------------------------------------------------------ velocity ------
# Current body-frame velocity. Peers use linear.x and angular.z to
# extrapolate my short-horizon future position for time-to-collision maths.
geometry_msgs/Twist velocity

# -------------------------------------------------------------- status ------
uint8 STATUS_IDLE=0        # no task, not moving
uint8 STATUS_MOVING=1      # executing a path
uint8 STATUS_WAITING=2     # blocked, holding position, wants to move
uint8 STATUS_YIELDING=3    # actively giving way (reversing / stepping aside)
uint8 STATUS_CHARGING=4    # at a charger
uint8 STATUS_FAULT=5       # unsafe - peers must treat me as a static obstacle
uint8 status

# ---------------------------------------------------------------- task ------
string task_id             # "" when idle
uint8 task_priority        # 0 = low ... 3 = urgent. Feeds the priority score.

# --------------------------------------------- coordination inputs ----------
# These four fields are what let EVERY robot compute EVERY robot's priority
# score locally and arrive at the same answer. They are the reason the system
# needs no central arbiter.

float32 battery_pct        # 0..100. Low battery RAISES priority: a robot that
                           # strands itself becomes a permanent obstacle.

float32 waiting_time       # seconds continuously in STATUS_WAITING.
                           # Grows without bound -> this is the anti-starvation
                           # term. Resets to 0 when the robot gets to move.

string waiting_for         # robot_id I am currently blocked by, "" if none.
                           # Every robot collects these into the same wait-for
                           # graph and runs cycle detection locally.

float32 priority_score     # MY OWN computed score. Peers recompute it from the
                           # fields above; this is a cross-check, not the
                           # source of truth. A mismatch means divergent data.

# ------------------------------------------------------------- liveness -----
# Explicit "I am healthy" flag. Distinct from "a message arrived": a robot can
# be publishing while knowing its LiDAR has failed. Peers treat alive=false as
# a static obstacle rather than a moving agent.
bool alive

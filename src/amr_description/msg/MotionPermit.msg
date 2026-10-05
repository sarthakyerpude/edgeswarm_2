# =============================================================================
# MotionPermit.msg   ->  <robot_ns>/motion_permit
# =============================================================================
# THE OUTPUT OF THIS ENTIRE PACKAGE.
#
# This is the clean seam between fleet coordination (this package) and motion
# control (amr_navigation / Nav2). We publish a DECISION, not a velocity.
# The navigation layer stays the only owner of cmd_vel, so no package fights
# another for the actuator.
#
# WHY speed_scale is continuous instead of a GO/STOP boolean:
# Traditional stop-and-wait binarises everything. A robot that slows from
# 0.5 to 0.3 m/s and glides through an intersection two seconds later loses
# about 2 s. A robot that stops fully and re-accelerates under a 0.5 m/s^2
# limit loses about 6 s. Across 30 tasks and 3 robots, that difference is
# where most of the throughput improvement comes from.
# =============================================================================

std_msgs/Header header
string robot_id

uint8 ACTION_GO=0          # proceed normally,            speed_scale = 1.0
uint8 ACTION_SLOW=1        # proceed reduced,             0 < speed_scale < 1
uint8 ACTION_STOP=2        # hold position,               speed_scale = 0.0
uint8 ACTION_YIELD=3       # actively give way (deadlock victim)
uint8 ACTION_REROUTE=4     # current path abandoned, replan requested
uint8 action

float32 speed_scale        # 0.0 .. 1.0 multiplier on the nominal speed

string reason              # human-readable, e.g.
                           # "zone inter_X1 held by robot_2 until t+2.4s"
string blocking_robot      # "" if none
string zone_id             # "" if not zone-related
bool   deadlock_detected
string[] deadlock_cycle    # robot_ids forming the cycle, in order

# =============================================================================
# Intent.msg
# =============================================================================
# Published on /fleet/intent when the plan changes, plus a 1 Hz keepalive.
#
# THE CENTRAL IDEA OF THE WHOLE PROJECT:
# RobotState says "where I am". Intent says "where I WILL BE, and WHEN".
#
# Conflict prediction is impossible from position alone. Constant-velocity
# extrapolation is meaningless beyond ~2 s because the robot will turn. So
# each robot publishes its own time-parameterised plan and peers check for
# overlap in space AND time.
# =============================================================================

std_msgs/Header header
string robot_id
uint32 seq

# ------------------------------------------------------- the space-time path -
# Parallel arrays, all the same length N. Element i describes one grid cell.
#
# Flat arrays are used instead of an array of a custom Cell type because
# rosidl serialises them far more cheaply and they are trivial to zip() in
# Python:  for r, c, ti, to in zip(path_rows, path_cols, t_enter, t_exit)
#
# t_enter/t_exit are ABSOLUTE epoch seconds, already widened by a safety
# margin at generation time to absorb clock skew and control lag.
int32[]   path_rows        # grid row of each cell, in traversal order
int32[]   path_cols        # grid column of each cell
float64[] t_enter          # predicted entry time into that cell
float64[] t_exit           # predicted exit time from that cell

# ---------------------------------------------------------- named resources -
# Capacity-1 shared resources this path needs (intersections, single-lane
# aisles, choke points), in the order they will be entered.
#
# Reserving a WHOLE narrow aisle rather than its individual cells is what makes
# head-on deadlock inside an aisle structurally impossible - two robots can
# never each own half of it.
string[] zones_needed

# Index into zones_needed of the zone currently being negotiated, or -1.
int32 active_zone_index

# ----------------------------------------------------------------- context --
geometry_msgs/Pose2D goal  # final destination, for dashboards and for peers
                           # estimating how much longer I will be in the way
string task_id
uint8   task_priority
float32 priority_score     # duplicated from RobotState so a peer that has an
                           # Intent but not yet a State can still rank me

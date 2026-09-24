# =============================================================================
# Conflict.msg   ->  <robot_ns>/conflicts   (per-robot, diagnostic)
# =============================================================================
# NOT part of the coordination protocol. Robots do not act on each other's
# Conflict messages - each detects conflicts itself from State + Intent.
#
# This exists so the dashboard, RViz and your experiment logger can SEE what
# each robot detected and when. It is what turns "zero collisions" from an
# assertion into a measurement.
# =============================================================================

std_msgs/Header header
string robot_id            # who detected it
string peer_id             # who it is with

uint8 KIND_CELL=0          # same grid cell, overlapping time windows
uint8 KIND_SWAP=1          # head-on exchange: I go A->B while peer goes B->A
uint8 KIND_ZONE=2          # both need the same capacity-1 zone
uint8 KIND_TTC=3           # continuous time-to-collision below threshold
uint8 kind

int32   cell_row           # -1 if not cell-specific
int32   cell_col
string  zone_id            # "" if not zone-specific
float64 t_conflict         # predicted epoch time of the conflict
float32 severity           # 1/max(0.1, time_until_conflict) - bigger = sooner
float32 my_priority
float32 peer_priority
bool    i_have_priority    # result of the deterministic comparison

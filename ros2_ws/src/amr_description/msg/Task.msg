# =============================================================================
# Task.msg   ->  /fleet/task_announce
# =============================================================================
# A unit of warehouse work: collect from a pickup cell, deliver to a dropoff.
#
# IMPORTANT: whatever publishes this is an EVENT SOURCE, not a decision maker.
# It says WHAT needs doing. It never says WHO does it. Any robot (or an
# external order feed) may announce a task. The assignment is decided by the
# auction in Task / TaskBid / TaskAward, with no auctioneer.
# =============================================================================

std_msgs/Header header
string  task_id            # globally unique, e.g. "T-0047"
int32   pickup_row
int32   pickup_col
int32   dropoff_row
int32   dropoff_col
uint8   priority           # 0 low .. 3 urgent
float64 created_at         # epoch seconds
float64 deadline           # epoch seconds; 0.0 = no deadline
string  announcer_id       # who put it on the bus (provenance only, no authority)

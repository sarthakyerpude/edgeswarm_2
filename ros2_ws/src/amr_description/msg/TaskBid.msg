# =============================================================================
# TaskBid.msg   ->  /fleet/task_bid
# =============================================================================
# Every eligible robot broadcasts its bid. Every robot HEARS every bid.
# After the bid window closes, all robots independently sort the same set and
# reach the same winner. Nobody runs the auction.
#
# bid = 1 / (1 + cost)   ->  HIGHER IS BETTER, and the value is bounded (0,1],
# which keeps float comparison well behaved.
# =============================================================================

std_msgs/Header header
string  robot_id
string  task_id
float64 bid                # higher = better fit. See amr_fleet/core/tasks.py
float64 est_completion_s   # my estimate, for dashboards and deadline checks
float32 battery_after_pct  # projected battery at completion. A robot bids only
                           # if this stays above the hard floor - stranding
                           # yourself mid-aisle is worse than any inefficiency.
uint32  seq

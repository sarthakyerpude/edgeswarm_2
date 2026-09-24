# =============================================================================
# TaskAward.msg   ->  /fleet/task_award
# =============================================================================
# The winner CONFIRMS it is taking the task. This is an announcement, not a
# request for permission - the winner already knows it won, because it ran the
# same deterministic selection everyone else ran.
#
# Collision case: if two robots both claim the same task (possible only if
# their bid sets differed because of packet loss), the one with the
# lexicographically LOWER robot_id keeps it and the other immediately releases
# and re-enters the pool. Cheap, deterministic, and rare in practice.
# =============================================================================

std_msgs/Header header
string  task_id
string  winner_id
float64 winning_bid
uint8   num_bidders        # how many bids the winner saw. If two robots report
                           # different values for the same task, that is direct
                           # evidence of message loss - useful in your results.
float64 stamp

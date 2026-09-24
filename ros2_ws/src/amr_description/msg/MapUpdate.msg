# =============================================================================
# MapUpdate.msg   ->  /fleet/map_update
# =============================================================================
# Broadcast when a robot's LiDAR finds (or stops finding) an obstacle that is
# not in the static map - a dropped pallet, a parked forklift, a blocked aisle.
#
# THE VALUE: peers learn about the blockage BEFORE they drive to it and
# discover it themselves. A robot two aisles away reroutes immediately. This
# is the single clearest demonstration of why sharing beats independent
# operation, and it needs no map server.
# =============================================================================

std_msgs/Header header
string reporter_id

# Newly blocked / newly cleared cells (parallel row/col arrays).
int32[] blocked_rows
int32[] blocked_cols
int32[] cleared_rows
int32[] cleared_cols

# 0..1, derived from how many consecutive scans confirmed the return.
# Receivers scale it down for distant reporters (a reading from 8 m away is
# less trustworthy than one from 1 m) and require either high confidence OR
# corroboration from a second robot before hard-blocking a cell. One robot
# with a noisy scan must not be able to wall off an aisle for the whole fleet.
float32 confidence

# Absolute epoch time after which this observation is STALE and must be
# discarded by every receiver.
#
# WHY EXPIRY IS MANDATORY: without it, a forklift that pauses in an aisle
# blocks that aisle permanently in all three robots' maps for the rest of the
# run, and throughput collapses. Expiry forces periodic re-verification.
# Typical value: now + 20 s.
float64 expiry

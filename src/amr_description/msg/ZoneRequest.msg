# =============================================================================
# ZoneRequest.msg   ->  /fleet/zone_request
# =============================================================================
# WHY THIS EXISTS AT ALL:
#
# A naive design says "everyone has everyone's data, so everyone computes the
# same winner, so no handshake is needed." That is true ONLY if every robot
# holds identical data. With one lost packet, a 100 ms delay, or a peer that
# just updated its waiting_time, two robots can compute DIFFERENT winners at
# the same instant and both enter the intersection. That is a collision.
#
# So: implicit priority is used for SPEED, and this explicit handshake is used
# for SAFETY. A robot never enters a capacity-1 zone on an assumption.
#
# The protocol is Ricart-Agrawala distributed mutual exclusion, with the
# ordering key changed from pure Lamport timestamp to (priority, robot_id).
# =============================================================================

std_msgs/Header header
string robot_id            # requester
string zone_id             # e.g. "inter_X1", "aisle_A2"

# Lamport logical clock. Updated as lc = max(lc, received_lc) + 1 on every
# coordination message.
#
# WHY a logical clock when we already have wall timestamps:
# wall clocks on three separate Jetsons are skewed by an unknown amount, so
# "who asked first" is not answerable from timestamps. A Lamport clock gives a
# consistent causal ordering with no clock sync at all. It is the SECONDARY
# ordering key here - priority decides first, Lamport breaks near-simultaneous
# requests, robot_id breaks the remaining ties. The result is a TOTAL order
# that every robot computes identically.
uint64 lamport_ts

float32 priority_score     # my score at request time - the primary ordering key
float64 t_enter_est        # when I expect to enter  (epoch seconds)
float64 t_exit_est         # when I expect to leave  (epoch seconds)
uint32  req_id             # request ID randomized per process start, then
                           # incremented; echoed in ZoneGrant so retained or
                           # late replies from earlier process epochs are ignored

# =============================================================================
# ZoneGrant.msg   ->  /fleet/zone_grant
# =============================================================================
# A reply to one ZoneRequest from one peer.
#
# The requester may enter the zone ONLY after receiving granted=true from
# EVERY peer that is (a) currently alive, and (b) needs the same zone within
# the planning horizon. Peers that do not want the zone grant immediately, so
# an uncontested zone costs one round trip (~10 ms), not a negotiation.
# =============================================================================

std_msgs/Header header
string granter_id          # who is replying
string requester_id        # who they are replying to
string zone_id
uint32 req_id              # MUST echo the ZoneRequest.req_id

# true  = "go ahead, I am not competing / you outrank me"
# false = "deferred - I outrank you or I hold the zone. I will send you a
#          grant when I release it."
#
# false is NOT a permanent denial. It is a promise to reply later. This is
# what prevents the requester from spinning: it waits, and the grant arrives.
bool granted

uint64 lamport_ts          # granter's clock, so the requester can advance its own

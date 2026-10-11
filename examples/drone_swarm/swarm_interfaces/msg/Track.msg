# A coordinator track: drone reports fused across the swarm.
string id
geometry_msgs/Point position
string[] classes
float64[] probabilities
uint32 sightings
float64 best_altitude
string status            # unresolved | inspecting | confirmed_survivor | confirmed_non_survivor | dismissed
string assigned          # drone dispatched to inspect it ("" = none)
string[] reporters

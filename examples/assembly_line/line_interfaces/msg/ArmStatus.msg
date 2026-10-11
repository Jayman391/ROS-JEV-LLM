# One arm, as its own sensors see it (SYSTEM_MODEL §4). Published by line_sim on /<arm>/status.
# NaN readings: not applicable to the pending op, or a sensor dropout (see `dropout`).
std_msgs/Header header
string arm
int32 station
string state              # idle | working | coop_wait | regrip | tool_change | jammed
int32 part_id             # the part with an op pending for (or being worked by) this arm; -1 none
int32 op_index
string op_kind
string role               # solo | primary (fasten: screws) | partner (fasten: holds)
string partner            # the other fasten arm
string partner_state      # the partner's state; '' when solo
float64 coop_wait_s       # how long this arm has waited for its partner
float64 grip              # 0..1, this arm's grip on the part (gripping ops)
float64 partner_grip      # fasten: the holding arm's grip
float64 pose_mm           # part alignment offset
float64 wear_est          # 0..1 tool wear estimate
float64 current_a         # joint current; spikes while jammed
bool dropout
float64 jam_clear_est_s   # jammed: estimated time to clear
int32 regrips             # on the pending op
int32 queue               # parts whose next op is routed to this arm
int32 ops_done
int32 ops_since_change
int32 jams

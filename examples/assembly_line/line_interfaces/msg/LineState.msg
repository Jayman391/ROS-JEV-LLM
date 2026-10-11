# The line as the supervisor sees it (no hidden quality). /line/state, 2 Hz.
std_msgs/Header header
float64 t                 # sim s since start
PartState[] parts         # every part on the line
int32[] buffer            # pallets queued or in transit per stop: S1..S4, QA
int32 buffer_cap
bool feeder_free          # a release would be accepted
string[] products
int32[] released
int32[] shipped
int32[] scrapped
int32[] quarantined

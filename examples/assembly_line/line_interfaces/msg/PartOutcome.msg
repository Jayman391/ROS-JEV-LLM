# Ground truth for a part leaving the line. /line/outcomes, read only by line_scorer.
std_msgs/Header header
int32 part_id
string product
string outcome            # shipped | scrapped | quarantined
bool defective            # some op has quality < q_min (a rework's redo replaces the original)
string[] op_kinds
string[] op_arms
float64[] quality         # true quality per recipe op
int32 reworks
float64 released_s
float64 done_s

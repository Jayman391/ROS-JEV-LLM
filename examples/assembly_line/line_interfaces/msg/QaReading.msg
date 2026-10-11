# A finished part at QA: one noisy reading per op (SYSTEM_MODEL §4). /qa/reading.
std_msgs/Header header
int32 part_id
string product
int32 reworks
string[] op_kinds
string[] op_arms
float64[] readings        # ~ quality; < 0.4 reads as a missing feature

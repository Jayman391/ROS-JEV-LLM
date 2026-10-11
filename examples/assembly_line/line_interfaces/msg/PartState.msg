int32 id
string product
string where              # feeder | S1..S4 | QA | lane | transit
string state              # queued | slot | transit | lane
int32 next_op             # index into route; == n_ops when every op is done
int32 n_ops
string next_kind
string[] route            # arm per op ("arm_d+arm_c" for fasten); '' = unroutable
int32 reworks
float64 released_s

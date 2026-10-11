int32 id
string product
string where              # feeder | S1..S4 | QA | lane | transit
string state              # queued | slot | transit | lane
int32 stop                # where it is or is heading: 0 feeder/lane, 1..n stations, n+1 QA
int32 next_op             # index into route; == n_ops when every op is done
int32 n_ops
string next_kind
string[] kinds            # op kind per op (recipe, then rework redos)
string[] route            # arm per op ("arm_d+arm_c" for fasten); '' = unroutable
int32 reworks
float64 released_s

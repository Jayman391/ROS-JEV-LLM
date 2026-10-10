# The engine's decision for one question set at one point in the commitment cycle.
uint8 INTERFACE_VERSION=1

string PROVISIONAL=PROVISIONAL
string REVISED=REVISED
string KEPT=KEPT
string COMMITTED=COMMITTED
string FALLBACK=FALLBACK
string FORCED=FORCED

uint64 decision_id
uint64 generation
string phase                     # one of the constants above
jev_ai_msgs/ClassRef[] selected
jev_ai_msgs/Distribution[] fused
diagnostic_msgs/KeyValue[] values    # expectation/threshold outputs, e.g. urgency.level_mean=0.62
string llm_params                # JSON, clamped to catalog bounds
string rationale

# Full trace of one decision; published by engines, persisted by decision_logger.
uint8 INTERFACE_VERSION=1

std_msgs/Header header           # node clock (sim time ok)
builtin_interfaces/Time wall_stamp
string run_id
string agent
string group
string question_set
uint64 decision_id
uint64 generation
string phase                     # Decision.PROVISIONAL ... Decision.FORCED
string[] triggers
string subject
diagnostic_msgs/KeyValue[] state
jev_ai_msgs/QuestionTrace[] questions
string llm_params
string llm_rationale
string[] llm_flags
string catalog_hash
string policy_hash
string jev_model
string llm_model
float64 jev_latency_ms
float64 llm_latency_ms
uint32 jev_input_tokens
uint32 llm_input_tokens
uint32 llm_output_tokens
float64 cost_usd

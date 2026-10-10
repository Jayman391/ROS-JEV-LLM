# Worker -> supervisor (hierarchy module).
uint8 INTERFACE_VERSION=1

std_msgs/Header header
string agent
string kind
string subject
float64 priority
jev_ai_msgs/Distribution[] beliefs
string payload                   # JSON, app-defined

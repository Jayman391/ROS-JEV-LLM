# Supervisor -> worker(s) (hierarchy module). (issuer, directive_id) is the coupling epoch.
uint8 INTERFACE_VERSION=1

std_msgs/Header header
string issuer
uint64 directive_id
string target                    # agent name or "*"
string kind
builtin_interfaces/Time expires
string payload                   # JSON, app-defined

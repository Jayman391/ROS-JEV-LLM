# Simulated radio link to the base (SYSTEM_MODEL §4.8).
std_msgs/Header header
string drone
float64 quality          # q = 1 / (1 + exp((d - R_c) / s)), 0 inside a blackout
string via               # base | uavK (relay hop)
float64 distance         # m, to the base or the relay

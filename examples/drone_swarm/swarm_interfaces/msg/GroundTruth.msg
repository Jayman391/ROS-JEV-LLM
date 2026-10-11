# The episode's hidden truth (environment → sensor_sim, mission_scorer only).
std_msgs/Header header
string[] target_ids
string[] target_classes          # survivor | vehicle | debris | animal
geometry_msgs/Point[] target_positions
geometry_msgs/Vector3 wind       # m/s, ENU
float64 visibility               # 0.5..1, scales detection probability
string[] fault_drones            # parallel arrays: fault k hits fault_drones[k] from fault_start_s[k]
string[] fault_kinds             # battery_fault | gps_degraded | motor_degraded | comms_blackout
float64[] fault_start_s
geometry_msgs/Point[] blackouts  # comms_blackout discs: x, y, z = radius

# One drone as its flight_controller sees it (5 Hz on /uavK/status and /swarm/status).
# The single source of the deterministic quantities that both the C++ safety layer and the
# agent's masks use (DECISION_MODEL §5).
std_msgs/Header header
string drone
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 energy_frac          # E / E_0
float64 energy_needed_frac   # (E_rtb + reserve) / E_0
float64 margin               # m_E = (E - E_rtb - R) / E_0
float64 link_q
float64 link_lost_s          # seconds with q < 0.1
string task                  # primitive (TAKEOFF, SURVEY, ...) or IDLE / LANDED
string task_detail           # e.g. "sector 2", "T uav1-3"
float64 progress             # 0..1
float64 eta_s
string[] reflexes            # active: geofence, nfz, obstacle, separation, forced_rtb, lost_link
string[] faults              # detected: battery, gps, motor
bool airborne
bool critical                # H8: land_now is unmasked
bool over_obstacle
float64 nearest_obstacle_m
float64 nearest_drone_m

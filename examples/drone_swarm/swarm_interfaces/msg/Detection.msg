# One sighting from the simulated downward detector (SYSTEM_MODEL §4.6).
string track_id                  # local track: sightings within 5 m share it ("uav2-3")
geometry_msgs/Point position     # estimated (noisy) ground position
string[] classes                 # survivor, vehicle, debris, animal, nothing
float64[] scores                 # class scores, sum 1
float64 altitude                 # drone altitude at the sighting
float64 speed                    # drone ground speed at the sighting
